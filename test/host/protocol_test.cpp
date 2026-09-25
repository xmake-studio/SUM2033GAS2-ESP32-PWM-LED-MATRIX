// Host-side check of the DMA stream produced by lib/SUM2033.
//
// A behavioral model of the SUM2033 chain (shift registers, LAT command
// decoding, config registers, double-buffered SRAM, GCLK/row tracking) is fed
// with
//   1. the original bit-banged driver (copied from the first version of
//      src/main.cpp, GPIO calls replaced by the model), and
//   2. the word stream built by sum2033::FrameLayout, clocked the way the
//      ESP32-S3 LCD peripheral outputs it,
// and the results are compared.
//
// Build & run (from the repository root):
//   g++ -std=c++11 -O2 -Wall -Wextra -Ilib/SUM2033/src test/host/protocol_test.cpp
//       lib/SUM2033/src/Sum2033Protocol.cpp -o protocol_test && ./protocol_test

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <utility>
#include <vector>

#include "Sum2033Protocol.h"

using namespace sum2033;

// ---------------------------------------------------------------------------
// Panel model

struct PanelModel {
  int chips, scan;

  // Pins
  int data[6] = {0, 0, 0, 0, 0, 0};  // R1 G1 B1 R2 G2 B2
  bool lat = false, gclk = false;
  int addr = 0;

  // Shift chain: sr[line][p], p = 0 is the chip next to the connector.
  std::vector<std::vector<uint16_t> > sr;
  int latClocks = 0;

  bool preActive = false;
  uint16_t regs[3] = {0, 0, 0};
  int regWrites[3] = {0, 0, 0};

  // SRAM[buffer][((row * 16 + ch) * chips + chip) * 6 + line]
  std::vector<uint16_t> sram[2];
  int front = 0;
  int latchIndex = 0;

  int vsyncs = 0;
  int latchesThisRefresh = 0;
  std::vector<int> latchesPerRefresh;
  std::vector<std::pair<int, int> > gclkRuns;  // (address, pulses) since last VSYNC
  std::vector<std::vector<std::pair<int, int> > > refreshRuns;
  std::vector<std::string> errors;

  PanelModel(int chips_, int scan_) : chips(chips_), scan(scan_) {
    sr.assign(6, std::vector<uint16_t>(chips, 0));
    for (int b = 0; b < 2; ++b) sram[b].assign(size_t(scan) * 16 * chips * 6, 0xDEAD);
  }

  void error(const std::string& e) {
    if (errors.size() < 20) errors.push_back(e);
  }

  void clkRise() {
    for (int line = 0; line < 6; ++line) {
      int carry = data[line];
      for (int p = 0; p < chips; ++p) {
        const int out = sr[line][p] >> 15;
        sr[line][p] = uint16_t((sr[line][p] << 1) | carry);
        carry = out;
      }
    }
    if (lat) ++latClocks;
  }

  void setLat(bool v) {
    if (lat && !v) command(latClocks);
    if (!v) latClocks = 0;
    lat = v;
  }

  void setGclk(bool v) {
    if (!gclk && v) {
      if (!gclkRuns.empty() && gclkRuns.back().first == addr)
        ++gclkRuns.back().second;
      else
        gclkRuns.push_back(std::make_pair(addr, 1));
    }
    gclk = v;
  }

  // Value held by logical chip k (the k-th 16 bit word shifted in).
  uint16_t chipValue(int line, int k) const { return sr[line][chips - 1 - k]; }

  void command(int n) {
    const bool wasPreActive = preActive;
    preActive = false;
    switch (n) {
      case kCmdDataLatch: {
        const int row = latchIndex / 16, ch = latchIndex % 16;
        if (row >= scan) {
          error("too many data latches");
          break;
        }
        std::vector<uint16_t>& back = sram[1 - front];
        for (int k = 0; k < chips; ++k)
          for (int line = 0; line < 6; ++line)
            back[((size_t(row) * 16 + ch) * chips + k) * 6 + line] = chipValue(line, k);
        ++latchIndex;
        ++latchesThisRefresh;
        break;
      }
      case kCmdVsync:
        front = 1 - front;
        latchIndex = 0;
        ++vsyncs;
        latchesPerRefresh.push_back(latchesThisRefresh);
        latchesThisRefresh = 0;
        refreshRuns.push_back(gclkRuns);
        gclkRuns.clear();
        break;
      case kCmdPreActive:
        preActive = true;
        break;
      case kCmdWriteReg1:
      case kCmdWriteReg2:
      case kCmdWriteReg3: {
        if (!wasPreActive) error("register write without pre-active");
        const int idx = (n - kCmdWriteReg1) / 2;
        const uint16_t v = chipValue(0, 0);
        for (int k = 0; k < chips; ++k)
          for (int line = 0; line < 6; ++line)
            if (chipValue(line, k) != v) error("register value differs between chips/lines");
        regs[idx] = v;
        ++regWrites[idx];
        break;
      }
      default: {
        char buf[64];
        snprintf(buf, sizeof buf, "unknown command: %d clocks", n);
        error(buf);
      }
    }
  }

  uint16_t displayed(int x, int y, int color) const {
    const int row = y % scan, half = y / scan, ch = x % 16, k = x / 16;
    return sram[front][((size_t(row) * 16 + ch) * chips + k) * 6 + half * 3 + color];
  }
};

// ---------------------------------------------------------------------------
// 1. Original bit-banged driver, GPIO replaced by the model.

namespace ref {

enum { B1 = 13, B2 = 11, G1 = 45, G2 = 41, R1 = 12, R2 = 10 };
enum { CH_A = 9, CH_B = 38, CH_C = 8, CH_D = 18, CH_E = 39, CLK = 7, LAT = 17, OE = 14 };

const int MATRIX_WIDTH = 64, MATRIX_SCAN = 32;
uint8_t cfg_brightness = 63, cfg_color_bits = 13, cfg_gclk_mul = 16;
const int storedColorBits = 16, channelsPerChip = 16, chips_count = MATRIX_WIDTH / channelsPerChip;

PanelModel* m;

void gpio_set_level(int pin, int v) {
  v = v ? 1 : 0;
  switch (pin) {
    case R1: m->data[0] = v; break;
    case G1: m->data[1] = v; break;
    case B1: m->data[2] = v; break;
    case R2: m->data[3] = v; break;
    case G2: m->data[4] = v; break;
    case B2: m->data[5] = v; break;
    case CH_A: m->addr = (m->addr & ~1) | v; break;
    case CH_B: m->addr = (m->addr & ~2) | v << 1; break;
    case CH_C: m->addr = (m->addr & ~4) | v << 2; break;
    case CH_D: m->addr = (m->addr & ~8) | v << 3; break;
    case CH_E: m->addr = (m->addr & ~16) | v << 4; break;
    case LAT: m->setLat(v); break;
    case OE: m->setGclk(v); break;
    default: abort();
  }
}
#define CLK_PULSE m->clkRise();
#define GCLK_PULSE \
  m->setGclk(1);   \
  m->setGclk(0);

void selectRow(int row) {
  gpio_set_level(CH_A, (row >> 0) & 1);
  gpio_set_level(CH_B, (row >> 1) & 1);
  gpio_set_level(CH_C, (row >> 2) & 1);
  gpio_set_level(CH_D, (row >> 3) & 1);
  gpio_set_level(CH_E, (row >> 4) & 1);
}
void sendPwmClock(int clocks) {
  while (clocks--) {
    GCLK_PULSE
  }
}
void sendLatch(unsigned char clocks) {
  gpio_set_level(LAT, 1);
  while (clocks--) {
    CLK_PULSE
  }
  gpio_set_level(LAT, 0);
}
void sendConfiguration(unsigned char latches, unsigned int data) {
  unsigned char num = chips_count;
  latches = 16 - latches;
  gpio_set_level(LAT, 0);
  while (num--) {
    for (unsigned char x = 0; x < 16; x++) {
      unsigned int dataMask = 0x8000 >> x;
      bool en = data & dataMask;
      gpio_set_level(R1, en);
      gpio_set_level(G1, en);
      gpio_set_level(B1, en);
      gpio_set_level(R2, en);
      gpio_set_level(G2, en);
      gpio_set_level(B2, en);
      if (num == 0 && x == latches) gpio_set_level(LAT, 1);
      CLK_PULSE
    }
    gpio_set_level(LAT, 0);
  }
}
void sendConfigRegs() {
  sendLatch(14);
  sendConfiguration(9, 0b0);
  uint16_t reg2 = cfg_brightness & 0b111111;
  reg2 |= (MATRIX_SCAN - 1) << 6;
  reg2 |= (~(cfg_color_bits - 10) & 0b11) << 11;
  sendLatch(14);
  sendConfiguration(11, reg2);
  uint16_t reg3 = 0;
  reg3 |= (cfg_gclk_mul >> 4) << 2;
  reg3 |= 1 << 4;
  sendLatch(14);
  sendConfiguration(13, reg3);
}
struct pixel {
  uint16_t r, g, b;
};
uint16_t gammaCorrectGrayscale(uint16_t input, float gamma = 2.4) {
  float col = (float)input / 65535.0f;
  col = powf(col, gamma);
  return (uint16_t)(col * 65535.0f);
}
pixel getPixel(uint8_t x, uint8_t y) {
  pixel pix = {0, 0, 0};
  pix.r = gammaCorrectGrayscale(x * 1040);
  pix.g = gammaCorrectGrayscale(y * 1040);
  return pix;
}
void loop() {
  sendConfigRegs();
  static int c = -1;
  c++;
  bool fullUpdate = c < 2;
  int gclk_pulses_per_line = pow(2, cfg_color_bits) / cfg_gclk_mul;
  sendLatch(3);
  for (uint8_t line = 0; line < MATRIX_SCAN; line++) {
    sendPwmClock(17);
    selectRow(line);
    sendPwmClock(gclk_pulses_per_line);
    if (!fullUpdate) continue;
    for (uint8_t channel = 0; channel < channelsPerChip; channel++) {
      for (uint8_t chip = 0; chip < chips_count; chip++) {
        const auto x = channel + channelsPerChip * chip;
        const auto p1 = getPixel(x, line);
        const auto p2 = getPixel(x, line + MATRIX_SCAN);
        for (int8_t bit = storedColorBits - 1; bit >= 0; bit--) {
          const uint16_t msk = 1UL << bit;
          gpio_set_level(R1, p1.r & msk);
          gpio_set_level(G1, p1.g & msk);
          gpio_set_level(B1, p1.b & msk);
          gpio_set_level(R2, p2.r & msk);
          gpio_set_level(G2, p2.g & msk);
          gpio_set_level(B2, p2.b & msk);
          if (chip == chips_count - 1 && bit == 0) gpio_set_level(LAT, 1);
          CLK_PULSE
        }
      }
      gpio_set_level(LAT, 0);
    }
  }
}

}  // namespace ref

// ---------------------------------------------------------------------------
// 2. DMA stream: one word per PCLK period. All outputs change together at the
// start of the word (PCLK falling edge), CLK rises in the middle of the word.

struct StreamPlayer {
  PanelModel& m;
  uint16_t prev = 0;
  size_t addrOnGclkEdge = 0;
  explicit StreamPlayer(PanelModel& model) : m(model) {}

  void play(const uint16_t* w, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      const uint16_t v = w[i];
      const int a = (v & kAddrMask) >> kBitAddr;
      if (!(prev & kGclk) && (v & kGclk) && a != m.addr) ++addrOnGclkEdge;
      for (int line = 0; line < 6; ++line) m.data[line] = (v >> (kBitR1 + line)) & 1;
      m.addr = a;
      m.setLat(v & kLat);
      m.setGclk(v & kGclk);
      m.clkRise();
      prev = v;
    }
  }
};

struct Frame {
  std::vector<uint16_t> words;  // header + rows, contiguous
  size_t headerWords;
};

template <class Fetch>
Frame buildFrame(const FrameLayout& layout, uint8_t brightness, Fetch fetch) {
  Frame f;
  f.headerWords = layout.headerWords();
  f.words.assign(layout.frameWords(), 0);
  layout.writeHeader(&f.words[0], brightness);
  for (int r = 0; r < layout.scan(); ++r) {
    uint16_t* row = &f.words[layout.headerWords() + size_t(r) * layout.rowWords()];
    layout.writeRow(row, r);
    layout.encodeRow(row, r, fetch);
  }
  return f;
}

// ---------------------------------------------------------------------------

static int failures = 0;
#define CHECK(cond, ...)                   \
  do {                                     \
    if (!(cond)) {                         \
      ++failures;                          \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                 \
      printf("\n");                        \
    }                                      \
  } while (0)

static uint32_t rngState = 12345;
static uint16_t rnd16() {
  rngState = rngState * 1664525u + 1013904223u;
  return uint16_t(rngState >> 16);
}

// Expected merged GCLK runs of one refresh.
static std::vector<std::pair<int, int> > expectedRuns(int scan, int gpr) {
  std::vector<std::pair<int, int> > runs;
  runs.push_back(std::make_pair(scan - 1, kBlankingGclk));
  for (int r = 0; r < scan; ++r) runs.push_back(std::make_pair(r, gpr + (r + 1 < scan ? kBlankingGclk : 0)));
  if (scan == 1) {  // single row: everything on address 0
    runs.clear();
    runs.push_back(std::make_pair(0, kBlankingGclk + gpr));
  }
  return runs;
}

static void testAgainstReference() {
  printf("reference (bit-banged driver) vs DMA stream, 64x64, 13 bit, x16\n");
  Sum2033Config cfg;
  FrameLayout layout;
  CHECK(layout.configure(cfg), "configure");

  PanelModel refModel(4, 32);
  ref::m = &refModel;
  for (int i = 0; i < 3; ++i) ref::loop();
  CHECK(refModel.errors.empty(), "reference errors: %s", refModel.errors.empty() ? "" : refModel.errors[0].c_str());

  PanelModel dmaModel(4, 32);
  StreamPlayer player(dmaModel);
  Frame f = buildFrame(layout, cfg.brightness, [](int x, int y) {
    const ref::pixel p = ref::getPixel(uint8_t(x), uint8_t(y));
    const Rgb16 c = {p.r, p.g, p.b};
    return c;
  });
  for (int i = 0; i < 3; ++i) player.play(&f.words[0], f.words.size());
  CHECK(dmaModel.errors.empty(), "DMA errors: %s", dmaModel.errors.empty() ? "" : dmaModel.errors[0].c_str());
  CHECK(player.addrOnGclkEdge == 0, "row address changes on a GCLK edge");

  for (int i = 0; i < 3; ++i) {
    CHECK(refModel.regs[i] == dmaModel.regs[i], "REG%d: ref 0x%04x dma 0x%04x", i + 1, refModel.regs[i],
          dmaModel.regs[i]);
  }
  printf("  regs: 0x%04x 0x%04x 0x%04x\n", dmaModel.regs[0], dmaModel.regs[1], dmaModel.regs[2]);
  CHECK(refModel.vsyncs == 3 && dmaModel.vsyncs == 3, "vsyncs %d %d", refModel.vsyncs, dmaModel.vsyncs);

  // Steady state refresh: identical GCLK/row sequence.
  CHECK(refModel.refreshRuns.size() == 3 && dmaModel.refreshRuns.size() == 3, "refresh count");
  if (refModel.refreshRuns.size() == 3 && dmaModel.refreshRuns.size() == 3) {
    CHECK(refModel.refreshRuns[2] == dmaModel.refreshRuns[2], "GCLK/row sequence differs");
    CHECK(dmaModel.refreshRuns[2] == expectedRuns(32, 512), "unexpected GCLK/row sequence");
    CHECK(dmaModel.latchesPerRefresh[2] == 32 * 16, "latches per refresh %d", dmaModel.latchesPerRefresh[2]);
  }
  // The GCLK sequence also has to be complete after the last refresh.
  CHECK(dmaModel.gclkRuns == expectedRuns(32, 512), "last refresh GCLK/row sequence");

  // Frame uploaded in refresh 1 is displayed after VSYNC of refresh 2 (and the
  // reference driver uploaded the same frame in refreshes 0 and 1).
  int diffs = 0;
  for (int y = 0; y < 64; ++y)
    for (int x = 0; x < 64; ++x)
      for (int c = 0; c < 3; ++c)
        if (refModel.displayed(x, y, c) != dmaModel.displayed(x, y, c)) ++diffs;
  CHECK(diffs == 0, "%d displayed values differ", diffs);
  const ref::pixel p = ref::getPixel(40, 50);
  CHECK(dmaModel.displayed(40, 50, 0) == p.r && dmaModel.displayed(40, 50, 1) == p.g &&
            dmaModel.displayed(40, 50, 2) == p.b,
        "pixel (40,50)");
  // Both SRAM buffers contain the frame (it is re-sent every refresh).
  CHECK(dmaModel.sram[0] == dmaModel.sram[1], "SRAM buffers differ");
}

static void testConfig(int width, int scan, int bits, int mul, bool overlap, int halfPeriod, int gap,
                       int brightness) {
  printf("width %d scan %d bits %d x%d %s halfPeriod %d gap %d\n", width, scan, bits, mul,
         overlap ? "overlapped" : "sequential", halfPeriod, gap);
  Sum2033Config cfg;
  cfg.width = uint16_t(width);
  cfg.scan = uint8_t(scan);
  cfg.colorBits = uint8_t(bits);
  cfg.gclkMultiplier = uint8_t(mul);
  cfg.shiftDuringPwm = overlap;
  cfg.gclkHalfPeriod = uint8_t(halfPeriod);
  cfg.rowSwitchGap = uint8_t(gap);
  FrameLayout layout;
  CHECK(layout.configure(cfg), "configure");
  CHECK(layout.headerWords() % 2 == 0 && layout.rowWords() % 2 == 0 && layout.dataOffset() % 2 == 0,
        "odd sizes");
  const int chips = width / 16, height = 2 * scan;

  std::vector<Rgb16> img(size_t(width) * height);
  for (size_t i = 0; i < img.size(); ++i) {
    img[i].r = rnd16();
    img[i].g = rnd16();
    img[i].b = rnd16();
  }
  img[0].r = img[0].g = img[0].b = 0xFFFF;
  img[1].r = img[1].g = img[1].b = 0;

  PanelModel model(chips, scan);
  StreamPlayer player(model);
  // Start with a black frame, then switch to the image (like the driver does).
  Frame black = buildFrame(layout, uint8_t(brightness), [](int, int) {
    const Rgb16 c = {0, 0, 0};
    return c;
  });
  Frame f = buildFrame(layout, uint8_t(brightness), [&](int x, int y) { return img[size_t(y) * width + x]; });
  player.play(&black.words[0], black.words.size());
  player.play(&f.words[0], f.words.size());
  player.play(&f.words[0], f.headerWords);  // VSYNC: frame becomes visible

  CHECK(model.errors.empty(), "errors: %s", model.errors.empty() ? "" : model.errors[0].c_str());
  CHECK(player.addrOnGclkEdge == 0, "row address changes on a GCLK edge");
  const uint16_t reg2 = uint16_t((brightness & 63) | ((scan - 1) << 6) | ((~(bits - 10) & 3) << 11));
  const uint16_t reg3 = uint16_t(((mul >> 4) << 2) | (1 << 4));
  CHECK(model.regs[0] == 0 && model.regs[1] == reg2 && model.regs[2] == reg3, "regs 0x%04x 0x%04x 0x%04x",
        model.regs[0], model.regs[1], model.regs[2]);
  CHECK(model.regWrites[0] == 3 && model.regWrites[1] == 3 && model.regWrites[2] == 3, "reg write count");

  int diffs = 0;
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      const Rgb16& c = img[size_t(y) * width + x];
      if (model.displayed(x, y, 0) != c.r || model.displayed(x, y, 1) != c.g || model.displayed(x, y, 2) != c.b)
        ++diffs;
    }
  CHECK(diffs == 0, "%d pixels differ", diffs);

  const int gpr = (1 << bits) / mul;
  CHECK(model.refreshRuns.size() == 3, "refreshes %zu", model.refreshRuns.size());
  if (model.refreshRuns.size() == 3) {
    CHECK(model.refreshRuns[2] == expectedRuns(scan, gpr), "GCLK/row sequence");
    CHECK(model.latchesPerRefresh[2] == scan * 16, "latches per refresh");
  }

  // GCLK duty: count high/low words inside the PWM section of row 3.
  if (scan > 3) {
    const uint16_t* row = &f.words[layout.headerWords() + 3 * layout.rowWords()];
    int high = 0;
    for (size_t i = 0; i < layout.rowWords(); ++i) high += (row[i] & kGclk) ? 1 : 0;
    CHECK(high == (kBlankingGclk + gpr) * halfPeriod, "GCLK high words %d", high);
  }
  const double words = double(layout.frameWords());
  printf("  %zu words/refresh (header %zu, row %zu), %.0f Hz @ 10 MHz, %.0f Hz @ 20 MHz\n",
         layout.frameWords(), layout.headerWords(), layout.rowWords(), 10e6 / words, 20e6 / words);
}

static void testInvalid() {
  printf("invalid configurations\n");
  FrameLayout layout;
  Sum2033Config cfg;
  cfg.width = 60;
  CHECK(!layout.configure(cfg), "width 60");
  cfg = Sum2033Config();
  cfg.colorBits = 14;
  CHECK(!layout.configure(cfg), "14 bits");
  cfg = Sum2033Config();
  cfg.gclkMultiplier = 4;
  CHECK(!layout.configure(cfg), "x4");
  cfg = Sum2033Config();
  cfg.scan = 33;
  CHECK(!layout.configure(cfg), "scan 33");
}

int main() {
  setvbuf(stdout, NULL, _IONBF, 0);
  testAgainstReference();
  for (int bits = 10; bits <= 13; ++bits)
    for (int mul = 8; mul <= 16; mul += 8)
      for (int overlap = 0; overlap < 2; ++overlap) testConfig(64, 32, bits, mul, overlap != 0, 1, 8, 63);
  testConfig(64, 32, 13, 16, true, 2, 8, 17);
  testConfig(64, 32, 10, 16, true, 8, 2, 0);
  testConfig(64, 32, 13, 16, true, 1, 0, 40);
  testConfig(128, 32, 13, 16, true, 1, 8, 63);
  testConfig(32, 16, 12, 8, false, 1, 4, 5);
  testConfig(16, 1, 13, 16, true, 1, 8, 63);
  testInvalid();
  printf(failures ? "\n%d FAILURES\n" : "\nALL OK\n", failures);
  return failures ? 1 : 0;
}
