// SUM2033 panel protocol and DMA stream layout.
//
// This file has no hardware dependencies (it also builds on a PC, see test/host),
// it only knows how to lay out the stream of 16-bit words that the ESP32-S3 LCD
// peripheral pushes to the panel, and how to encode pixels into it.
//
// How the panel is driven
// -----------------------
// Every word of the stream is one period of the panel CLK: the LCD peripheral's
// PCLK (WR) output is routed to the CLK pin, and every other panel pin is one bit
// of the word (routed through the GPIO matrix, so the wiring stays the same):
//
//   bit  0..5   R1 G1 B1 R2 G2 B2   column data, sampled on the CLK rising edge
//   bit  6      LAT                 command = number of CLK pulses while LAT is high
//   bit  7      OE / GCLK           PWM clock, toggled "by hand" (high one word, low the next)
//   bit  8..12  A B C D E           row address
//
// One panel refresh (the unit the DMA loops over) is:
//
//   header:  [pre-active + REG1] [pre-active + REG2] [pre-active + REG3] [VSYNC]
//   row 0:   [17 GCLK, previous row address] [gap] [2^colorBits/multiplier GCLK, row 0]
//   ...
//   row N-1
//
// The SUM2033 has two frame buffers in its SRAM: the one being shown and the one
// being written. VSYNC swaps them. The data for the next frame (16 latches of
// 16 bits x chips per row) is shifted in while the GCLK pulses of the current
// row are running, so a complete new frame is uploaded in every refresh and
// does not cost any extra time.

#pragma once

#include <stddef.h>
#include <stdint.h>

// Pin numbers are ESP32 GPIOs, -1 = not connected. Defaults are the wiring of the
// original bit-banged driver (on that panel B and R were swapped).
struct Sum2033Pins {
  int8_t r1 = 12, g1 = 45, b1 = 13;
  int8_t r2 = 10, g2 = 41, b2 = 11;
  int8_t a = 9, b = 38, c = 8, d = 18, e = 39;
  int8_t clk = 7;
  int8_t lat = 17;
  int8_t oe = 14;  // OE is the GCLK input of the SUM2033
};

struct Sum2033Config {
  Sum2033Pins pins;

  uint16_t width = 64;  // pixels, 16 per SUM2033 chip (can be larger for chained panels)
  uint8_t scan = 32;    // multiplexed rows (1/32 scan), panel height = 2 * scan

  uint8_t colorBits = 13;       // 10..13, PWM depth used by the chip
  uint8_t gclkMultiplier = 16;  // 8 or 16, in-chip GCLK multiplier
  uint8_t brightness = 63;      // 0..63, global brightness register

  // Frequency of CLK (one stream word). GCLK runs at half of it. Every refresh
  // carries a full frame, so this directly sets the refresh rate:
  // 10 MHz -> ~290 Hz, 16 MHz -> ~465 Hz, 20 MHz -> ~580 Hz (13 bit, x16).
  // Lower it if you see random speckles (long/bad cables).
  uint32_t clockHz = 10000000;

  float gamma = 2.4f;  // used for 8 bit input (show(), Sum2033Display)

  // Drive strength of the panel pins, 0..3 (ESP-IDF GPIO_DRIVE_CAP_x).
  uint8_t driveStrength = 2;

  // --- Advanced timing ---------------------------------------------------------
  // Number of CLK periods GCLK stays high and then low (1 = GCLK is CLK / 2).
  uint8_t gclkHalfPeriod = 1;
  // CLK periods with GCLK stopped after switching the row address, lets the row
  // drivers settle (the bit-banged driver had a similar pause). Rounded up to even.
  uint8_t rowSwitchGap = 8;
  // true:  shift the next frame in while GCLK is running (fastest).
  // false: shift each row after its GCLK pulses, with GCLK stopped, exactly like
  //        the original bit-banged driver did (slower refresh, fallback).
  bool shiftDuringPwm = true;
};

namespace sum2033 {

// Word bit layout (see top of file).
constexpr int kBitR1 = 0;
constexpr int kBitLat = 6;
constexpr int kBitGclk = 7;
constexpr int kBitAddr = 8;
constexpr int kAddrLines = 5;
constexpr int kDataLines = 6;

constexpr uint16_t kDataMask = (1u << kDataLines) - 1;
constexpr uint16_t kLat = 1u << kBitLat;
constexpr uint16_t kGclk = 1u << kBitGclk;
constexpr uint16_t kAddrMask = ((1u << kAddrLines) - 1) << kBitAddr;

constexpr int kMaxScan = 1 << kAddrLines;
constexpr int kChannelsPerChip = 16;
constexpr int kBitsPerChannel = 16;  // always 16, fills the whole SRAM word
constexpr int kBlankingGclk = 17;    // GCLK pulses at the start of every row

// Commands: number of CLK pulses while LAT is high.
constexpr int kCmdDataLatch = 1;
constexpr int kCmdVsync = 3;
constexpr int kCmdWriteReg1 = 9;
constexpr int kCmdWriteReg2 = 11;
constexpr int kCmdWriteReg3 = 13;
constexpr int kCmdPreActive = 14;

// Idle words between commands, so two LAT-high windows never touch.
constexpr int kGuardWords = 2;

// Linear (already gamma corrected) 16 bit per channel color.
struct Rgb16 {
  uint16_t r, g, b;
};

// gSpread[v][i]: bits of byte v spread over 8 stream words, MSB first, as 4
// little-endian pairs of words: bit (7 - 2i) -> bit 0, bit (6 - 2i) -> bit 16.
// Filled by FrameLayout::configure().
extern uint32_t gSpread[256][4];

class FrameLayout {
 public:
  FrameLayout() = default;
  ~FrameLayout();
  FrameLayout(const FrameLayout&) = delete;
  FrameLayout& operator=(const FrameLayout&) = delete;

  // Validates the configuration and computes the stream layout.
  bool configure(const Sum2033Config& cfg);
  bool valid() const { return dataTemplate_ != nullptr && rowValues_ != nullptr; }

  int width() const { return width_; }
  int height() const { return 2 * scan_; }
  int scan() const { return scan_; }
  int chips() const { return chips_; }
  int gclkPerRow() const { return gclkPerRow_; }

  // Sizes in 16-bit words, all even.
  size_t headerWords() const { return headerWords_; }
  size_t rowWords() const { return rowWords_; }
  size_t frameWords() const { return headerWords_ + rowWords_ * size_t(scan_); }
  size_t dataOffset() const { return dataOffset_; }  // first data word inside a row
  size_t dataWords() const { return dataWords_; }    // data words per row

  // Register values sent in every header (same as the bit-banged driver).
  uint16_t reg1() const { return 0; }
  uint16_t reg2(uint8_t brightness) const;
  uint16_t reg3() const;

  // Config register writes + VSYNC.
  void writeHeader(uint16_t* dst, uint8_t brightness) const;
  // Complete row with its GCLK/LAT/address pattern and black pixels.
  void writeRow(uint16_t* dst, int row) const;

  // Encodes one row of the next frame into a buffer prepared by writeRow().
  // fetch(x, y) must return the Rgb16 color of pixel (x, y); it is called for
  // y = row (upper half) and y = row + scan (lower half).
  template <class Fetch>
  void encodeRow(uint16_t* dst, int row, Fetch&& fetch) const;

 private:
  uint16_t addressBits(int row) const { return uint16_t((row & (kMaxScan - 1)) << kBitAddr); }
  uint16_t gclkAt(size_t pwmWord) const { return ((pwmWord / gclkHalfPeriod_) & 1) ? 0 : kGclk; }
  // Inner loop of every render(), inlined even into -Os code. Encodes one
  // byte (hi or lo) of the 6 channel values into 8 words (4 word pairs).
  __attribute__((always_inline)) static inline void encodeByte(uint32_t* out, const uint32_t* tmpl,
                                                               uint32_t addr, const uint16_t* values,
                                                               int shift);

  int width_ = 0, scan_ = 0, chips_ = 0;
  int colorBits_ = 13, gclkMultiplier_ = 16;
  int gclkPerRow_ = 0;
  size_t gclkHalfPeriod_ = 1;
  size_t headerWords_ = 0;
  size_t blankWords_ = 0, gapWords_ = 0, pwmWords_ = 0;
  size_t dataOffset_ = 0, dataWords_ = 0, rowWords_ = 0;
  bool shiftDuringPwm_ = true;
  // GCLK/LAT bits of the data words of a row (row address not included).
  uint32_t* dataTemplate_ = nullptr;
  // Pixel values of the row being encoded, in shift order.
  uint16_t* rowValues_ = nullptr;
};

// ---------------------------------------------------------------------------

inline void FrameLayout::encodeByte(uint32_t* out, const uint32_t* tmpl, uint32_t addr,
                                    const uint16_t* values, int shift) {
  // Channel by channel with 4 accumulators, so everything stays in registers.
  uint32_t w0 = tmpl[0] | addr, w1 = tmpl[1] | addr, w2 = tmpl[2] | addr, w3 = tmpl[3] | addr;
#pragma GCC unroll 6  // constant shifts
  for (int line = 0; line < kDataLines; ++line) {
    const uint32_t* s = gSpread[(values[line] >> shift) & 0xFF];
    w0 |= s[0] << line;
    w1 |= s[1] << line;
    w2 |= s[2] << line;
    w3 |= s[3] << line;
  }
  out[0] = w0;
  out[1] = w1;
  out[2] = w2;
  out[3] = w3;
}

template <class Fetch>
void FrameLayout::encodeRow(uint16_t* dst, int row, Fetch&& fetch) const {
  // Same order as the bit-banged driver: for every channel, one 16 bit value per
  // chip (chip 0 first, it ends up furthest down the chain), MSB first, and a
  // data latch on the very last bit (already part of the template).
  // Pixels are fetched first so that the encode loop below has all registers
  // for itself.
  uint16_t* v = rowValues_;
  const int lower = row + scan_;
  for (int ch = 0; ch < kChannelsPerChip; ++ch) {
    for (int chip = 0; chip < chips_; ++chip) {
      const int x = ch + chip * kChannelsPerChip;
      const Rgb16 t = fetch(x, row);
      const Rgb16 b = fetch(x, lower);
      v[0] = t.r;  // R1 G1 B1 R2 G2 B2
      v[1] = t.g;
      v[2] = t.b;
      v[3] = b.r;
      v[4] = b.g;
      v[5] = b.b;
      v += kDataLines;
    }
  }

  // Word pairs are written as 32-bit values; dataOffset_ is even and row
  // buffers are 4-byte aligned.
  uint32_t* out = reinterpret_cast<uint32_t*>(dst + dataOffset_);
  const uint32_t* tmpl = dataTemplate_;
  const uint32_t addr = uint32_t(addressBits(row)) * 0x10001u;
  for (const uint16_t* p = rowValues_; p != v; p += kDataLines) {
    encodeByte(out, tmpl, addr, p, 8);
    encodeByte(out + 4, tmpl + 4, addr, p, 0);
    out += 8;
    tmpl += 8;
  }
}

}  // namespace sum2033
