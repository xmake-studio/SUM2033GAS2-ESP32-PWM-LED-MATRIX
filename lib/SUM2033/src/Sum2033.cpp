#include "Sum2033.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Sum2033 needs the LCD_CAM peripheral of the ESP32-S3"
#endif

#include <math.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_private/gdma.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_struct.h>
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif

using namespace sum2033;

static const char* TAG = "sum2033";

// GDMA link descriptor (layout defined by the hardware).
struct Sum2033::DmaDesc {
  volatile uint32_t dw0;  // size[11:0] length[23:12] suc_eof[30] owner[31]
  const void* volatile buffer;
  DmaDesc* volatile next;
};

namespace {

constexpr uint32_t kDescSucEof = 1u << 30;
constexpr uint32_t kDescOwnerDma = 1u << 31;
constexpr size_t kMaxDescBytes = 4092;  // 4095 max, keep chunks word aligned

// LCD clock: PLL_F160M / 2. CLK (PCLK) = kLcdClockHz / prescale.
constexpr uint32_t kLcdClockHz = 80000000;
constexpr uint32_t kLcdClkSelPll160 = 3;

constexpr uint32_t kDmaCaps = MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL;

size_t descsFor(size_t bytes) { return (bytes + kMaxDescBytes - 1) / kMaxDescBytes; }

}  // namespace

// Access to Sum2033 internals from the DMA interrupt and descriptor setup.
struct Sum2033Internal {
  using DmaDesc = Sum2033::DmaDesc;
  static_assert(sizeof(DmaDesc) == 12, "GDMA descriptor must be 12 bytes");

  static bool onEof(gdma_channel_handle_t chan, gdma_event_data_t* event, void* user);
  static size_t linkBuffer(DmaDesc* d, const void* buffer, size_t bytes);
};

bool IRAM_ATTR Sum2033Internal::onEof(gdma_channel_handle_t, gdma_event_data_t* event, void* user) {
  Sum2033* self = static_cast<Sum2033*>(user);
  const DmaDesc* desc = reinterpret_cast<const DmaDesc*>(event->tx_eof_desc_addr);
  int8_t idx;
  if (desc == self->buffers_[0].eofDesc) {
    idx = 0;
  } else if (desc == self->buffers_[1].eofDesc) {
    idx = 1;
  } else {
    return false;
  }
  // The header (config + VSYNC) of buffer idx has just been fetched: a new
  // refresh started, and the DMA will not read the other buffer again until it
  // is queued.
  BaseType_t woken = pdFALSE;
  self->vsyncs_ = self->vsyncs_ + 1;
  self->active_ = idx;
  if (self->pending_ == idx) {
    self->pending_ = -1;
    xSemaphoreGiveFromISR(self->swapSem_, &woken);
  }
  xSemaphoreGiveFromISR(self->vsyncSem_, &woken);
  return woken == pdTRUE;
}

// Fills descriptors for one buffer, each chained to the following one.
size_t Sum2033Internal::linkBuffer(DmaDesc* d, const void* buffer, size_t bytes) {
  const uint8_t* p = static_cast<const uint8_t*>(buffer);
  size_t n = 0;
  while (bytes > 0) {
    const size_t len = bytes > kMaxDescBytes ? kMaxDescBytes : bytes;
    d[n].dw0 = uint32_t(len) | (uint32_t(len) << 12) | kDescOwnerDma;
    d[n].buffer = p;
    d[n].next = &d[n + 1];
    p += len;
    bytes -= len;
    ++n;
  }
  return n;
}

// ---------------------------------------------------------------------------

bool Sum2033::begin(const Sum2033Config& config) {
  end();
  cfg_ = config;
  if (!layout_.configure(cfg_)) {
    ESP_LOGE(TAG, "invalid configuration");
    return false;
  }
  brightness_ = cfg_.brightness > 63 ? 63 : cfg_.brightness;
  setGamma(cfg_.gamma);

  const uint32_t hz = cfg_.clockHz ? cfg_.clockHz : 1;
  prescale_ = (kLcdClockHz + hz / 2) / hz;
  if (prescale_ < 2) prescale_ = 2;
  if (prescale_ > 64) prescale_ = 64;
  clockHz_ = kLcdClockHz / prescale_;

  swapSem_ = xSemaphoreCreateBinary();
  vsyncSem_ = xSemaphoreCreateBinary();
  if (!swapSem_ || !vsyncSem_ || !allocBuffers()) {
    ESP_LOGE(TAG, "out of memory (%u bytes of DMA capable RAM needed)", unsigned(memory_));
    end();
    return false;
  }

  // Both buffers show black and loop on themselves.
  for (int i = 0; i < 2; ++i) {
    Buffer& b = buffers_[i];
    layout_.writeHeader(b.header, brightness_);
    for (int row = 0; row < layout_.scan(); ++row) layout_.writeRow(b.rows[row], row);
    b.descs[b.descCount - 1].next = b.descs;
  }
  active_ = 0;
  pending_ = -1;
  vsyncs_ = 0;
  frames_ = 0;

  if (!startHardware()) {
    end();
    return false;
  }
  running_ = true;
  ESP_LOGI(TAG, "%dx%d, CLK %u Hz, %.1f Hz refresh, %u bytes DMA memory", width(), height(),
           unsigned(clockHz_), refreshRate(), unsigned(memory_));
  return true;
}

void Sum2033::end() {
  running_ = false;
  stopHardware();
  freeBuffers();
  if (swapSem_) vSemaphoreDelete(swapSem_);
  if (vsyncSem_) vSemaphoreDelete(vsyncSem_);
  swapSem_ = vsyncSem_ = nullptr;
}

bool Sum2033::allocBuffers() {
  const size_t headerBytes = layout_.headerWords() * 2;
  const size_t rowBytes = layout_.rowWords() * 2;
  const size_t descCount = descsFor(headerBytes) + layout_.scan() * descsFor(rowBytes);
  memory_ = 2 * (headerBytes + layout_.scan() * rowBytes + descCount * sizeof(DmaDesc));

  for (int i = 0; i < 2; ++i) {
    Buffer& b = buffers_[i];
    b.descs = static_cast<DmaDesc*>(heap_caps_calloc(descCount, sizeof(DmaDesc), kDmaCaps));
    b.header = static_cast<uint16_t*>(heap_caps_malloc(headerBytes, kDmaCaps));
    if (!b.descs || !b.header) return false;
    b.descCount = descCount;
    size_t n = Sum2033Internal::linkBuffer(b.descs, b.header, headerBytes);
    b.eofDesc = &b.descs[n - 1];
    b.eofDesc->dw0 |= kDescSucEof;
    // One allocation per row: ~2 KB blocks are easy to find even in a
    // fragmented heap.
    for (int row = 0; row < layout_.scan(); ++row) {
      b.rows[row] = static_cast<uint16_t*>(heap_caps_malloc(rowBytes, kDmaCaps));
      if (!b.rows[row]) return false;
      n += Sum2033Internal::linkBuffer(&b.descs[n], b.rows[row], rowBytes);
    }
  }
  return true;
}

void Sum2033::freeBuffers() {
  for (int i = 0; i < 2; ++i) {
    Buffer& b = buffers_[i];
    heap_caps_free(b.header);
    for (int row = 0; row < kMaxScan; ++row) heap_caps_free(b.rows[row]);
    heap_caps_free(b.descs);
    b = Buffer();
  }
}

bool Sum2033::startHardware() {
  // GDMA channel feeding the LCD peripheral.
  gdma_channel_handle_t chan = nullptr;
  gdma_channel_alloc_config_t alloc = {};
  alloc.direction = GDMA_CHANNEL_DIRECTION_TX;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
  esp_err_t err = gdma_new_ahb_channel(&alloc, &chan);
#else
  esp_err_t err = gdma_new_channel(&alloc, &chan);
#endif
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "no free GDMA channel (%d)", err);
    return false;
  }
  dma_ = chan;
  gdma_connect(chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  gdma_strategy_config_t strategy = {};
  strategy.owner_check = false;       // descriptors are reused forever
  strategy.auto_update_desc = false;  // and never written back
  gdma_apply_strategy(chan, &strategy);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  gdma_transfer_config_t transfer = {};
  transfer.max_data_burst_size = 16;
  transfer.access_ext_mem = false;
  gdma_config_transfer(chan, &transfer);
#else
  gdma_transfer_ability_t ability = {};
  ability.sram_trans_align = 4;  // enables burst reads
  ability.psram_trans_align = 64;
  gdma_set_transfer_ability(chan, &ability);
#endif
  gdma_tx_event_callbacks_t callbacks = {};
  callbacks.on_trans_eof = &Sum2033Internal::onEof;
  err = gdma_register_tx_event_callbacks(chan, &callbacks, this);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "GDMA interrupt setup failed (%d)", err);
    return false;
  }

  // LCD_CAM in i8080 mode: 16 bit words, output forever while DMA has data.
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  lcdEnabled_ = true;
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(10);

  lcd_cam_lcd_clock_reg_t clock;
  clock.val = 0;
  clock.clk_en = 1;
  clock.lcd_clk_sel = kLcdClkSelPll160;
  clock.lcd_clkm_div_num = 2;  // 160 MHz / 2
  clock.lcd_clkm_div_a = 0;
  clock.lcd_clkm_div_b = 0;
  clock.lcd_ck_idle_edge = 0;  // CLK idles low
  clock.lcd_ck_out_edge = 0;   // CLK low in the first half of a word: the rising
                               // edge is in the middle, data is stable around it
  clock.lcd_clk_equ_sysclk = 0;
  clock.lcd_clkcnt_n = prescale_ - 1;
  LCD_CAM.lcd_clock.val = clock.val;

  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;     // i8080 mode
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;  // no color conversion
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  LCD_CAM.lcd_misc.lcd_bk_en = 1;
  LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 0;
  LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;
  LCD_CAM.lcd_data_dout_mode.val = 0;  // no output delays

  lcd_cam_lcd_user_reg_t user;
  user.val = 0;
  user.lcd_always_out_en = 1;  // length is given by the (endless) DMA list
  user.lcd_2byte_en = 1;       // 16 bit bus, bit n of a word -> LCD_DATA_OUTn
  user.lcd_dout = 1;           // data phase only, no command/dummy phases
  LCD_CAM.lcd_user.val = user.val;
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;

  routePins(true);

  gdma_start(chan, reinterpret_cast<intptr_t>(buffers_[0].descs));
  esp_rom_delay_us(1);  // let the DMA fill the LCD FIFO
  LCD_CAM.lcd_user.lcd_update = 1;
  LCD_CAM.lcd_user.lcd_start = 1;
  return true;
}

void Sum2033::stopHardware() {
  if (lcdEnabled_) {
    LCD_CAM.lcd_user.lcd_start = 0;
    LCD_CAM.lcd_user.lcd_update = 1;
  }
  gdma_channel_handle_t chan = static_cast<gdma_channel_handle_t>(dma_);
  if (chan) {
    gdma_stop(chan);
    gdma_disconnect(chan);
    gdma_del_channel(chan);
    dma_ = nullptr;
  }
  if (lcdEnabled_) {
    routePins(false);
    periph_module_disable(PERIPH_LCD_CAM_MODULE);
    lcdEnabled_ = false;
  }
}

void Sum2033::routePins(bool toLcd) {
  const Sum2033Pins& p = cfg_.pins;
  // Index = bit of the stream word = LCD data line (see Sum2033Protocol.h).
  int8_t bus[kBitAddr + kAddrLines];
  bus[kBitR1 + 0] = p.r1;
  bus[kBitR1 + 1] = p.g1;
  bus[kBitR1 + 2] = p.b1;
  bus[kBitR1 + 3] = p.r2;
  bus[kBitR1 + 4] = p.g2;
  bus[kBitR1 + 5] = p.b2;
  bus[kBitLat] = p.lat;
  bus[kBitGclk] = p.oe;
  bus[kBitAddr + 0] = p.a;
  bus[kBitAddr + 1] = p.b;
  bus[kBitAddr + 2] = p.c;
  bus[kBitAddr + 3] = p.d;
  bus[kBitAddr + 4] = p.e;

  const size_t lines = sizeof(bus) / sizeof(bus[0]);
  for (size_t i = 0; i <= lines; ++i) {
    const int8_t pin = i < lines ? bus[i] : p.clk;
    if (pin < 0) continue;
    const gpio_num_t gpio = static_cast<gpio_num_t>(pin);
    esp_rom_gpio_pad_select_gpio(pin);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    if (toLcd) {
      gpio_set_drive_capability(gpio, static_cast<gpio_drive_cap_t>(cfg_.driveStrength & 3));
      esp_rom_gpio_connect_out_signal(pin, i < lines ? LCD_DATA_OUT0_IDX + i : LCD_PCLK_IDX, false, false);
    } else {
      esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
      gpio_set_level(gpio, 0);
    }
  }
}

// ---------------------------------------------------------------------------

Sum2033::Buffer& Sum2033::acquire() {
  // The frame queued last time starts with the next refresh; until then both
  // buffers are in use.
  while (pending_ >= 0) xSemaphoreTake(swapSem_, pdMS_TO_TICKS(20));
  Buffer& b = buffers_[active_ ^ 1];
  layout_.writeHeader(b.header, brightness_);
  return b;
}

void Sum2033::submit(Buffer& b) {
  const int8_t idx = int8_t(&b - buffers_);
  DmaDesc* head = b.descs;
  // Once it is reached, the new frame repeats until the next one is queued.
  b.descs[b.descCount - 1].next = head;
  pending_ = idx;
  // Continue with the new frame when the current refresh ends.
  Buffer& playing = buffers_[idx ^ 1];
  playing.descs[playing.descCount - 1].next = head;
  ++frames_;
}

void Sum2033::show(const uint8_t* rgb888, size_t stride) {
  if (!stride) stride = size_t(width()) * 3;
  const uint16_t* g = gamma_;
  render([=](int x, int y) -> Rgb16 {
    const uint8_t* p = rgb888 + size_t(y) * stride + size_t(x) * 3;
    return Rgb16{g[p[0]], g[p[1]], g[p[2]]};
  });
}

void Sum2033::show(const uint16_t* rgb565, size_t stride) {
  if (!stride) stride = size_t(width());
  const uint16_t* g = gamma_;
  render([=](int x, int y) -> Rgb16 {
    const uint16_t c = rgb565[size_t(y) * stride + size_t(x)];
    const uint8_t r = (c >> 11) & 0x1F, gr = (c >> 5) & 0x3F, b = c & 0x1F;
    return Rgb16{g[(r << 3) | (r >> 2)], g[(gr << 2) | (gr >> 4)], g[(b << 3) | (b >> 2)]};
  });
}

void Sum2033::showLinear(const Rgb16* pixels, size_t stride) {
  if (!stride) stride = size_t(width());
  render([=](int x, int y) -> Rgb16 { return pixels[size_t(y) * stride + size_t(x)]; });
}

void Sum2033::clear() {
  render([](int, int) -> Rgb16 { return Rgb16{0, 0, 0}; });
}

void Sum2033::refresh() {
  if (!running_) return;
  Buffer& b = acquire();
  // Nothing is queued now, so the playing buffer holds the latest frame.
  const Buffer& current = buffers_[active_];
  for (int row = 0; row < layout_.scan(); ++row)
    memcpy(b.rows[row], current.rows[row], layout_.rowWords() * 2);
  submit(b);
}

void Sum2033::setGamma(float gamma) {
  for (int i = 0; i < 256; ++i) gamma_[i] = uint16_t(lroundf(powf(i / 255.0f, gamma) * 65535.0f));
}

bool Sum2033::waitVSync(uint32_t timeoutMs) {
  if (!running_) return false;
  const uint32_t start = vsyncs_;
  const TickType_t until = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
  while (vsyncs_ == start) {
    const TickType_t now = xTaskGetTickCount();
    if (int32_t(until - now) <= 0) return false;
    xSemaphoreTake(vsyncSem_, until - now);
  }
  return true;
}

float Sum2033::refreshRate() const {
  return layout_.valid() ? float(clockHz_) / float(layout_.frameWords()) : 0.0f;
}
