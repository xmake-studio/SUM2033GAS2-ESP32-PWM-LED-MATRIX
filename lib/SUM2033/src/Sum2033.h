// DMA driver for SUM2033 LED panels on the ESP32-S3.
//
// The LCD_CAM peripheral (i8080 mode, 16 bit bus) continuously streams a
// complete panel refresh from RAM with GDMA: config registers, VSYNC, GCLK for
// every row and, at the same time, the pixel data of the frame to show next.
// The CPU is not involved in refreshing the panel at all; it only encodes new
// frames into the second of two DMA buffers, which is swapped in exactly at the
// start of a refresh (no tearing).
//
//   Sum2033 panel;
//   panel.begin();                       // default config = original wiring
//   panel.show(rgb888);                  // 64x64x3 bytes, gamma corrected
//   panel.render([](int x, int y) {      // or compute pixels directly,
//     return sum2033::Rgb16{...};        // 16 bit linear per channel
//   });
//
// See Sum2033Display.h for an Adafruit_GFX canvas on top of this.

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "Sum2033Protocol.h"

class Sum2033 {
 public:
  using Rgb16 = sum2033::Rgb16;

  Sum2033() = default;
  ~Sum2033() { end(); }
  Sum2033(const Sum2033&) = delete;
  Sum2033& operator=(const Sum2033&) = delete;

  // Allocates the DMA buffers (~135 KB of internal RAM for 64x64), configures
  // the panel pins and starts refreshing (black). Returns false on bad config
  // or when out of memory.
  bool begin(const Sum2033Config& config = Sum2033Config());
  void end();
  bool running() const { return running_; }

  int width() const { return layout_.width(); }
  int height() const { return layout_.height(); }

  // --- Frames -----------------------------------------------------------------
  // All of these encode a complete frame into the free DMA buffer and queue it:
  // it becomes visible on the next-but-one VSYNC (the panel is double buffered
  // too). They only block while the previously queued frame has not started yet,
  // i.e. frame rate is limited to the refresh rate. Call them from one task at
  // a time.

  // 8 bit per channel RGB, gamma corrected. stride in bytes, 0 = width * 3.
  void show(const uint8_t* rgb888, size_t stride = 0);
  // RGB565, gamma corrected. stride in pixels, 0 = width.
  void show(const uint16_t* rgb565, size_t stride = 0);
  // 16 bit linear per channel, sent as is. stride in pixels, 0 = width.
  void showLinear(const Rgb16* pixels, size_t stride = 0);
  // Calls shader(x, y) -> Rgb16 (linear) for every pixel, no framebuffer needed.
  template <class Shader>
  void render(Shader&& shader);
  void clear();
  // Queues the currently shown frame again, e.g. to apply setBrightness()
  // without redrawing.
  void refresh();

  // --- Settings ---------------------------------------------------------------
  // Global brightness register, 0..63. Applied with the next frame.
  void setBrightness(uint8_t level) { brightness_ = level > 63 ? 63 : level; }
  uint8_t brightness() const { return brightness_; }
  // Gamma for 8 bit input (show(), Sum2033Display).
  void setGamma(float gamma);
  // 8 bit value -> 16 bit linear value with the current gamma.
  uint16_t toLinear(uint8_t v) const { return gamma_[v]; }

  // --- Timing -----------------------------------------------------------------
  // Waits for the start of the next panel refresh.
  bool waitVSync(uint32_t timeoutMs = 100);
  uint32_t vsyncCount() const { return vsyncs_; }  // refreshes since begin()
  uint32_t frameCount() const { return frames_; }  // frames queued since begin()
  uint32_t clockHz() const { return clockHz_; }    // actual CLK frequency
  float refreshRate() const;                       // Hz, = max frame rate
  size_t memoryUsage() const { return memory_; }   // bytes of DMA memory
  const sum2033::FrameLayout& layout() const { return layout_; }

 private:
  friend struct Sum2033Internal;
  struct DmaDesc;
  struct Buffer {
    uint16_t* header = nullptr;
    uint16_t* rows[sum2033::kMaxScan] = {};
    DmaDesc* descs = nullptr;  // header first, then rows; the last one loops
    size_t descCount = 0;
    DmaDesc* eofDesc = nullptr;  // end of header, raises the VSYNC interrupt
  };

  bool allocBuffers();
  void freeBuffers();
  bool startHardware();
  void stopHardware();
  void routePins(bool toLcd);
  Buffer& acquire();
  void submit(Buffer& buffer);

  Sum2033Config cfg_;
  sum2033::FrameLayout layout_;
  Buffer buffers_[2];
  void* dma_ = nullptr;  // gdma_channel_handle_t
  SemaphoreHandle_t swapSem_ = nullptr;
  SemaphoreHandle_t vsyncSem_ = nullptr;
  volatile int8_t active_ = 0;    // buffer the DMA is playing
  volatile int8_t pending_ = -1;  // buffer queued to be played next
  volatile uint32_t vsyncs_ = 0;
  uint32_t frames_ = 0;
  uint32_t clockHz_ = 0;
  uint32_t prescale_ = 8;
  size_t memory_ = 0;
  uint8_t brightness_ = 63;
  bool running_ = false;
  bool lcdEnabled_ = false;
  uint16_t gamma_[256];
};

template <class Shader>
void Sum2033::render(Shader&& shader) {
  if (!running_) return;
  Buffer& b = acquire();
  for (int row = 0; row < layout_.scan(); ++row) layout_.encodeRow(b.rows[row], row, shader);
  submit(b);
}
