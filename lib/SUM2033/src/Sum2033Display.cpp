#include "Sum2033Display.h"

#include <string.h>

#include <esp_heap_caps.h>

namespace {

void expand565(uint16_t color, uint8_t* rgb) {
  const uint8_t r = (color >> 11) & 0x1F, g = (color >> 5) & 0x3F, b = color & 0x1F;
  rgb[0] = uint8_t((r << 3) | (r >> 2));
  rgb[1] = uint8_t((g << 2) | (g >> 4));
  rgb[2] = uint8_t((b << 3) | (b >> 2));
}

}  // namespace

Sum2033Display::Sum2033Display(uint16_t width, uint16_t height) : Adafruit_GFX(width, height) {}

Sum2033Display::~Sum2033Display() { end(); }

bool Sum2033Display::begin(Sum2033Config config) {
  end();
  config.width = WIDTH;
  config.scan = uint8_t(HEIGHT / 2);
  const size_t bytes = size_t(WIDTH) * HEIGHT * 3;
  fb_ = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (!fb_) fb_ = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  if (!fb_) return false;
  memset(fb_, 0, bytes);
  if (HEIGHT % 2 != 0 || !panel_.begin(config)) {
    end();
    return false;
  }
  return true;
}

void Sum2033Display::end() {
  panel_.end();
  heap_caps_free(fb_);
  fb_ = nullptr;
}

void Sum2033Display::flip() {
  if (fb_) panel_.show(fb_, size_t(WIDTH) * 3);
}

uint8_t* Sum2033Display::pixelAt(int16_t x, int16_t y) {
  if (!fb_ || x < 0 || y < 0 || x >= _width || y >= _height) return nullptr;
  int16_t t;
  switch (rotation) {
    case 1:
      t = x;
      x = WIDTH - 1 - y;
      y = t;
      break;
    case 2:
      x = WIDTH - 1 - x;
      y = HEIGHT - 1 - y;
      break;
    case 3:
      t = x;
      x = y;
      y = HEIGHT - 1 - t;
      break;
  }
  return fb_ + (size_t(y) * WIDTH + x) * 3;
}

void Sum2033Display::drawPixel(int16_t x, int16_t y, uint16_t color) {
  uint8_t* p = pixelAt(x, y);
  if (p) expand565(color, p);
}

void Sum2033Display::setPixel(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t* p = pixelAt(x, y);
  if (!p) return;
  p[0] = r;
  p[1] = g;
  p[2] = b;
}

void Sum2033Display::fillScreen(uint16_t color) {
  if (!fb_) return;
  const size_t bytes = size_t(WIDTH) * HEIGHT * 3;
  if (color == 0) {
    memset(fb_, 0, bytes);
    return;
  }
  uint8_t rgb[3];
  expand565(color, rgb);
  for (size_t i = 0; i < bytes; i += 3) memcpy(fb_ + i, rgb, 3);
}
