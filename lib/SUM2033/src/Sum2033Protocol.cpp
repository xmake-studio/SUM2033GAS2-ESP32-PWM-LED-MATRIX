#include "Sum2033Protocol.h"

#include <stdlib.h>

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#endif

namespace sum2033 {

uint32_t gSpread[256][4];

namespace {

void* allocInternal(size_t bytes) {
#ifdef ESP_PLATFORM
  // Read for every encoded word, keep it out of PSRAM.
  return heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
#else
  return malloc(bytes);
#endif
}

size_t roundUpEven(size_t n) { return (n + 1) & ~size_t(1); }

void buildSpreadTable() {
  for (unsigned v = 0; v < 256; ++v) {
    for (unsigned i = 0; i < 4; ++i) {
      gSpread[v][i] = ((v >> (7 - 2 * i)) & 1u) | (((v >> (6 - 2 * i)) & 1u) << 16);
    }
  }
}

}  // namespace

FrameLayout::~FrameLayout() {
  free(dataTemplate_);
  free(rowValues_);
}

bool FrameLayout::configure(const Sum2033Config& cfg) {
  free(dataTemplate_);
  free(rowValues_);
  dataTemplate_ = nullptr;
  rowValues_ = nullptr;

  if (cfg.width == 0 || cfg.width % kChannelsPerChip != 0) return false;
  if (cfg.scan < 1 || cfg.scan > kMaxScan) return false;
  if (cfg.colorBits < 10 || cfg.colorBits > 13) return false;
  if (cfg.gclkMultiplier != 8 && cfg.gclkMultiplier != 16) return false;
  if (cfg.gclkHalfPeriod < 1) return false;

  width_ = cfg.width;
  scan_ = cfg.scan;
  chips_ = width_ / kChannelsPerChip;
  colorBits_ = cfg.colorBits;
  gclkMultiplier_ = cfg.gclkMultiplier;
  gclkPerRow_ = (1 << colorBits_) / gclkMultiplier_;
  gclkHalfPeriod_ = cfg.gclkHalfPeriod;
  shiftDuringPwm_ = cfg.shiftDuringPwm;

  const size_t latchWords = size_t(chips_) * kBitsPerChannel;
  headerWords_ = roundUpEven(kGuardWords + 3 * (kCmdPreActive + latchWords + kGuardWords) +
                             kCmdVsync + kGuardWords);

  blankWords_ = 2 * gclkHalfPeriod_ * kBlankingGclk;
  gapWords_ = roundUpEven(cfg.rowSwitchGap < 2 ? 2 : cfg.rowSwitchGap);
  pwmWords_ = 2 * gclkHalfPeriod_ * size_t(gclkPerRow_);
  dataWords_ = kChannelsPerChip * latchWords;

  const size_t pwmStart = blankWords_ + gapWords_;
  if (shiftDuringPwm_) {
    dataOffset_ = pwmStart;
    rowWords_ = pwmStart + (pwmWords_ > dataWords_ ? pwmWords_ : dataWords_);
  } else {
    dataOffset_ = pwmStart + pwmWords_;
    rowWords_ = dataOffset_ + dataWords_;
  }

  buildSpreadTable();

  dataTemplate_ = static_cast<uint32_t*>(allocInternal(dataWords_ * sizeof(uint16_t)));
  rowValues_ = static_cast<uint16_t*>(allocInternal(size_t(width_) * kDataLines * sizeof(uint16_t)));
  if (!dataTemplate_ || !rowValues_) {
    free(dataTemplate_);
    free(rowValues_);
    dataTemplate_ = nullptr;
    rowValues_ = nullptr;
    return false;
  }
  for (size_t k = 0; k < dataWords_; k += 2) {
    uint32_t pair = 0;
    for (size_t j = 0; j < 2; ++j) {
      const size_t pwmWord = dataOffset_ + k + j - pwmStart;
      uint32_t w = pwmWord < pwmWords_ ? gclkAt(pwmWord) : 0;
      if ((k + j) % latchWords == latchWords - 1) w |= kLat;
      pair |= w << (16 * j);
    }
    dataTemplate_[k / 2] = pair;
  }
  return true;
}

uint16_t FrameLayout::reg2(uint8_t brightness) const {
  uint16_t r = brightness & 0x3F;                          // 6 bits global brightness
  r |= uint16_t((scan_ - 1) & 0x1F) << 6;                  // 5 bits scan lines - 1
  r |= uint16_t(~(colorBits_ - 10) & 0x3) << 11;           // 2 bits color depth, inverted
  return r;
}

uint16_t FrameLayout::reg3() const {
  uint16_t r = uint16_t(gclkMultiplier_ >> 4) << 2;  // x16 GCLK multiplier (x8 when 0)
  r |= 1u << 4;                                      // must be 1 for the matrix to work
  return r;
}

void FrameLayout::writeHeader(uint16_t* dst, uint8_t brightness) const {
  // The previous refresh ended on the last row, keep it selected.
  const uint16_t idle = addressBits(scan_ - 1);
  const size_t latchWords = size_t(chips_) * kBitsPerChannel;
  uint16_t* p = dst;

  // Separates us from the data latch that ended the previous refresh.
  for (int i = 0; i < kGuardWords; ++i) *p++ = idle;

  const uint16_t values[3] = {reg1(), reg2(brightness), reg3()};
  const size_t commands[3] = {kCmdWriteReg1, kCmdWriteReg2, kCmdWriteReg3};
  for (int r = 0; r < 3; ++r) {
    for (int i = 0; i < kCmdPreActive; ++i) *p++ = idle | kLat;
    // Same value for every chip and data line, MSB first, LAT high during the
    // last `command` clocks.
    for (size_t i = 0; i < latchWords; ++i) {
      uint16_t w = idle;
      if ((values[r] >> (15 - i % 16)) & 1) w |= kDataMask;
      if (i >= latchWords - commands[r]) w |= kLat;
      *p++ = w;
    }
    for (int i = 0; i < kGuardWords; ++i) *p++ = idle;
  }

  // Swap the chip's frame buffers: the frame uploaded during the previous
  // refresh becomes visible, and the rows below write the other buffer.
  for (int i = 0; i < kCmdVsync; ++i) *p++ = idle | kLat;
  while (p < dst + headerWords_) *p++ = idle;
}

void FrameLayout::writeRow(uint16_t* dst, int row) const {
  const uint16_t prev = addressBits((row + scan_ - 1) % scan_);
  const uint16_t cur = addressBits(row);
  const size_t pwmStart = blankWords_ + gapWords_;
  const size_t latchWords = size_t(chips_) * kBitsPerChannel;

  size_t i = 0;
  // 17 blanking pulses, previous row still selected (same order as the
  // bit-banged driver, which selected the new row after them).
  for (; i < blankWords_; ++i) dst[i] = prev | gclkAt(i);
  // New row address, GCLK stopped while the row drivers settle.
  for (; i < pwmStart; ++i) dst[i] = cur;
  // PWM pulses of this row, then GCLK stays low until the next row.
  for (; i < rowWords_; ++i) {
    const size_t k = i - pwmStart;
    dst[i] = cur | (k < pwmWords_ ? gclkAt(k) : 0);
  }
  // Data latch after every chips * 16 bits (pixel data itself is 0 = black).
  for (size_t k = latchWords - 1; k < dataWords_; k += latchWords) dst[dataOffset_ + k] |= kLat;
}

}  // namespace sum2033
