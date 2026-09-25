#include <Arduino.h>
#include <Sum2033Display.h>

// Wiring is the same as in the bit-banged version (on my matrix B and R
// channels were swapped places). These are also the library defaults.
static Sum2033Config panelConfig()
{
  Sum2033Config cfg;
  cfg.pins.b1 = 13;
  cfg.pins.b2 = 11;
  cfg.pins.g1 = 45;
  cfg.pins.g2 = 41;
  cfg.pins.r1 = 12;
  cfg.pins.r2 = 10;

  cfg.pins.a = 9;
  cfg.pins.b = 38;
  cfg.pins.c = 8;
  cfg.pins.d = 18;
  cfg.pins.e = 39;

  cfg.pins.clk = 7;
  cfg.pins.lat = 17;
  cfg.pins.oe  = 14; /* also GCLK */

  cfg.brightness = 63;      // 0-63  (6 bits)
  cfg.colorBits = 13;       // 10-13 (2 bits)
  cfg.gclkMultiplier = 16;  // 8 or 16

  // Every panel refresh also uploads a complete frame, so this sets both the
  // refresh rate and the max frame rate: 10 MHz ~290 Hz, 16 MHz ~465 Hz,
  // 20 MHz ~580 Hz. Go lower if you see random speckles.
  cfg.clockHz = 10000000;
  return cfg;
}

Sum2033Display display(64, 64);

// Same colorful RG gradient as the original test code, 16 bit per channel,
// rendered straight into the DMA buffer without a framebuffer.
static void showTestGradient()
{
  auto gamma16 = [](float v) { return uint16_t(powf(v / 65535.0f, 2.4f) * 65535.0f); };
  display.panel().render([&](int x, int y) -> sum2033::Rgb16 {
    return {gamma16(x * 1040), gamma16(y * 1040), 0};
  });
}

static uint8_t sinTable[256];

static void drawPlasma(uint32_t t)
{
  for (int y = 0; y < display.height(); y++) {
    for (int x = 0; x < display.width(); x++) {
      uint8_t v = sinTable[uint8_t(x * 4 + t)] / 4 + sinTable[uint8_t(y * 5 - t * 2)] / 4 +
                  sinTable[uint8_t((x + y) * 3 + t)] / 4 + sinTable[uint8_t(x * y / 8 - t)] / 4;
      display.setPixel(x, y, sinTable[uint8_t(v + t)], sinTable[uint8_t(v + 85)], sinTable[uint8_t(v + 170 - t)]);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < 256; i++) sinTable[i] = uint8_t(127.5f + 127.5f * sinf(i * 2 * PI / 256));

  if (!display.begin(panelConfig())) {
    while (true) {
      Serial.println("SUM2033 init failed (bad config or not enough internal RAM)");
      delay(1000);
    }
  }
  Sum2033& panel = display.panel();
  Serial.printf("SUM2033: CLK %u Hz, refresh %.1f Hz, DMA memory %u bytes, free heap %u bytes\n",
                unsigned(panel.clockHz()), panel.refreshRate(), unsigned(panel.memoryUsage()),
                unsigned(ESP.getFreeHeap()));

  showTestGradient();
  delay(3000);
}

void loop()
{
  static uint32_t frames = 0, lastReport = millis(), lastVsyncs = 0;
  static uint32_t drawUs = 0, flipUs = 0;
  static float fps = 0;

  const uint32_t t0 = micros();
  drawPlasma(millis() / 16);
  display.setCursor(1, 1);
  display.setTextColor(Sum2033Display::color565(255, 255, 255));
  display.print(int(fps + 0.5f));
  const uint32_t t1 = micros();
  display.flip();
  const uint32_t t2 = micros();

  drawUs += t1 - t0;
  flipUs += t2 - t1;
  frames++;

  const uint32_t now = millis();
  if (now - lastReport >= 1000) {
    Sum2033& panel = display.panel();
    const uint32_t vsyncs = panel.vsyncCount();
    const float seconds = (now - lastReport) / 1000.0f;
    fps = frames / seconds;
    Serial.printf("%.1f fps (draw %.2f ms, flip %.2f ms), panel refresh %.1f Hz\n", fps,
                  drawUs / 1000.0f / frames, flipUs / 1000.0f / frames, (vsyncs - lastVsyncs) / seconds);
    frames = drawUs = flipUs = 0;
    lastVsyncs = vsyncs;
    lastReport = now;
  }
}
