## Background

One day I ordered random very cheap (7$) 64x64 P3 RGB LED panel. When I recieved it, I tried it with many popular libs for ESP32, with no success. Then I found out, that as column drivers it has SUM2033GAS2, with about zero info about it on internet. By using techinque of educated guessing (also called "fucking around and finding out") I have wrote simple ESP32 code to test out my theories, eventually having fully featured matrix driver (except for speed, since I use GPIO bitbanging for simplicity). This driver can be used as a starting point to test panels, or to write actually fast drivers with DMA SPI or I2S. Which I might do later.  
This driver is most likely can be used to drive similar SUM20XX panels, i've seen that SUM2030 and SUM2032 exists, but of course I haven't tested this.

## Optimized DMA driver

`lib/SUM2033` is a fast driver for the same panel and the same wiring. The panel is refreshed entirely by hardware: the ESP32-S3 LCD peripheral (LCD_CAM, i8080 mode, 16 bit bus) streams a prepared sequence of pin states from RAM with GDMA, forever, without any CPU involvement.

- LCD `PCLK` drives the panel `CLK` pin; every other panel pin is one bit of the 16 bit words (routed through the GPIO matrix, so **the wiring is unchanged**). `OE`/GCLK is toggled as a data bit, one word high, one word low.
- One DMA loop = one panel refresh: config registers, VSYNC, then for every row the 17 blanking GCLK pulses, the row switch and the PWM GCLK pulses, exactly like the bit-banged code.
- The data of the next frame (16 latches per row) is shifted in **while** the GCLK pulses run, so a complete frame is uploaded in every refresh for free. The panel's second frame buffer holds it until the next VSYNC.
- There are two DMA buffers. You draw/encode into the one that isn't playing; it is linked in exactly at the start of the next refresh, so there is no tearing.

| Panel pin | GPIO (unchanged) | driven by |
|---|---|---|
| R1 G1 B1 | 12 45 13 | LCD_DATA_OUT0..2 |
| R2 G2 B2 | 10 41 11 | LCD_DATA_OUT3..5 |
| LAT | 17 | LCD_DATA_OUT6 |
| OE (GCLK) | 14 | LCD_DATA_OUT7 |
| A B C D E | 9 38 8 18 39 | LCD_DATA_OUT8..12 |
| CLK | 7 | LCD_PCLK |

| CLK (`clockHz`) | GCLK | refresh = max. frame rate (13 bit, x16) |
|---|---|---|
| 10 MHz (default) | 5 MHz | 291 Hz |
| 16 MHz | 8 MHz | 466 Hz |
| 20 MHz | 10 MHz | 582 Hz |

It needs about 135 KB of internal RAM for the two DMA buffers of a 64x64 panel (plus 12 KB for the `Sum2033Display` canvas). Works with Arduino-ESP32 2.0.x (IDF 4.4) and 3.x (IDF 5.x).

### Usage

With the Adafruit_GFX canvas (see `src/main.cpp` for a complete demo):

```cpp
#include <Sum2033Display.h>

Sum2033Display display(64, 64);

void setup() {
  Sum2033Config cfg;         // defaults: the wiring above, 13 bit, x16, brightness 63
  cfg.clockHz = 16000000;    // optional, faster refresh
  display.begin(cfg);
}

void loop() {
  display.fillScreen(0);
  display.setCursor(0, 0);
  display.print(millis());
  display.setPixel(10, 10, 255, 128, 0);  // 24 bit color
  display.flip();                          // queued for the next refresh
}
```

Or without a framebuffer, straight into the DMA buffer, with 16 bit (linear) precision per channel:

```cpp
Sum2033 panel;
panel.begin();
panel.render([](int x, int y) -> sum2033::Rgb16 {
  return {uint16_t(x * 1040), uint16_t(y * 1040), 0};
});
```

`show(rgb888)`, `show(rgb565)` and `showLinear(rgb16)` take your own buffers. `setBrightness(0..63)` and `setGamma()` apply to the next frame, `refresh()` re-sends the current one. `waitVSync()`, `vsyncCount()` and `refreshRate()` help with timing. All frame functions encode into the free DMA buffer and only block while the previously queued frame hasn't started yet, so the frame rate is capped at the refresh rate.

### If something looks wrong

- Random speckles or wrong colors: lower `clockHz` (8 or 5 MHz), or try `driveStrength = 3`.
- Ghosting between rows: increase `rowSwitchGap` (CLK periods with GCLK stopped after the row address changes, default 8).
- Garbage instead of the image: set `shiftDuringPwm = false`. Data is then shifted after the PWM pulses of each row with GCLK stopped, exactly in the order the bit-banged code used (slower refresh).

### Tests

`test/host/protocol_test.cpp` runs on a PC. It feeds a behavioral model of the SUM2033 chain (shift registers, LAT commands, config registers, double buffered SRAM, GCLK/row tracking) with the original bit-banged code and with the DMA stream, and checks that they produce the same registers, the same GCLK/row sequence and the same image, for all color depths and multipliers:

```
g++ -std=c++11 -O2 -Ilib/SUM2033/src test/host/protocol_test.cpp lib/SUM2033/src/Sum2033Protocol.cpp -o protocol_test && ./protocol_test
```

## So what I know now

- it supports 10-16 bit color per channel (altho I only successfully got 13 bits)
- it does that by counting clocks on OE (GCLK) pin, and it has internal 8-16x configurable clocks frequency multiplicator, so it has to recieve between 2^10/16=64 and 2^13/8=1024 equally spaced PWM pulses per row of display
- it also has to recieve 17 addditional GCLK pulses independent of color depth chosen
- it has 16K bits of internal RAM (2 entire frame buffers), and you can send new frame to RAM, while displaying previous one (double buffering)
- I think sending new data in can be independent of GCLK, but for simplicity i've made it syncronized (10 bit color perfectly matches 16x4x16=1024 pulses needed to fill next line)
- when you want to show next frame, you should send special command, N=2 (or 3) CLK pulses with high latch. It swaps frame buffers and resets GCLK, CLK and LATCH counter.
- it may have other commands, activated in same way. Altho I haven't found any particulary useful ones.

## Internal config registers

- when I say "bit number N" below -- I count from zero
- it has 3 internal config registers, written with 9, 11 and 13 CLK pulses after pulling LATCH high:

### REG1

- Consists of wierd bit number 2, most likely used to achieve more than 13 bits of color depth, but this requires wierd row activation sequence or some other configuration, and 13 bits is plenty for me.  
- Also consists of wierd bit number 3, which, when enabled, can do wierd stuff combined with other otherwise seemingly unused bits.  
- All other unused bits here and in other registers are most likely those "wierd" bits that only change anything when this bit 3 is enabled.

### REG2

- 6 bits of global brightness (64 levels, but 0 brightness is more like half of max brightness, still useful)
- 5 bits of scan select (1-32, my panel is 1/32 scan (64x64), so should be all ones)
- 2 bits of color depth select, inverted (00 means 13 bits, 01 means 12 bits, 10 means 11 bits, 11 means 10 bits)
- 2 unknown purpose bits (somehow cuts dark colors?)
- 1 bit that inverts PWM sequence (useless)

### REG3

- Bit number 2 is used to enable X16 GCLK multiplication (X8 otherwise).  
- Bit number 4, when 0, seems to almost completely disable matrix? this "almost" is wierd tho, and this "almost" can be achieved with bit 3 of REG1 set to 1, so idk.  
- Otherwise also consists of those "wierd" bits.