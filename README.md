One day I ordered random very cheap (7$) 64x64 P3 RGB LED panel. When I recieved it, I tried it with many popular libs for ESP32, with no success. Then I found out, that as column drivers it has SUM2033GAS2, with about zero info about it on internet. By using techinque of educated guessing (also called "fuckind around and finding out") I have wrote simple ESP32 code to test out my theories, eventually having fully featured matrix driver (except for speed, since I use GPIO bitbanging for simplicity). This driver can be used as a starting point to test panels, or to write actually fast drivers with DMA SPI or I2S. Which I might do later.
This driver is most likely can be used to drive similar SUM20XX panels, i've seen that SUM2030 and SUM2032 exists, but of course I haven't tested this.

So what I know now:
- it supports 10-16 bit color per channel (altho I only successfully got 13 bits)
- it does that by counting clocks on OE (GCLK) pin, and it has internal 8-16x configurable clocks frequency multiplicator, so it has to recieve between 2^10/16=64 and 2^13/8=1024 equally spaced PWM pulses per row of display
- it also has to recieve 17 addditional GCLK pulses independent of color depth chosen
- it has 16K bits of internal RAM (2 entire frame buffers), and you can send new frame to RAM, while displaying previous one (double buffering)
- I think sending new data in can be independent of GCLK, but for simplicity i've made it syncronized (10 bit color perfectly matches 16x4x16=1024 pulses needed to fill next line)
- when you want to show next frame, you should send special command, N=2 (or 3) CLK pulses with high latch. It swaps frame buffers and resets GCLK, CLK and LATCH counter.
- it may have other commands, activated in same way. Altho I haven't found any particulary useful ones.

- it has 3 internal config registers, written with 9, 11 and 13 CLK pulses after pulling LATCH high:

- REG1: 
Consists of wierd bit number 2, most likely used to achieve more than 13 bits of color depth, but this requires wierd row activation sequence or some other configuration, and 13 bits is plenty for me.
Also consists of wierd bit number 3, which, when enabled, can do wierd stuff combined with other otherwise seemingly unused bits
All other bits here and in other registers are most likely those "wierd" bits that only change anything when this bit 3 is enabled.

- REG2:
6 bits of global brightness (64 levels, but 0 brightness is more like half of max brightness, still useful)
5 bits of scan select (1-32, my panel is 1/32 scan (64x64), so should be all ones)
2 bits of color depth select, inverted (00 means 13 bits, 01 means 12 bits, 10 means 11 bits, 11 means 10 bits)
2 unknown purpose bits (somehow cuts dark colors?)
1 bit that inverts PWM sequence (useless)

- REG3:
Bit number 2 is used to enable X16 GCLK multiplication (X8 otherwise)
Bit number 4, when 0, seems to almost completely disable matrix? this "almost" is wierd tho, and this "almost" can be achieved with bit 3 of REG1 set to 1, so idk
Otherwise also consists of those "wierd" bits