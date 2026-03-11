One day I ordered random very cheap (7$) 64x64 P3 RGB LED panel. When I recieved it, I tried it with many popular libs for ESP32, with no success. Then I found out, that as column drivers it has SUM2033GAS2, with about zero info about it on internet. By using techinque of educated guessing (also called "fuckind around and finding out") I have wrote simple ESP32 code to test out my theories, eventually having fully featured matrix driver (except for speed, since I use GPIO bitbanging for simplicity). This driver can be used as a starting point to test panels, or to write actually fast drivers with DMA SPI or I2S. Which I might do later.
This driver is most likely can be used to drive similar SUM20XX panels, i've seen that SUM2030 and SUM2032 exists, but of course I haven't tested this.

So what I know now:
- it supports 10-16 bit color
- it does that by counting clocks on OE pin (connected to GCLK), so it has to recieve between 2^10=1023 and 2^16=65535 equally spaced pulses per row of display
- it automagically works with any color depth (from 10 to 16 ig), you just should send more GCLK pulses per row and get more color depth (datasheet says it can recieve up to 30 mHz on GCLK)
- it has some amount of internal RAM, and you should send next line to his ram, while displaying previous one. 
- I think sending new data in can be independent of GCLK, but for simplicity i've made it syncronized (10 bit color perfectly matches 16x4x16=1024 pulses needed to fill next line)
- when you want to show next line, you should send special command, N=2 (or 3) CLK pulses with high latch. It swaps buffers and resets GCLK counter.
- it may have other commands. Maybe even up to 32 of them. Altho I haven't found any particulary useful ones.