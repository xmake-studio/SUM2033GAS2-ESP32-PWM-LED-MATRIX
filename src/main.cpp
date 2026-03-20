#include <Arduino.h>
#include "soc/gpio_struct.h"

// on my matrix B and R channels were swapped places
#define B1 13
#define B2 11
#define G1 45
#define G2 41
#define R1 12
#define R2 10

#define CH_A 9
#define CH_B 38
#define CH_C 8
#define CH_D 18
#define CH_E 39

#define CLK 7
#define LAT 17
#define OE  14 /* also GLCK */

#define MATRIX_WIDTH 64 /* can be increased in case of chain link, altho I have only one matrix */
#define MATRIX_SCAN  32 /* 1/32 scan in my case, also means 64 pixels height */

uint8_t cfg_brightness = 63; // 0-63  (6 bits)
uint8_t cfg_color_bits = 13; // 10-13 (2 bits)

// in-chip multiplier of GCLK pulses count, as far as I know can be configured to either 8 or 16, 
// altho I haven't seen any benefit in decreasing it to 8.
uint8_t cfg_gclk_mul = 16;  

const int storedColorBits = 16; // should always be 16, independent on cfg_color_bits, since we should fill entire RAM of SUM2033
const int channelsPerChip = 16; // always 16 for SUM2033
const int chips_count = MATRIX_WIDTH / channelsPerChip;

constexpr uint32_t CLK_MASK = 1UL << CLK;
constexpr uint32_t GCLK_MASK  = 1UL << OE;

// fast GPIO magik used here, if you want CLK or OE pins to be larger than 31 - change registers or change this magik to gpio_set_level
#define CLK_PULSE   GPIO.out_w1ts = CLK_MASK;   GPIO.out_w1tc = CLK_MASK;
#define GCLK_PULSE  GPIO.out_w1ts = GCLK_MASK;  GPIO.out_w1tc = GCLK_MASK;

void selectRow(int row)
{
  gpio_set_level((gpio_num_t)CH_A, (row >> 0) & 1);
  gpio_set_level((gpio_num_t)CH_B, (row >> 1) & 1);
  gpio_set_level((gpio_num_t)CH_C, (row >> 2) & 1);
  gpio_set_level((gpio_num_t)CH_D, (row >> 3) & 1);
  gpio_set_level((gpio_num_t)CH_E, (row >> 4) & 1);
}

void IRAM_ATTR sendPwmClock(int clocks) {
  while (clocks--) {
    GCLK_PULSE
  }
}

void IRAM_ATTR sendLatch(unsigned char clocks)
{
  gpio_set_level((gpio_num_t)LAT, HIGH);

  while(clocks--) {
    CLK_PULSE
  }

  gpio_set_level((gpio_num_t)LAT, LOW);
}

void sendConfiguration(unsigned char latches, unsigned int data) {
    unsigned char num = chips_count;

    latches = 16 - latches;

    gpio_set_level((gpio_num_t)LAT, 0); 

    while(num--) {
        for(unsigned char x = 0; x < 16; x++) {
            unsigned int dataMask = 0x8000 >> x;

            bool en = data & dataMask;

            gpio_set_level((gpio_num_t)R1, en);
            gpio_set_level((gpio_num_t)G1, en);
            gpio_set_level((gpio_num_t)B1, en);
            gpio_set_level((gpio_num_t)R2, en);
            gpio_set_level((gpio_num_t)G2, en);
            gpio_set_level((gpio_num_t)B2, en);

            if(num == 0 && x == latches) 
            { 
              gpio_set_level((gpio_num_t)LAT, 1); 
            }
            CLK_PULSE
        }

        gpio_set_level((gpio_num_t)LAT, 0); 
    }
}

void sendConfigRegs()
{
  sendLatch(14); // Pre-active command
  sendConfiguration(9, 0b0); // Write config register 1

  // 6 bits of brightness
  uint16_t reg2 = cfg_brightness & 0b111111; 

  // 5 bits after brightness - scan lines register, we have 1/32 scan, so all ones (literally 1984) 
  reg2 |= (MATRIX_SCAN - 1) << 6; 
  
  // another 2 bits is inverted color depth - from 10 to 13 in normal (slow?) mode
  reg2 |= (~(cfg_color_bits - 10) & 0b11) << 11; 
  
  sendLatch(14);  // Pre-active command
  sendConfiguration(11, reg2); // Write config register 2

  uint16_t reg3 = 0;

  reg3 |= (cfg_gclk_mul >> 4) << 2; // shifting by 4, since 16 = 0b10000 >> 4 = 0b1
  reg3 |= 1 << 4; // must be 1 for matrix to function correctly

  sendLatch(14); // Pre-active command
  sendConfiguration(13, reg3); // Write config register 3
}

void setup() 
{
  Serial.begin(115200);
  pinMode(R1  , OUTPUT);
  pinMode(G1  , OUTPUT);
  pinMode(B1  , OUTPUT);
  pinMode(R2  , OUTPUT);
  pinMode(G2  , OUTPUT);
  pinMode(B2  , OUTPUT);
  pinMode(CH_A, OUTPUT);  
  pinMode(CH_B, OUTPUT);
  pinMode(CH_C, OUTPUT);  
  pinMode(CH_D, OUTPUT);
  pinMode(CH_E, OUTPUT);
  pinMode(CLK , OUTPUT);  
  pinMode(LAT , OUTPUT);
  pinMode(OE  , OUTPUT);
}

struct pixel
{
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

uint16_t gammaCorrectGrayscale(uint16_t input, float gamma = 2.4)
{
  float col = (float)input / 65535.0f;
  col = powf(col, gamma);
  return (uint16_t)(col * 65535.0f);
}

pixel gammaCorrectPixel(pixel p, float gamma = 2.4) {
  p.r = gammaCorrectGrayscale(p.r, gamma);
  p.g = gammaCorrectGrayscale(p.g, gamma);
  p.b = gammaCorrectGrayscale(p.b, gamma);
  return p;
}

pixel getPixel(uint8_t x, uint8_t y)
{
  pixel pix;
  
  // 1040 here everywhere since it is 65535/63

  //pix.r = pix.g = pix.b = ((x == y) * gammaCorrectGrayscale(x * 1040)); // diagonal white line with gradient
  //pix.r = pix.g = pix.b = y == 1 ? 65535 : ((y == 5) * (x * 1040));     // test
  //pix.r = pix.g = pix.b = gammaCorrectGrayscale(x * 1040);              // just white gradient

  pix.r = gammaCorrectGrayscale(x * 1040);                  // colorful RG gradient
  pix.g = gammaCorrectGrayscale(y * 1040);                  // colorful RG gradient
  
  return pix;
}

void loop() 
{
  //cfg_brightness = (millis() / 100) % 64; // to test brightness
  //cfg_color_bits = 10 + (millis() / 1000) % 4; // to test color depth

  sendConfigRegs();

  static int c = -1;
  c++;
  // updating entire frame in first 2 loops (since double buffering)
  bool fullUpdate = c < 2;

  int gclk_pulses_per_line = pow(2, cfg_color_bits) / cfg_gclk_mul;

  sendLatch(3); // vsync + buffer swap

  for(uint8_t line = 0; line < MATRIX_SCAN; line++) 
  {
    sendPwmClock(17); // should be here, and 17 clocks exactly
    selectRow(line);
    sendPwmClock(gclk_pulses_per_line);

    if(!fullUpdate)
      continue;
    
    for(uint8_t channel = 0; channel < channelsPerChip; channel++) 
    {
      for(uint8_t chip = 0; chip < chips_count; chip++)
      {
        const auto x = channel + channelsPerChip * chip;
        const auto p1 = getPixel(x, line);
        const auto p2 = getPixel(x, line + MATRIX_SCAN);

        for(int8_t bit = storedColorBits - 1; bit >= 0; bit--) // inverted bit order
        {
          const uint16_t m = 1UL << bit;
        
          gpio_set_level((gpio_num_t)R1, p1.r & m);
          gpio_set_level((gpio_num_t)G1, p1.g & m);
          gpio_set_level((gpio_num_t)B1, p1.b & m);
          gpio_set_level((gpio_num_t)R2, p2.r & m);
          gpio_set_level((gpio_num_t)G2, p2.g & m);
          gpio_set_level((gpio_num_t)B2, p2.b & m);

          if(chip == chips_count - 1 && bit == 0)
          {
            // send a latch for 1 clock after every chip was written to 
            gpio_set_level((gpio_num_t)LAT, 1); 
          }

          CLK_PULSE
        }
      }
      gpio_set_level((gpio_num_t)LAT, 0);
    }
  }
}
