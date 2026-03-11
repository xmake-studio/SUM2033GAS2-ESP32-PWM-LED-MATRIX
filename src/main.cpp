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
#define OE 14


#define MATRIX_WIDTH 64 /* can be increased in case of chain link, altho I have only one matrix */
#define MATRIX_SCAN  32 /* 1/32 scan in my case, also means 64 pixels height */


constexpr uint32_t CLK_MASK = 1UL << CLK;
constexpr uint32_t OE_MASK  = 1UL << OE;
constexpr uint32_t BOTH_MASK = CLK_MASK | OE_MASK;

// fast GPIO magik used here, if you want CLK or OE pins to be larger than 31 - change registers or change this magik to gpio_set_level
#define CLK_PULSE GPIO.out_w1tc = CLK_MASK; GPIO.out_w1ts = CLK_MASK;
#define OE_PULSE  GPIO.out_w1tc = OE_MASK;  GPIO.out_w1ts = OE_MASK;

// pulsing both CLK and OE in single call
#define BOTH_PULSE  GPIO.out_w1tc = BOTH_MASK;  GPIO.out_w1ts = BOTH_MASK;

void selectRow(int row)
{
  gpio_set_level((gpio_num_t)CH_A, (row >> 0) & 1);
  gpio_set_level((gpio_num_t)CH_B, (row >> 1) & 1);
  gpio_set_level((gpio_num_t)CH_C, (row >> 2) & 1);
  gpio_set_level((gpio_num_t)CH_D, (row >> 3) & 1);
  gpio_set_level((gpio_num_t)CH_E, (row >> 4) & 1);
}

void latch(int clk_count)
{
  gpio_set_level((gpio_num_t)LAT, HIGH);

  for(int i = 0; i < clk_count; i++)
  {
    CLK_PULSE
  }

  gpio_set_level((gpio_num_t)LAT, LOW);
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
  
  pix.r = pix.g = pix.b = ((x == y) * (x * 1023)); // diagonal white line with gradient
  //pix.r = pix.g = pix.b = (x * 1023);            // just white gradient
  
  return gammaCorrectPixel(pix);
}

void loop() 
{
  const int colorBits = 16;
  const int channelsPerChip = 16;
  const int chips = MATRIX_WIDTH / channelsPerChip;
  const int rowsPerChip = MATRIX_SCAN;

  uint64_t test = millis() / 200;

  for(uint8_t row = 0; row < rowsPerChip; row++) 
  {
    for(uint8_t channel = 0; channel < channelsPerChip; channel++) 
    {
      for(uint8_t chip = 0; chip < chips; chip++)
      {
        const auto x = channel + channelsPerChip * chip;
        const auto p1 = getPixel(x, row);
        const auto p2 = getPixel(x, row + rowsPerChip);

        // test pattern for blue horizontal lines
        gpio_set_level((gpio_num_t)B1, row == (test % rowsPerChip));

        for(int8_t bit = colorBits - 1; bit >= 0; bit--) // inverted bit order
        {
          const uint16_t m = 1UL << bit;
          
          // full color set - kinda slow this way. Comment out 6 lines below and uncomment needed test pattern lines to see test patterns
          gpio_set_level((gpio_num_t)R1, p1.r & m);
          gpio_set_level((gpio_num_t)G1, p1.g & m);
          gpio_set_level((gpio_num_t)B1, p1.b & m);
          gpio_set_level((gpio_num_t)R2, p2.r & m);
          gpio_set_level((gpio_num_t)G2, p2.g & m);
          gpio_set_level((gpio_num_t)B2, p2.b & m);

          // alternative - test patterns:
          // (it is normal that in test pattern mode only ony half of matrix works, it is 2X speed this way)
          //gpio_set_level((gpio_num_t)G1, bit == (test % colorBits));                                      // test pattern to check individual color bits
          //gpio_set_level((gpio_num_t)R1, (channel + channelsPerChip * chip) == ((test % MATRIX_WIDTH)));  // test pattern for checking individual channels (vertical lines)

          //BOTH_PULSE // pulsing both CLK and OE in single call
          CLK_PULSE
          OE_PULSE
          for(int i = 0; i < 64; i++)
          {
          }
        }
      }
      latch(0);
    }

    latch(2); // 2, 3 works?
    OE_PULSE // to prevent ghosting
    selectRow(row);
  }
}


