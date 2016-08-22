/*
PICCOLO is a tiny Arduino-based audio visualizer.
Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Bicolor LED strip with I2C Backpack (ID: 902)
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Optional: battery for portable use (else power through USB)
Software requirements:
 - elm-chan's ffft library for Arduino
Connections:
 - 3.3V to mic amp+ and Arduino AREF pin <-- important!
 - GND to mic amp-
 - Analog pin 0 to mic amp output
 - +5V, GND, SDA (or analog 4) and SCL (analog 5) to I2C strip backpack
Written by Adafruit Industries.  Distributed under the BSD license --
see license.txt for more information.  This paragraph must be included
in any redistribution.
ffft library is provided under its own terms -- see ffft.S for specifics.
*/

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.

#include <avr/pgmspace.h>
#include "ffft.h"
#include <math.h>
#include <Wire.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif



//analogReference(DEFAULT);
#include <Adafruit_NeoPixel.h>


// Microphone connects to Analog Pin 0.  Corresponding ADC channel number
// varies among boards...it's ADC0 on Uno and Mega, ADC7 on Leonardo.
// Other boards may require different settings; refer to datasheet.
#ifdef __AVR_ATmega32U4__
 #define ADC_CHANNEL 0x25  /* A10 on Flora */
#else
 #define ADC_CHANNEL 0    /* A0 on Uno */
#endif

#define N_PIXELS  16  // Number of pixels you are using
#define MIC_PIN    A9  // Microphone is attached to Trinket GPIO #2/Gemma D2 (A1)
#define LED_PIN    6  // NeoPixel LED strand is connected to GPIO #0 / D0

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N / 2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

byte
peak[8],      // Peak level of each column; used for falling dots
     dotCount = 0, // Frame counter for delaying dot-falling speed
     colCount = 0; // Frame counter for storing past column data
int
col[8][10],   // Column levels for the prior 10 frames
    minLvlAvg[8], // For dynamic adjustment of low & high ends of graph,
    maxLvlAvg[8], // pseudo rolling averages for the prior few frames.
    colDiv[8];    // Used when filtering FFT output to 8 columns

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/
static const uint8_t PROGMEM
// This is low-level noise that's subtracted from each FFT output column:
noise[64] = { 8, 6, 6, 5, 3, 4, 4, 4, 3, 4, 4, 3, 2, 3, 3, 4,
              2, 1, 2, 1, 3, 2, 3, 2, 1, 2, 3, 1, 2, 3, 4, 4,
              3, 2, 2, 2, 2, 2, 2, 1, 3, 2, 2, 2, 2, 2, 2, 2,
              2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4
            },
            // These are scaling quotients for each FFT output column, sort of a
            // graphic EQ in reverse.  Most music is pretty heavy at the bass end.
eq[64] = {
  255, 175, 218, 225, 220, 198, 147, 99, 68, 47, 33, 22, 14,  8,  4,  2,
  0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
},
// When filtering down to 8 columns, these tables contain indexes
// and weightings of the FFT spectrum output values to use.  Not all
// buckets are used -- the bottom-most and several at the top are
// either noisy or out of range or generally not good for a graph.
col0data[] = {  2,  1,  // # of spectrum bins to merge, index of first
                111,   8
             },           // Weights for each bin
             col1data[] = {  4,  1,  // 4 bins, starting at index 1
                             19, 186,  38,   2
                          }, // Weights for 4 bins.  Got it now?
                          col2data[] = {  5,  2,
                                          11, 156, 118,  16,   1
                                       },
                                       col3data[] = {  8,  3,
                                                       5,  55, 165, 164,  71,  18,   4,   1
                                                    },
                                           col4data[] = { 11,  5,
                                                          3,  24,  89, 169, 178, 118,  54,  20,   6,   2,   1
                                                        },
                                               col5data[] = { 17,  7,
                                                              2,   9,  29,  70, 125, 172, 185, 162, 118, 74,
                                                              41,  21,  10,   5,   2,   1,   1
                                                            },
                                                   col6data[] = { 25, 11,
                                                                  1,   4,  11,  25,  49,  83, 121, 156, 180, 185,
                                                                  174, 149, 118,  87,  60,  40,  25,  16,  10,   6,
                                                                  4,   2,   1,   1,   1
                                                                },
                                                       col7data[] = { 37, 16,
                                                                      1,   2,   5,  10,  18,  30,  46,  67,  92, 118,
                                                                      143, 164, 179, 185, 184, 174, 158, 139, 118,  97,
                                                                      77,  60,  45,  34,  25,  18,  13,   9,   7,   5,
                                                                      3,   2,   2,   1,   1,   1,   1
                                                                    },
                                                           // And then this points to the start of the data for each of the columns:
* const colData[]  = {
  col0data, col1data, col2data, col3data,
  col4data, col5data, col6data, col7data
};

//Adafruit_Neostrip strip = Adafruit_Neostrip(8, 8, PIN,
//                            NEO_strip_TOP     + NEO_strip_RIGHT +
//                            NEO_strip_COLUMNS + NEO_strip_PROGRESSIVE,
//                            NEO_GRB            + NEO_KHZ800);

// using Adafruit_NeoPixel strip

const uint16_t LED_RED = strip.Color(128, 0, 0);
const uint16_t LED_YELLOW = strip.Color(128, 128, 0);
const uint16_t LED_GREEN = strip.Color(0, 128, 0);
const uint16_t LED_OFF = strip.Color(0, 0, 0);

const uint16_t colors[] = {
  strip.Color(255, 0, 0), strip.Color(255, 255, 0), strip.Color(0, 255, 0)
};

void setup() {

  strip.begin();
  strip.setBrightness(40);

  uint8_t i, j, nBins, binNum, *data;

  memset(peak, 0, sizeof(peak));
  memset(col , 0, sizeof(col));

  for (i = 0; i < 8; i++) {
    minLvlAvg[i] = 0; //min level...frequency?
    maxLvlAvg[i] = 512;
    data         = (uint8_t *)pgm_read_word(&colData[i]);
    nBins        = pgm_read_byte(&data[0]) + 2;
    binNum       = pgm_read_byte(&data[1]);
    for (colDiv[i] = 0, j = 2; j < nBins; j++)
      colDiv[i] += pgm_read_byte(&data[j]);
  }

    // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion 
#ifdef __AVR_ATmega32U4__
  ADMUX  = ADC_CHANNEL | _BV(REFS0); // FLORA Channel sel, right-adj, use internal AREF
#else
  ADMUX  = ADC_CHANNEL; // Arduino Uno: Channel sel, right-adj, use AREF pin
#endif


#if (ADC_CHANNEL > 7)
  ADMUX  = _BV(REFS0) | (ADC_CHANNEL - 8);
  ADCSRB = _BV(MUX5);        // Free run mode, high MUX bit
  DIDR2  = 1 << (ADC_CHANNEL - 8);
#else
  ADMUX  = _BV(REFS0) | ADC_CHANNEL;
  ADCSRB = 0;                // Free run mode, no high MUX bit
  DIDR0  = 1 << ADC_CHANNEL;
#endif
  ADCSRA = _BV(ADEN)  | // ADC enable
           _BV(ADSC)  | // ADC start
           _BV(ADATE) | // Auto trigger
           _BV(ADIE)  | // Interrupt enable
           _BV(ADPS1) | _BV(ADPS0); // 64:1 / 13 = 9615 Hz

          
  TIMSK0 = 0;                // Timer0 off
  sei(); // Enable interrupts


  




  Serial.begin(9600);
}

void loop() {
  uint8_t  i, x, L, *data, nBins, binNum, weighting, c;
  uint16_t minLvl, maxLvl;
  int      level, y, sum;

  while (ADCSRA & _BV(ADIE)); // Wait for audio sampling to finish

  fft_input(capture, bfly_buff);   // Samples -> complex #s
  samplePos = 0;                   // Reset sample counter
  ADCSRA |= _BV(ADIE);             // Resume sampling interrupt
  fft_execute(bfly_buff);          // Process complex data
  fft_output(bfly_buff, spectrum); // Complex -> spectrum

  // Remove noise and apply EQ levels
  for (x = 0; x < FFT_N / 2; x++) {
    L = pgm_read_byte(&noise[x]);
    spectrum[x] = (spectrum[x] <= L) ? 0 :
                  (((spectrum[x] - L) * (256L - pgm_read_byte(&eq[x]))) >> 8);
  }

  // Fill background w/colors, then idle parts of columns will erase
//  strip.fillRect(0, 0, 8, 3, LED_RED);    // Upper section
//  strip.fillRect(0, 3, 8, 2, LED_YELLOW); // Mid
//  strip.fillRect(0, 5, 8, 3, LED_GREEN);  // Lower section

//possible functions to use to illustrate
//  colorWipe(strip.Color(255, 0, 0), 50); // Red
//  colorWipe(strip.Color(0, 255, 0), 50); // Green
//  colorWipe(strip.Color(0, 0, 255), 50); // Blue
//  // Send a theater pixel chase in...
//  theaterChase(strip.Color(127, 127, 127), 50); // White
//  theaterChase(strip.Color(127, 0, 0), 50); // Red
//  theaterChase(strip.Color(0, 0, 127), 50); // Blue
//
//  rainbow(20);
//  rainbowCycle(20);
//  theaterChaseRainbow(50);

  

  // Downsample spectrum output to 8 columns:
  for (x = 0; x < 8; x++) {
    data   = (uint8_t *)pgm_read_word(&colData[x]);
    nBins  = pgm_read_byte(&data[0]) + 2;
    binNum = pgm_read_byte(&data[1]);
    for (sum = 0, i = 2; i < nBins; i++)
      sum += spectrum[binNum++] * pgm_read_byte(&data[i]); // Weighted
    col[x][colCount] = sum / colDiv[x];                    // Average
    minLvl = maxLvl = col[x][0];
    for (i = 1; i < 10; i++) { // Get range of prior 10 frames
      if (col[x][i] < minLvl)      minLvl = col[x][i];
      else if (col[x][i] > maxLvl) maxLvl = col[x][i];
    }
    // minLvl and maxLvl indicate the extents of the FFT output, used
    // for vertically scaling the output graph (so it looks interesting
    // regardless of volume level).  If they're too close together though
    // (e.g. at very low volume levels) the graph becomes super coarse
    // and 'jumpy'...so keep some minimum distance between them (this
    // also lets the graph go to zero when no sound is playing):
    if ((maxLvl - minLvl) < 8) maxLvl = minLvl + 8;
    minLvlAvg[x] = (minLvlAvg[x] * 7 + minLvl) >> 3; // Dampen min/max levels
    maxLvlAvg[x] = (maxLvlAvg[x] * 7 + maxLvl) >> 3; // (fake rolling average)

    // Second fixed-point scale based on dynamic min/max levels:
    level = 10L * (col[x][colCount] - minLvlAvg[x]) /
            (long)(maxLvlAvg[x] - minLvlAvg[x]);

    // Clip output and convert to byte:
    if (level < 0L)      c = 0;
    else if (level > 10) c = 10; // Allow dot to go a couple pixels off top
    else                c = (uint8_t)level;

    if (c > peak[x]) peak[x] = c; // Keep dot on top

    if (peak[x] <= 0) { // Empty column?
      //strip.drawLine(x, 0, x, 7, LED_OFF);
      //colorWipe(strip.Color(0, 0, 0), 255); // Green


          for (int i = 0; i < NUMPIXELS; i++) {
      if ( i == 0 || i == 1 || i == 2 || i == 3) {

        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
        stripLeft.setPixelColor(i, stripLeft.Color(255, 255, 0)); // yellow



        stripLeft.show();
      continue;
    } else if (c < 8) { // Partial column?
      //strip.drawLine(x, 0, x, 7 - c, LED_OFF);
      colorWipe(strip.Color(255, 0, 0), 50); // Red
    }

    // The 'peak' dot color varies, but doesn't necessarily match
    // the three screen regions...yellow has a little extra influence.
    y = 8 - peak[x];
    if (y < 2)      colorWipe(strip.Color(255, 0, 0), 50); // Red
    else if (y < 6) colorWipe(strip.Color(0, 255, 0), 50); // Green
    else           colorWipe(strip.Color(0, 0, 255), 50); // Blue
  }

  strip.show();

  // Every third frame, make the peak pixels drop by 1:
  if (++dotCount >= 3) {
    dotCount = 0;
    for (x = 0; x < 8; x++) {
      if (peak[x] > 0) peak[x]--;
    }
  }

  if (++colCount >= 10) colCount = 0;
}

ISR(ADC_vect) { // Audio-sampling interrupt
  static const int16_t noiseThreshold = 4;
  int16_t              sample         = ADC; // 0-1023

  capture[samplePos] =
    ((sample > (512 - noiseThreshold)) &&
     (sample < (512 + noiseThreshold))) ? 0 :
    sample - 512; // Sign-convert for FFT; -512 to +511

  if (++samplePos >= FFT_N) ADCSRA &= ~_BV(ADIE); // Buffer full, interrupt off
}




/////NeoPixel Ring animations >> from strandtest
// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
