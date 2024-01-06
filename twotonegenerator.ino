//
// Simple Two Tone Generator
//
// github.com/VE3QVY/twotonegenerator
//
// Generates two audio tones using an ATtiny1614 which is passed
// through a 4Khz low pass Sallen-Key filter and attenuated to 
// microphone level.  Can be used for HAM/CB for a two tone test
// for linearity
//
// Project as designed uses a ATtiny1614 which requires the 
// megaTinyCore board definitions.  The software should be generic
// enough to run on any other suitable Arduino board (with the appropriate
// modifications) that supports a DAC output.
//
// Other libraries required
//  - SSD1306Ascii - for the OLED display
//  - ATtiny_TimerInterrupt

//
// User defines - adjust to suit your setup

// SSD1306 type LCD module 128x96 display
// Uses the SSD1306Ascii slim library since the microcontroller
// has only 16K of ROM.  Just need the bare minimum basics to display
// some text on the display
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

// These define's must be placed at the beginning before #include "ATtiny_TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

// Select USING_FULL_CLOCK      == true for  20/16MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_HALF_CLOCK      == true for  10/ 8MHz to Timer TCBx => shorter timer, but better accuracy
// Select USING_250KHZ          == true for 250KHz to Timer TCBx => longer timer,  but worse  accuracy
// Not select for default 250KHz to Timer TCBx => longer timer,  but worse accuracy
#define USING_FULL_CLOCK      true
#define USING_HALF_CLOCK      false
#define USING_250KHZ          false         // Not supported now

// Try to use RTC, TCA0 or TCD0 for millis()
#define USE_TIMER_0           true          // Check if used by millis(), Servo or tone()
#define USE_TIMER_1           false         // Check if used by millis(), Servo or tone()

#if USE_TIMER_0
  #define CurrentTimer   ITimer0
#elif USE_TIMER_1
  #define CurrentTimer   ITimer1
#else
  #error You must select one Timer  
#endif

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ATtiny_TimerInterrupt.h"

//
// Ideally we want the interrupt to have at 20Khz, however due to variances in the 
// microcontroller adjust TIMER2_FREQUENCY_HZ plus or minus 20000UL so the interrupt occurs
// as close as possible to 20Khz.  The output pin PA5 is toggled on the interrupt
// so it should produce a 10Khz square at this output for fine tuning purposes

#define TIMER2_FREQUENCY_HZ     20160UL

//
// The sine wave table
// A full cycle of 8 bit values, an entire wave is represented in 256 steps
// normalized_phase_increment represents the fractional increment to produce
// a 1Hz waveform at the interrupt rate of 20Khz.  The produce an wave of
// n Hz then multiply n by normalized_phase_increment to get the fractional
// increment in phase when the microcontroller is interrupted at 20Khz
uint8_t  sine_wave[256] = {
  0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
  0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
  0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
  0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
  0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
  0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
  0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
  0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
  0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
  0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
  0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
  0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
  0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
  0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
  0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
  0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
  0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
  0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
  0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
  0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
  0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
  0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
  0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
  0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
  0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
  0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
  0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
  0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
  0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
  0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};
float normalized_phase_increment = 256.0/20000.0;

// the working variables to index the sine_wave table to produce two
// independant sine waves
//  index = the current index into the sine_wave table
//  minor_inc = the fractional amount to increment index on the interrupt
//  acc_inc = an accumulator (overflow at 32768) of minor_inc.  when this
//            value overflows the integer amount to increment the index is +1
//  major_inc = the integer amount to increment index on the interrupt

uint8_t index1 = 0;
uint8_t index2 = 0;

uint8_t major_inc1 = 0;
uint16_t acc_inc1 = 0;
uint16_t minor_inc1 = 0;

uint8_t major_inc2 = 0;
uint16_t acc_inc2 = 0;
uint16_t minor_inc2 = 0;

void TimerHandler()
{
  // get the value of each sine wave, add them to together and div by 2
  // this effectively mixes the two independant sine waves
  // output the value on DAC0 (for ATtiny1614 this is pin PA6)
  uint16_t value = sine_wave[index1] + sine_wave[index2];
  analogWrite(PIN_PA6, value/2);

  // now increment the index into the sine wave table for the next interrupt
  // using fractional math
  uint8_t increment = major_inc1;
  acc_inc1 += minor_inc1;
  if (acc_inc1 > 32768)
  {
    acc_inc1 -= 32768;
    increment++;
  }
  index1 += increment;

  increment = major_inc2;
  acc_inc2 += minor_inc2;
  if (acc_inc2 > 32768)
  {
    acc_inc2 -= 32768;
    increment++;
  }
  index2 += increment;

  // toggle the output in PA5 - can be used for debug purposes
  // should be a 10Khz square wave on PA5 that can be used to tune TIMER2_FREQUENCY_HZ
  PORTA.OUTTGL = 0x20;
}

// the frequencies of the two sine waves - initial values
volatile uint16_t frequency1 = 700;
volatile uint16_t frequency2 = 1900;

void updateFrequencies()
{
  // frequency ranges is restricted to 100Hz - 4500Hz
  if (frequency1 < 100)
    frequency1 = 100;
  if (frequency1 > 4500)
    frequency1 = 4500;
    
  // calculate integer and fractional increment of index on interrupt
  float ticks = normalized_phase_increment * (float)frequency1;
  major_inc1 = (uint8_t)ticks;
  minor_inc1 = 32768 * (ticks - (float)major_inc1);

  if (frequency2 < 100)
    frequency2 = 100;
  if (frequency2 > 4500)
    frequency2 = 4500;

  ticks = normalized_phase_increment * (float)frequency2;
  major_inc2 = (uint8_t)ticks;
  minor_inc2 = 32768 * (ticks - (float)major_inc2);
}

// rotary encoder vars to determine which direction the dial is turning
// and if the button is pressed.  also a rudementary debouncing logic
volatile byte debounce = false;
volatile bool clockwise = false;
volatile bool counterclockwise = false;
volatile bool button = false;
volatile bool debounce_button = false;

// state - if true we are setting frequency1, else frequency2
volatile bool set_frequency1 = true;

// interrupt vector, services rotary encoder and button presses
ISR(PORTA_PORT_vect)
{
  byte flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags;
  if (debounce_button)
  {
    if ((PORTA.IN & 0x2) == 0x02)
      debounce_button = false;
  }
  else
  {
    if ((PORTA.IN & 0x2) == 0x0)
    {
      button = true;
      debounce_button = true;
    }
  }
  if (debounce)
  {
    if ((PORTA.IN & 0xC) == 0xC)
      debounce = false;
  }
  else
  {
    if ((PORTA.IN & 0xC) == 0)
    {
      debounce = true;
      if (flags & 0x04)
        clockwise = true;
      if (flags & 0x08)
        counterclockwise = true;
    }
  }
}

void update_display()
{
  oled.clear();
  oled.set2X();
  oled.printf("%c%4d Hz\n", (set_frequency1 ? '>' : ' '), (int)frequency1);
  oled.printf("%c%4d Hz", (set_frequency1 ? ' ' : '>'), (int)frequency2);
}


// the setup function runs once when you press reset or power the board
void setup() {
  // debug pin out - 10Khz square wave
  pinMode(PIN_PA5, OUTPUT);
  // interface to rotary encoder, set internal pull up resistors
  // and interrupt of both edges
  pinMode(PIN_PA3, INPUT_PULLUP);
  pinMode(PIN_PA2, INPUT_PULLUP);
  pinMode(PIN_PA1, INPUT_PULLUP);
  PORTA.PIN1CTRL |= 0x01;
  PORTA.PIN2CTRL |= 0x01;
  PORTA.PIN3CTRL |= 0x01;

  // our DAC is set to 1.1 Vref
  DACReference(INTERNAL1V1);

  // start the 20Khz interrupt timer
  CurrentTimer.init();
  CurrentTimer.attachInterrupt(TIMER2_FREQUENCY_HZ, TimerHandler);

  updateFrequencies();

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(X11fixed7x14B);
  update_display();
}

// the loop function runs over and over again forever
void loop() {
  bool update = false;
  if (button)
  {
    button = false;
    update = true;
    set_frequency1 = !set_frequency1;
  }
  if (clockwise)
  {
    if (set_frequency1)
      frequency1 += 50;
    else
      frequency2 += 50;
    updateFrequencies();
    update = true;
    clockwise = false;
  }
  if (counterclockwise)
  {
    if (set_frequency1)
      frequency1 -= 50;
    else
      frequency2 -= 50;
    updateFrequencies();
    update = true;
    counterclockwise = false;
  }

  if (update)
  {
    update_display();
  }
}
