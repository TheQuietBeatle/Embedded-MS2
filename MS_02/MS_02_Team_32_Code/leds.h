// leds.h - Status LED driver: three separate LEDs (red, yellow, green),
// one PWM channel each.
//
// Wiring per LED: GPIO -> 220 ohm -> LED anode (long leg); cathode (short leg) -> GND.
// Brightness 0 (off) to 255 (full) per LED, 1 kHz PWM.
#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

void leds_init(uint8_t pin_red, uint8_t pin_yellow, uint8_t pin_green);

// Set each LED's brightness, 0-255.
void leds_set(uint8_t red, uint8_t yellow, uint8_t green);

void leds_off(void);

#endif // LEDS_H
