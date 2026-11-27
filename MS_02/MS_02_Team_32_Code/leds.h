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

// Grip traffic light for the FSR force (0-1000 units). Pressing harder goes:
//   0 (not touched)            -> all off
//   1 .. yellow_at-1           -> RED     (too light)
//   yellow_at .. green_at-1    -> YELLOW  (getting close)
//   green_at .. danger_at-1    -> GREEN   (good grip)
//   danger_at and above        -> RED flashing (too hard - back off)
// Call it every time a new force value is read.
void leds_grip(uint16_t force, uint16_t yellow_at, uint16_t green_at, uint16_t danger_at);

#endif // LEDS_H
