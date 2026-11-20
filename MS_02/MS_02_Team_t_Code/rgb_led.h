// rgb_led.h - Status LED driver: three separate LEDs (red, green, blue),
// one PWM channel each.
//
// Wiring per LED: GPIO -> 220 ohm -> LED anode (long leg); cathode (short leg) -> GND.
// Brightness 0 (off) to 255 (full) per LED, 1 kHz PWM.
// "Mixed" colours (e.g. RGB_YELLOW) simply light more than one LED at once.
#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>

// Common colours for the station states
#define RGB_OFF     0,   0,   0
#define RGB_RED     255, 0,   0
#define RGB_GREEN   0,   255, 0
#define RGB_BLUE    0,   0,   255
#define RGB_YELLOW  255, 160, 0
#define RGB_WHITE   255, 255, 255

void rgb_led_init(uint8_t pin_r, uint8_t pin_g, uint8_t pin_b);

// Set each colour's brightness, 0-255.
void rgb_led_set(uint8_t r, uint8_t g, uint8_t b);

void rgb_led_off(void);

#endif // RGB_LED_H
