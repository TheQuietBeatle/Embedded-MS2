// rgb_led.h - Common-cathode RGB LED driver (one PWM channel per colour).
//
// Wiring: GPIO -> 220 ohm -> LED anode (R, G, B); LED cathode -> GND.
// Brightness 0 (off) to 255 (full) per colour, 1 kHz PWM.
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
