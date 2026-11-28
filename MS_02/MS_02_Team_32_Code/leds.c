// leds.c - Status LED driver: three separate LEDs (red, yellow, green), one PWM channel each.
#include "leds.h"                       // our own public interface

#include "pico/stdlib.h"                // GPIO functions and the millisecond clock
#include "hardware/pwm.h"               // Pico SDK PWM
#include "hardware/clocks.h"            // clock_get_hz()

#define LED_PWM_FREQ_HZ 1000u           // 1 kHz: too fast for the eye to see flicker
#define LED_WRAP        255u     // brightness 0-255 maps straight to the PWM level

enum { LED_RED, LED_YELLOW, LED_GREEN, NUM_LEDS };      // array index of each LED; NUM_LEDS = 3
static uint8_t led_pin[NUM_LEDS];       // GPIO pin of each LED

static void led_channel_init(uint8_t pin) {             // set up PWM on one LED pin
    gpio_set_function(pin, GPIO_FUNC_PWM);              // hand the pin over to the PWM hardware
    uint slice = pwm_gpio_to_slice_num(pin);            // which PWM slice drives this pin
    pwm_config cfg = pwm_get_default_config();          // start from the default settings
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (LED_PWM_FREQ_HZ * (LED_WRAP + 1u)));  // divide the clock to get 1 kHz
    pwm_config_set_wrap(&cfg, LED_WRAP);                // count 0-255 per period
    // GP6/GP7 share a slice; re-initialising it with the same config is harmless
    // because every LED starts at 0 here.
    pwm_init(slice, &cfg, true);        // apply the settings and start the slice
    pwm_set_gpio_level(pin, 0);         // LED off
}                                       // end of led_channel_init

void leds_init(uint8_t pin_red, uint8_t pin_yellow, uint8_t pin_green) {  // see leds.h
    led_pin[LED_RED]    = pin_red;      // remember the red pin
    led_pin[LED_YELLOW] = pin_yellow;   // remember the yellow pin
    led_pin[LED_GREEN]  = pin_green;    // remember the green pin
    for (int i = 0; i < NUM_LEDS; i++) {                // for each LED...
        led_channel_init(led_pin[i]);   // ...set up its PWM
    }                                   // end of LED loop
}                                       // end of leds_init

void leds_set(uint8_t red, uint8_t yellow, uint8_t green) {             // set all three brightnesses
    pwm_set_gpio_level(led_pin[LED_RED],    red);       // red brightness
    pwm_set_gpio_level(led_pin[LED_YELLOW], yellow);    // yellow brightness
    pwm_set_gpio_level(led_pin[LED_GREEN],  green);     // green brightness
}                                       // end of leds_set

void leds_off(void) {                   // all LEDs off
    leds_set(0, 0, 0);                  // brightness 0 on each
}                                       // end of leds_off

void leds_grip(uint16_t force, uint16_t yellow_at, uint16_t green_at, uint16_t danger_at) {  // grip traffic light
    if (force == 0) {                   // not touched
        leds_off();                     // everything off
    } else if (force < yellow_at) {     // touched, but too light
        leds_set(255, 0, 0);                                    // red
    } else if (force < green_at) {      // getting close
        leds_set(0, 255, 0);                                    // yellow
    } else if (force < danger_at) {     // good grip
        leds_set(0, 0, 255);                                    // green
    } else {                            // too hard: flash to warn
        bool on = (to_ms_since_boot(get_absolute_time()) / 150u) % 2u == 0u;   // 150 ms on, 150 ms off
        leds_set(on ? 255 : 0, 0, 0);                           // flashing red
    }                                   // end of force ranges
}                                       // end of leds_grip
