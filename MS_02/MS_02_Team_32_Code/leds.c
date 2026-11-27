// leds.c - Status LED driver: three separate LEDs (red, yellow, green), one PWM channel each.
#include "leds.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define LED_PWM_FREQ_HZ 1000u
#define LED_WRAP        255u     // brightness 0-255 maps straight to the PWM level

enum { LED_RED, LED_YELLOW, LED_GREEN, NUM_LEDS };
static uint8_t led_pin[NUM_LEDS];

static void led_channel_init(uint8_t pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (LED_PWM_FREQ_HZ * (LED_WRAP + 1u)));
    pwm_config_set_wrap(&cfg, LED_WRAP);
    // GP6/GP7 share a slice; re-initialising it with the same config is harmless
    // because every LED starts at 0 here.
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

void leds_init(uint8_t pin_red, uint8_t pin_yellow, uint8_t pin_green) {
    led_pin[LED_RED]    = pin_red;
    led_pin[LED_YELLOW] = pin_yellow;
    led_pin[LED_GREEN]  = pin_green;
    for (int i = 0; i < NUM_LEDS; i++) {
        led_channel_init(led_pin[i]);
    }
}

void leds_set(uint8_t red, uint8_t yellow, uint8_t green) {
    pwm_set_gpio_level(led_pin[LED_RED],    red);
    pwm_set_gpio_level(led_pin[LED_YELLOW], yellow);
    pwm_set_gpio_level(led_pin[LED_GREEN],  green);
}

void leds_off(void) {
    leds_set(0, 0, 0);
}

void leds_grip(uint16_t force, uint16_t yellow_at, uint16_t green_at, uint16_t danger_at) {
    if (force == 0) {
        leds_off();
    } else if (force < yellow_at) {
        leds_set(255, 0, 0);                                    // red
    } else if (force < green_at) {
        leds_set(0, 255, 0);                                    // yellow
    } else if (force < danger_at) {
        leds_set(0, 0, 255);                                    // green
    } else {
        bool on = (to_ms_since_boot(get_absolute_time()) / 150u) % 2u == 0u;   // 150 ms on, 150 ms off
        leds_set(on ? 255 : 0, 0, 0);                           // flashing red
    }
}
