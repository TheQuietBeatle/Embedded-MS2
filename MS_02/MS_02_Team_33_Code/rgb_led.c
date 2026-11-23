// rgb_led.c - Status LED driver: three separate LEDs (red, green, blue), one PWM channel each.
#include "rgb_led.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define LED_PWM_FREQ_HZ 1000u
#define LED_WRAP        255u     // brightness 0-255 maps straight to the PWM level

static uint8_t led_pin[3];

static void led_channel_init(uint8_t pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (LED_PWM_FREQ_HZ * (LED_WRAP + 1u)));
    pwm_config_set_wrap(&cfg, LED_WRAP);
    // GP6/GP7 share a slice; re-initialising it with the same config is harmless
    // because both colours start at 0 here.
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

void rgb_led_init(uint8_t pin_r, uint8_t pin_g, uint8_t pin_b) {
    led_pin[0] = pin_r;
    led_pin[1] = pin_g;
    led_pin[2] = pin_b;
    for (int i = 0; i < 3; i++) {
        led_channel_init(led_pin[i]);
    }
}

void rgb_led_set(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(led_pin[0], r);
    pwm_set_gpio_level(led_pin[1], g);
    pwm_set_gpio_level(led_pin[2], b);
}

void rgb_led_off(void) {
    rgb_led_set(0, 0, 0);
}
