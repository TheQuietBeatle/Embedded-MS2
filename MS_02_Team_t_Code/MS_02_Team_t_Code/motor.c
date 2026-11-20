// motor.c - DC motor driver for one L298N channel (IN1, IN2, ENA).
#include "motor.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define MOTOR_PWM_FREQ_HZ 1000u
#define MOTOR_WRAP        999u      // speed % * 10 = PWM level

static uint8_t in1, in2, ena;
static int8_t  current_speed;

void motor_init(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_ena) {
    in1 = pin_in1;
    in2 = pin_in2;
    ena = pin_ena;

    gpio_init(in1);
    gpio_set_dir(in1, GPIO_OUT);
    gpio_put(in1, 0);
    gpio_init(in2);
    gpio_set_dir(in2, GPIO_OUT);
    gpio_put(in2, 0);

    gpio_set_function(ena, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(ena);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (MOTOR_PWM_FREQ_HZ * (MOTOR_WRAP + 1u)));
    pwm_config_set_wrap(&cfg, MOTOR_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(ena, 0);       // motor off at start-up

    current_speed = 0;
}

void motor_set(int8_t speed) {
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;
    current_speed = speed;

    if (speed > 0) {          // forward
        gpio_put(in1, 1);
        gpio_put(in2, 0);
    } else if (speed < 0) {   // reverse
        gpio_put(in1, 0);
        gpio_put(in2, 1);
    } else {
        motor_coast();
        return;
    }
    uint16_t magnitude = (uint16_t)(speed < 0 ? -speed : speed);
    pwm_set_gpio_level(ena, magnitude * (MOTOR_WRAP + 1u) / 100u);
}

void motor_coast(void) {
    current_speed = 0;
    pwm_set_gpio_level(ena, 0);
    gpio_put(in1, 0);
    gpio_put(in2, 0);
}

void motor_brake(void) {
    current_speed = 0;
    gpio_put(in1, 1);
    gpio_put(in2, 1);
    pwm_set_gpio_level(ena, MOTOR_WRAP + 1u);   // full enable = short both terminals
}

int8_t motor_get(void) {
    return current_speed;
}
