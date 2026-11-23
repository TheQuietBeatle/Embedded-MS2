// servo.c - Hobby servo driver (MG996R / SG90) for the RP2040, Pico SDK.
#include "servo.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define SERVO_WRAP (1000000u / SERVO_FREQ_HZ - 1u)   // 19999 ticks of 1 us = 20 ms

// One flag per PWM slice so a shared slice is configured only once.
// (pwm_init() also clears both channels' levels, which would cut the
// partner servo's pulses if it were called a second time.)
static bool slice_ready[NUM_PWM_SLICES];

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint16_t angle_to_us(float deg) {
    return (uint16_t)(SERVO_MIN_PULSE_US +
                      (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * deg / 180.0f + 0.5f);
}

void servo_init(servo_t *s, uint8_t pin, float min_deg, float max_deg) {
    s->pin     = pin;
    s->slice   = (uint8_t)pwm_gpio_to_slice_num(pin);
    s->channel = (uint8_t)pwm_gpio_to_channel(pin);
    s->min_deg = min_deg;
    s->max_deg = max_deg;
    s->angle   = clampf(90.0f, min_deg, max_deg);
    s->active  = false;

    gpio_set_function(pin, GPIO_FUNC_PWM);

    if (!slice_ready[s->slice]) {
        pwm_config cfg = pwm_get_default_config();
        // Divide the system clock down to 1 MHz so one counter tick = 1 us.
        pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
        pwm_config_set_wrap(&cfg, SERVO_WRAP);
        pwm_init(s->slice, &cfg, true);       // both channels start at level 0
        slice_ready[s->slice] = true;
    }
    pwm_set_chan_level(s->slice, s->channel, 0);   // no pulses until first write
}

void servo_write(servo_t *s, float deg) {
    s->angle  = clampf(deg, s->min_deg, s->max_deg);
    s->active = true;
    pwm_set_chan_level(s->slice, s->channel, angle_to_us(s->angle));
}

void servo_write_us(servo_t *s, uint16_t us) {
    if (us < SERVO_MIN_PULSE_US) us = SERVO_MIN_PULSE_US;
    if (us > SERVO_MAX_PULSE_US) us = SERVO_MAX_PULSE_US;
    s->active = true;
    pwm_set_chan_level(s->slice, s->channel, us);   // 1 tick = 1 us
}

float servo_read(const servo_t *s) {
    return s->angle;
}

bool servo_step_toward(servo_t *s, float target, float max_step) {
    target = clampf(target, s->min_deg, s->max_deg);
    float diff = target - s->angle;
    if (diff > max_step)       servo_write(s, s->angle + max_step);
    else if (diff < -max_step) servo_write(s, s->angle - max_step);
    else                       servo_write(s, target);
    return s->angle == target;
}

void servo_move_smooth(servo_t *s, float target, float step_deg, uint32_t step_ms) {
    while (!servo_step_toward(s, target, step_deg)) {
        sleep_ms(step_ms);
    }
}

void servo_off(servo_t *s) {
    s->active = false;
    pwm_set_chan_level(s->slice, s->channel, 0);
}
