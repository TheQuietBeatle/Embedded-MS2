// servo.c - Hobby servo driver (MG996R / MG90S / SG90) for the RP2040, Pico SDK.
#include "servo.h"                      // our own public interface

#include "pico/stdlib.h"                // GPIO functions and sleep_ms()
#include "hardware/pwm.h"               // Pico SDK PWM
#include "hardware/clocks.h"            // clock_get_hz()

#define SERVO_WRAP (1000000u / SERVO_FREQ_HZ - 1u)   // 19999 ticks of 1 us = 20 ms

// One flag per PWM slice so a shared slice is configured only once.
// (pwm_init() also clears both channels' levels, which would cut the
// partner servo's pulses if it were called a second time.)
static bool slice_ready[NUM_PWM_SLICES];                // all false at start-up

static float clampf(float v, float lo, float hi) {      // keep v inside [lo, hi]
    return v < lo ? lo : (v > hi ? hi : v);             // too low -> lo, too high -> hi, else v
}                                       // end of clampf

static uint16_t angle_to_us(float deg) {                // angle in degrees -> pulse width in us
    return (uint16_t)(SERVO_MIN_PULSE_US +              // start from the 0 deg pulse...
                      (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * deg / 180.0f + 0.5f);  // ...add the angle's share (+0.5 rounds)
}                                       // end of angle_to_us

void servo_init(servo_t *s, uint8_t pin, float min_deg, float max_deg) {  // see servo.h
    s->pin     = pin;                   // remember the pin
    s->slice   = (uint8_t)pwm_gpio_to_slice_num(pin);   // which PWM slice drives it
    s->channel = (uint8_t)pwm_gpio_to_channel(pin);     // which channel of that slice
    s->min_deg = min_deg;               // lower angle limit
    s->max_deg = max_deg;               // upper angle limit
    s->angle   = clampf(90.0f, min_deg, max_deg);       // assume centre until the first write
    s->active  = false;                 // no pulses yet

    gpio_set_function(pin, GPIO_FUNC_PWM);              // hand the pin over to the PWM hardware

    if (!slice_ready[s->slice]) {       // first servo on this slice?
        pwm_config cfg = pwm_get_default_config();      // start from the default settings
        // Divide the system clock down to 1 MHz so one counter tick = 1 us.
        pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);  // system clock / 1 MHz
        pwm_config_set_wrap(&cfg, SERVO_WRAP);          // 20 ms period
        pwm_init(s->slice, &cfg, true);       // both channels start at level 0
        slice_ready[s->slice] = true;   // don't configure this slice again
    }                                   // end of slice setup
    pwm_set_chan_level(s->slice, s->channel, 0);   // no pulses until first write
}                                       // end of servo_init

void servo_write(servo_t *s, float deg) {               // jump to an angle
    s->angle  = clampf(deg, s->min_deg, s->max_deg);    // clamp and remember it
    s->active = true;                   // pulses are now being sent
    pwm_set_chan_level(s->slice, s->channel, angle_to_us(s->angle));    // 1 count = 1 us of pulse
}                                       // end of servo_write

float servo_read(const servo_t *s) {    // last commanded angle
    return s->angle;                    // stored by servo_write()
}                                       // end of servo_read

bool servo_step_toward(servo_t *s, float target, float max_step) {      // one small move toward the target
    target = clampf(target, s->min_deg, s->max_deg);    // clamp first so the target is reachable
    float diff = target - s->angle;     // how far is left (signed)
    if (diff > max_step)       servo_write(s, s->angle + max_step);     // target far above: one step up
    else if (diff < -max_step) servo_write(s, s->angle - max_step);     // target far below: one step down
    else                       servo_write(s, target);  // close enough: go straight there
    return s->angle == target;          // true = arrived
}                                       // end of servo_step_toward

void servo_move_smooth(servo_t *s, float target, float step_deg, uint32_t step_ms) {  // blocking smooth move
    while (!servo_step_toward(s, target, step_deg)) {   // step until the target is reached
        sleep_ms(step_ms);              // pause between steps sets the speed
    }                                   // end of step loop
}                                       // end of servo_move_smooth

void servo_off(servo_t *s) {            // stop the pulses
    s->active = false;                  // servo is limp
    pwm_set_chan_level(s->slice, s->channel, 0);        // level 0 = output stays low
}                                       // end of servo_off
