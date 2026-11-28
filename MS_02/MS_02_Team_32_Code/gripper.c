// gripper.c - MG90S gripper servo on GP15. See gripper.h.
#include "gripper.h"

#include "servo.h"
#include "hardware/pwm.h"

// GP15 shares PWM slice 7 with the wrist-pitch servo on GP14. servo_init() does the
// one-time slice setup (50 Hz, 1 count = 1 us) and won't re-run pwm_init() on a
// slice that is already configured, so the wrist's pulses aren't cut. It is the
// ONLY servo.c function used here: servo_write() is never called on the gripper,
// so its 500-2500 us angle maths can't reach this pin.
static servo_t  pwm;
static bool     ready;
static bool     released = true;
static uint16_t cmd_us   = GRIPPER_CENTRE_US;

static uint16_t clamp_us(int32_t us) {
    if (us < (int32_t)GRIPPER_MIN_US)      us = (int32_t)GRIPPER_MIN_US;
    if (us > (int32_t)GRIPPER_MAX_US)      us = (int32_t)GRIPPER_MAX_US;
    if (us > (int32_t)GRIPPER_HARD_MAX_US) us = (int32_t)GRIPPER_HARD_MAX_US;   // belt and braces
    return (uint16_t)us;
}

void gripper_init(uint8_t pin) {
    servo_init(&pwm, pin, 0.0f, 0.0f);     // slice setup only; output level 0 (no pulses)
    cmd_us   = clamp_us(GRIPPER_CENTRE_US);
    released = true;
    ready    = true;
}

uint16_t gripper_set_us(int32_t us) {
    cmd_us = clamp_us(us);
    if (ready) {
        pwm_set_chan_level(pwm.slice, pwm.channel, cmd_us);   // 1 count = 1 us
        released = false;
    }
    return cmd_us;
}

bool gripper_step_toward(int32_t target_us, uint16_t max_step_us) {
    int32_t target = clamp_us(target_us);
    int32_t diff   = target - (int32_t)cmd_us;
    if (diff >  (int32_t)max_step_us) diff =  (int32_t)max_step_us;
    if (diff < -(int32_t)max_step_us) diff = -(int32_t)max_step_us;
    gripper_set_us((int32_t)cmd_us + diff);
    return cmd_us == target;
}

uint16_t gripper_get_us(void) {
    return cmd_us;
}

void gripper_release(void) {
    if (ready) pwm_set_chan_level(pwm.slice, pwm.channel, 0);   // only channel B: GP14 keeps running
    released = true;
}

bool gripper_is_released(void) {
    return released;
}
