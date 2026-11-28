// gripper.c - MG90S gripper servo on GP15. See gripper.h.
#include "gripper.h"                    // our own public interface

#include "servo.h"                      // servo_init() and the servo_t type
#include "hardware/pwm.h"               // Pico SDK PWM

// GP15 shares PWM slice 7 with the wrist-pitch servo on GP14. servo_init() does the
// one-time slice setup (50 Hz, 1 count = 1 us) and won't re-run pwm_init() on a
// slice that is already configured, so the wrist's pulses aren't cut. It is the
// ONLY servo.c function used here: servo_write() is never called on the gripper,
// so its 500-2500 us angle maths can't reach this pin.
static servo_t  pwm;                    // holds the PWM slice and channel of the gripper pin
static bool     ready;                  // true once gripper_init() has run
static bool     released = true;        // true = no pulses are being sent
static uint16_t cmd_us   = GRIPPER_CENTRE_US;           // last commanded pulse width

static uint16_t clamp_us(int32_t us) {  // force a pulse width into the safe range
    if (us < (int32_t)GRIPPER_MIN_US)      us = (int32_t)GRIPPER_MIN_US;  // below the range: use the low end
    if (us > (int32_t)GRIPPER_MAX_US)      us = (int32_t)GRIPPER_MAX_US;  // above the range: use the high end
    if (us > (int32_t)GRIPPER_HARD_MAX_US) us = (int32_t)GRIPPER_HARD_MAX_US;   // belt and braces
    return (uint16_t)us;                // now safe to send
}                                       // end of clamp_us

void gripper_init(uint8_t pin) {        // see gripper.h
    servo_init(&pwm, pin, 0.0f, 0.0f);     // slice setup only; output level 0 (no pulses)
    cmd_us   = clamp_us(GRIPPER_CENTRE_US);             // assume the jaws are at centre (no position feedback)
    released = true;                    // no pulses yet
    ready    = true;                    // the PWM is now set up
}                                       // end of gripper_init

uint16_t gripper_set_us(int32_t us) {   // the only way to move the gripper
    cmd_us = clamp_us(us);              // clamp and remember the request
    if (ready) {                        // only touch the PWM after init
        pwm_set_chan_level(pwm.slice, pwm.channel, cmd_us);   // 1 count = 1 us
        released = false;               // pulses are now being sent
    }                                   // end of ready check
    return cmd_us;                      // the clamped pulse
}                                       // end of gripper_set_us

bool gripper_step_toward(int32_t target_us, uint16_t max_step_us) {     // one small move toward the target
    int32_t target = clamp_us(target_us);               // clamp first so the target is reachable
    int32_t diff   = target - (int32_t)cmd_us;          // how far is left (signed)
    if (diff >  (int32_t)max_step_us) diff =  (int32_t)max_step_us;     // limit the move upward
    if (diff < -(int32_t)max_step_us) diff = -(int32_t)max_step_us;     // limit the move downward
    gripper_set_us((int32_t)cmd_us + diff);             // do the (limited) move
    return cmd_us == target;            // true = arrived
}                                       // end of gripper_step_toward

uint16_t gripper_get_us(void) {         // last commanded pulse
    return cmd_us;                      // already clamped
}                                       // end of gripper_get_us

void gripper_release(void) {            // stop sending pulses
    if (ready) pwm_set_chan_level(pwm.slice, pwm.channel, 0);   // only channel B: GP14 keeps running
    released = true;                    // remember that the gripper is limp
}                                       // end of gripper_release

bool gripper_is_released(void) {        // is the gripper limp?
    return released;                    // set by release, cleared by set_us
}                                       // end of gripper_is_released
