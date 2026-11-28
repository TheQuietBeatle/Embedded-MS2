// gripper.h - MG90S gripper servo on GP15 (Pico SDK).
//
// This module is the ONLY code allowed to drive the gripper's PWM. Every move goes
// through gripper_set_us(), which clamps to the range between GRIPPER_OPEN_US and
// GRIPPER_CLOSE_US (either may be the bigger one) and never sends more than
// GRIPPER_HARD_MAX_US. The gripper works in microseconds, not degrees, so no angle
// maths can push it past the calibrated range.
//
// Measured on the arm (50 Hz, 1 PWM count = 1 us):
//   bigger pulse = MORE OPEN   (the first calibration had this backwards)
//   1500 us  closes further still is possible -> measure the real closed point
//   1700 us  opens  further still is possible -> measure the real open point
//   >2000 us jaws blocked, servo stalls       -> hard ceiling 1950 us
#ifndef GRIPPER_H                       // include guard: skip this file if it was already included
#define GRIPPER_H                       // mark this file as included

#include <stdbool.h>                    // bool type
#include <stdint.h>                     // uint16_t, int32_t

#define GRIPPER_OPEN_US      1900u   // TESTING: 1700 -> 1750 -> 1800. Raise 50 at a time until it strains (max 1950).
#define GRIPPER_CLOSE_US     1400u   // TESTING: 1500 -> 1450 -> 1400. Lower 50 at a time until the jaws touch.
#define GRIPPER_CENTRE_US    1600u   // half way between the two
#define GRIPPER_HARD_MAX_US  1950u   // absolute ceiling, never exceeded whatever the settings

// Low and high ends of the allowed range, whichever way round open/close are.
#define GRIPPER_MIN_US  (GRIPPER_OPEN_US < GRIPPER_CLOSE_US ? GRIPPER_OPEN_US : GRIPPER_CLOSE_US)  // the smaller of open / close
#define GRIPPER_MAX_US  (GRIPPER_OPEN_US > GRIPPER_CLOSE_US ? GRIPPER_OPEN_US : GRIPPER_CLOSE_US)  // the bigger of open / close

#if GRIPPER_OPEN_US == GRIPPER_CLOSE_US                 // compile-time check: the range must not be empty
#error "GRIPPER_OPEN_US and GRIPPER_CLOSE_US must be different."        // stop the build
#endif                                  // end of range check
#if GRIPPER_OPEN_US > GRIPPER_HARD_MAX_US || GRIPPER_CLOSE_US > GRIPPER_HARD_MAX_US  // compile-time check: neither end above the ceiling
#error "GRIPPER_OPEN_US and GRIPPER_CLOSE_US must not exceed GRIPPER_HARD_MAX_US (1950 us)."  // stop the build
#endif                                  // end of ceiling check

// Set up the PWM pin (50 Hz, 1 count = 1 us). The output stays LOW (released)
// until the first gripper_set_us().
void     gripper_init(uint8_t pin);     // call once at start-up

// The single gate for every gripper move: clamps `us` to MIN..MAX (and to
// HARD_MAX), drives the pin, and returns the pulse actually sent.
uint16_t gripper_set_us(int32_t us);    // out-of-range requests are clamped, not rejected

// Move at most max_step_us toward target_us (clamped). Returns true once there.
bool     gripper_step_toward(int32_t target_us, uint16_t max_step_us);  // call it periodically

// Last commanded pulse (always inside OPEN..CLOSE).
uint16_t gripper_get_us(void);          // keeps the last value while released

// Stop the pulses (PWM level 0): the servo goes limp and draws no stall current.
// The next gripper_set_us() turns it back on.
void     gripper_release(void);         // the PWM slice keeps running for the wrist servo

// True while no pulses are being sent.
bool     gripper_is_released(void);     // starts true after gripper_init()

#endif // GRIPPER_H
