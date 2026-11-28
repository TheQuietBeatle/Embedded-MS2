// motor.h - DC motor driver for one L298N channel (IN1, IN2, ENA).
//
// IN1/IN2 pick the direction, ENA is PWM for speed.
// Remove the ENA jumper on the L298N so the Pico controls ENA.
//
// Reward chute (Neural Calibrator brief):
//   SUCCESS -> motor turns CW to release the reward   -> reward_release()
//   FAILURE -> motor turns CCW for 5 s (error cue)     -> reward_fail()
// Both are NON-BLOCKING: they start the motor and return at once. Call
// motor_update() regularly (main loop now, the actuator RTOS task in MS3) and it
// stops the motor when the time is up. No sleep_ms() / busy-wait anywhere.
#ifndef MOTOR_H                         // include guard: skip this file if it was already included
#define MOTOR_H                         // mark this file as included

#include <stdbool.h>                    // bool type
#include <stdint.h>                     // int8_t, uint32_t

// ---- Reward chute settings ----------------------------------------------------
#define REWARD_SPEED        70      // percent, for both success and failure moves
#define REWARD_RELEASE_MS   2000u   // CW run time to drop the reward
#define FAIL_CUE_MS         5000u   // CCW run time on failure (5 s, from the brief)
// +1 if motor_set(+speed) turns the chute CW on our build, -1 if it turns CCW.
// Check once on the real chute and flip this instead of rewiring.
#define MOTOR_CW_SIGN       1           // multiplies the reward speeds in motor.c

void motor_init(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_ena);     // call once; leaves the motor off

// speed: -100 (full reverse) .. 0 (stop) .. +100 (full forward)
// Cancels any timed run that is in progress.
void motor_set(int8_t speed);           // values outside the range are clamped

// Free-running stop (outputs off, motor coasts). Cancels any timed run.
void motor_coast(void);                 // the chute may keep turning briefly

// Active brake (both motor terminals shorted through the driver). Cancels any timed run.
void motor_brake(void);                 // stops faster than coasting

// Last speed set with motor_set().
int8_t motor_get(void);                 // 0 after a coast or brake

// Run at `speed` for `ms` milliseconds, then coast. Non-blocking.
// ms == 0 just stops the motor.
void motor_run_for(int8_t speed, uint32_t ms);          // needs motor_update() to be called to stop

// Stop the motor if a timed run has finished. Call it periodically (e.g. every 10-20 ms).
void motor_update(void);                // does nothing when no timed run is active

// True while a timed run is still going.
bool motor_is_busy(void);               // cleared when the run ends or is cancelled

// SUCCESS: CW for REWARD_RELEASE_MS.
void reward_release(void);              // returns at once

// FAILURE: CCW for FAIL_CUE_MS.
void reward_fail(void);                 // returns at once

#endif // MOTOR_H
