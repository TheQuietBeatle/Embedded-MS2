// servo.h - Hobby servo driver (MG996R / MG90S / SG90) for the RP2040, Pico SDK.
//
// 50 Hz PWM, 1 tick = 1 us, pulse 500-2500 us for 0-180 deg.
// Each servo has its own angle limits; every write is clamped to them.
// Two servos may share one PWM slice (e.g. GP0/GP1): the slice is configured
// once and each servo only touches its own channel.
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

#define SERVO_FREQ_HZ      50u
#define SERVO_MIN_PULSE_US 500u    // pulse for 0 deg
#define SERVO_MAX_PULSE_US 2500u   // pulse for 180 deg

typedef struct {
    uint8_t  pin;
    uint8_t  slice;
    uint8_t  channel;
    float    min_deg;      // lower safety limit
    float    max_deg;      // upper safety limit
    float    angle;        // last commanded angle (deg)
    bool     active;       // false until the first write: output stays low (servo limp)
} servo_t;

// Configure the pin for 50 Hz PWM. Output stays LOW (no pulses) until the
// first servo_write(), so servos can be powered up one at a time.
void  servo_init(servo_t *s, uint8_t pin, float min_deg, float max_deg);

// Jump straight to an angle (clamped to the servo's limits).
void  servo_write(servo_t *s, float deg);

// Last commanded angle.
float servo_read(const servo_t *s);

// Move at most max_step deg toward target. Returns true once the target is reached.
// Non-blocking: call it from a periodic loop.
bool  servo_step_toward(servo_t *s, float target, float max_step);

// Blocking smooth move: step_deg every step_ms until target is reached.
void  servo_move_smooth(servo_t *s, float target, float step_deg, uint32_t step_ms);

// Stop sending pulses (servo goes limp, holds no torque).
void  servo_off(servo_t *s);

#endif // SERVO_H
