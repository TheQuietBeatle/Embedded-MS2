// motor.h - DC motor driver for one L298N channel (IN1, IN2, ENA).
//
// IN1/IN2 pick the direction, ENA is PWM for speed.
// Remove the ENA jumper on the L298N so the Pico controls ENA.
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_ena);

// speed: -100 (full reverse) .. 0 (stop) .. +100 (full forward)
void motor_set(int8_t speed);

// Free-running stop (outputs off, motor coasts).
void motor_coast(void);

// Active brake (both motor terminals shorted through the driver).
void motor_brake(void);

// Last speed set with motor_set().
int8_t motor_get(void);

#endif // MOTOR_H
