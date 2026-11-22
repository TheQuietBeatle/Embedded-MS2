// pins.h - pin constants generated from the wiring spec (spec.json). Do not edit by hand.
// Must match the physical wiring; see wiring.md.
#ifndef PINS_H
#define PINS_H

#define PIN_SERVO_WAIST           0u  // GP0 -> S1.SIG
#define PIN_SERVO_SHOULDER        1u  // GP1 -> S2.SIG
#define PIN_SERVO_ELBOW           4u  // GP4 -> S3.SIG
#define PIN_SERVO_WRIST_ROLL      5u  // GP5 -> S4.SIG (wrist ROLL servo now plugged here; was wrist pitch)

#define PIN_LED_R                 6u  // GP6 -> R2.1
#define PIN_LED_G                 7u  // GP7 -> R3.1
#define PIN_LED_B                 8u  // GP8 -> R4.1

#define PIN_MIC_SCK              10u  // GP10 -> MIC1.SCK
#define PIN_MIC_WS               11u  // GP11 -> MIC1.WS
#define PIN_MIC_SD               12u  // GP12 -> MIC1.SD

#define PIN_SERVO_WRIST_PITCH    14u  // GP14 -> S5.SIG (wrist PITCH servo now plugged here; was wrist roll)
#define PIN_SERVO_GRIPPER        15u  // GP15 -> S6.SIG

#define PIN_MOTOR_IN1            16u  // GP16 -> U1.IN1
#define PIN_MOTOR_IN2            17u  // GP17 -> U1.IN2
#define PIN_MOTOR_ENA            18u  // GP18 -> U1.ENA

#define PIN_FSR_ADC              26u  // GP26 -> FSR1.2

#endif // PINS_H
