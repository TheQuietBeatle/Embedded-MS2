// pins.h - pin constants (one place for every GPIO the project uses).
// Must match the physical wiring; see wiring.md.
#ifndef PINS_H                          // include guard: skip this file if it was already included
#define PINS_H                          // mark this file as included

#define PIN_SERVO_WAIST           0u  // GP0 -> S1.SIG
#define PIN_SERVO_SHOULDER        1u  // GP1 -> S2.SIG
#define PIN_SERVO_ELBOW           4u  // GP4 -> S3.SIG
#define PIN_SERVO_WRIST_ROLL      5u  // GP5 -> S4.SIG (wrist ROLL servo now plugged here; was wrist pitch)

#define PIN_LED_RED               6u  // GP6 -> 220R -> red LED
#define PIN_LED_GREEN             7u  // GP7 -> 220R -> green LED
#define PIN_LED_YELLOW            8u  // GP8 -> 220R -> yellow LED (where the RGB's blue was)


#define PIN_SERVO_WRIST_PITCH    14u  // GP14 -> S5.SIG (wrist PITCH servo now plugged here; was wrist roll)
#define PIN_SERVO_GRIPPER        15u  // GP15 -> S6.SIG

#define PIN_MOTOR_IN1            16u  // GP16 -> U1.IN1
#define PIN_MOTOR_IN2            17u  // GP17 -> U1.IN2
#define PIN_MOTOR_ENA            18u  // GP18 -> U1.ENA

#define PIN_FSR_ADC              26u  // GP26 -> FSR1.2
#define PIN_MIC_ADC              27u  // GP27 -> MIC1.OUT (MAX9814 analog out; was INMP441 on GP10-12)

#endif // PINS_H
