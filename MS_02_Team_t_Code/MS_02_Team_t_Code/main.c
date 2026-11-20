// main.c - Milestone 2 driver test for the Neural Calibrator arm (Pico W, Pico SDK).
//
// Actuators (run once at start-up, then the sensor loop starts):
//   1. Servos   - home all six to 90 deg one at a time, then sweep each joint
//                 inside its limits at 2 deg / 20 ms.
//   2. RGB LED  - red, green, blue, yellow, then a white fade up and down.
//   3. Motor    - ramp forward to 70 %, back to 0, ramp reverse to 70 %, brake.
// Sensors (forever, verified on the RGB LED as the brief asks):
//   - FSR force    -> GREEN brightness (press harder = brighter)
//   - Mic loudness -> RED brightness   (talk or clap = brighter)
//   Both values are also printed over USB serial every ~100 ms.

#include <stdio.h>
#include "pico/stdlib.h"

#include "pins.h"
#include "servo.h"
#include "fsr.h"
#include "rgb_led.h"
#include "motor.h"
#include "mic.h"

// ---- Tunable settings -------------------------------------------------------
#define HOME_ANGLE      90.0f
#define HOME_GAP_MS     400u     // pause between homing one joint and the next
#define JOINT_MIN       5.0f     // waist / shoulder / elbow / wrist limits
#define JOINT_MAX       175.0f
#define GRIPPER_OPEN    0.0f     // gripper limits (0 = open)
#define GRIPPER_CLOSED  120.0f
#define SWEEP_SPAN      30.0f    // test sweep: 90 -> 90-span -> 90+span -> 90
#define STEP_DEG        2.0f     // at most 2 deg ...
#define STEP_MS         20u      // ... every 20 ms

#define MOTOR_TEST_SPEED 70      // percent

#define MIC_QUIET       500u     // mic level shown as LED off  (tune from the serial output)
#define MIC_LOUD        60000u   // mic level shown as LED full

// ---- Servos -----------------------------------------------------------------
enum { WAIST, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER, NUM_SERVOS };

static const char *servo_name[NUM_SERVOS] = {
    "waist", "shoulder", "elbow", "wrist pitch", "wrist roll", "gripper"
};
static servo_t servos[NUM_SERVOS];

static void servos_init(void) {
    // All outputs stay silent here; homing below powers them one at a time.
    servo_init(&servos[WAIST],       PIN_SERVO_WAIST,       JOINT_MIN, JOINT_MAX);
    servo_init(&servos[SHOULDER],    PIN_SERVO_SHOULDER,    JOINT_MIN, JOINT_MAX);
    servo_init(&servos[ELBOW],       PIN_SERVO_ELBOW,       JOINT_MIN, JOINT_MAX);
    servo_init(&servos[WRIST_PITCH], PIN_SERVO_WRIST_PITCH, JOINT_MIN, JOINT_MAX);
    servo_init(&servos[WRIST_ROLL],  PIN_SERVO_WRIST_ROLL,  JOINT_MIN, JOINT_MAX);
    servo_init(&servos[GRIPPER],     PIN_SERVO_GRIPPER,     GRIPPER_OPEN, GRIPPER_CLOSED);
}

static void test_servos(void) {
    printf("[servo] homing one joint at a time\n");
    for (int i = 0; i < NUM_SERVOS; i++) {
        printf("  %s -> %.0f deg\n", servo_name[i], HOME_ANGLE);
        servo_write(&servos[i], HOME_ANGLE);
        sleep_ms(HOME_GAP_MS);
    }

    printf("[servo] sweeping each joint\n");
    for (int i = 0; i < NUM_SERVOS; i++) {
        servo_t *s = &servos[i];
        if (i == GRIPPER) {
            printf("  gripper: open -> closed -> 90\n");
            servo_move_smooth(s, GRIPPER_OPEN,   STEP_DEG, STEP_MS);
            servo_move_smooth(s, GRIPPER_CLOSED, STEP_DEG, STEP_MS);
        } else {
            printf("  %s: %.0f -> %.0f -> 90\n", servo_name[i],
                   HOME_ANGLE - SWEEP_SPAN, HOME_ANGLE + SWEEP_SPAN);
            servo_move_smooth(s, HOME_ANGLE - SWEEP_SPAN, STEP_DEG, STEP_MS);
            servo_move_smooth(s, HOME_ANGLE + SWEEP_SPAN, STEP_DEG, STEP_MS);
        }
        servo_move_smooth(s, HOME_ANGLE, STEP_DEG, STEP_MS);
        printf("    now at %.0f deg\n", servo_read(s));
    }
}

// ---- RGB LED ----------------------------------------------------------------
static void test_rgb_led(void) {
    printf("[led] red, green, blue, yellow, white fade\n");
    rgb_led_set(RGB_RED);    sleep_ms(500);
    rgb_led_set(RGB_GREEN);  sleep_ms(500);
    rgb_led_set(RGB_BLUE);   sleep_ms(500);
    rgb_led_set(RGB_YELLOW); sleep_ms(500);
    for (int v = 0; v <= 255; v += 5)  { rgb_led_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }
    for (int v = 255; v >= 0; v -= 5)  { rgb_led_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }
    rgb_led_off();
}

// ---- Motor ------------------------------------------------------------------
static void ramp_motor(int8_t from, int8_t to) {
    int8_t step = (to > from) ? 5 : -5;
    for (int8_t v = from; v != to; v += step) {
        motor_set(v);
        sleep_ms(60);
    }
    motor_set(to);
}

static void test_motor(void) {
    printf("[motor] forward ramp to %d %%\n", MOTOR_TEST_SPEED);
    ramp_motor(0, MOTOR_TEST_SPEED);
    sleep_ms(1000);
    ramp_motor(MOTOR_TEST_SPEED, 0);
    sleep_ms(300);
    printf("[motor] reverse ramp to %d %%\n", MOTOR_TEST_SPEED);
    ramp_motor(0, -MOTOR_TEST_SPEED);
    sleep_ms(1000);
    ramp_motor(-MOTOR_TEST_SPEED, 0);
    motor_brake();
    sleep_ms(200);
    motor_coast();
    printf("[motor] stopped\n");
}

// ---- Sensors -> LED -----------------------------------------------------------
static uint8_t mic_to_brightness(uint32_t level) {
    if (level <= MIC_QUIET) return 0;
    if (level >= MIC_LOUD)  return 255;
    return (uint8_t)((level - MIC_QUIET) * 255u / (MIC_LOUD - MIC_QUIET));
}

int main(void) {
    stdio_init_all();

    // Actuators off before anything else moves
    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);
    rgb_led_init(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    servos_init();
    fsr_init(PIN_FSR_ADC);

    sleep_ms(2000);   // time to open the USB serial monitor
    printf("\n=== Neural Calibrator - MS2 driver test ===\n");

    test_servos();
    test_rgb_led();
    test_motor();

    bool mic_ok = mic_init(PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_SD);
    printf("[mic] %s\n", mic_ok ? "I2S receiver running" : "INIT FAILED (check pins)");

    printf("[sensors] press the FSR (green) and talk/clap near the mic (red)\n");
    while (true) {
        uint16_t force = fsr_read_units();                       // 0-1000
        uint32_t loud  = mic_ok ? mic_read_level(1600) : 0;      // 1600 samples = 100 ms

        uint8_t green = (uint8_t)(force * 255u / 1000u);
        uint8_t red   = mic_to_brightness(loud);
        rgb_led_set(red, green, 0);

        printf("FSR raw %4u  units %4u  |  mic level %7lu  ->  LED R%3u G%3u\n",
               fsr_read_raw(), force, (unsigned long)loud, red, green);
        if (!mic_ok) sleep_ms(100);
    }
}
