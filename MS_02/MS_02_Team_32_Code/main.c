// main.c - Milestone 2 driver test for the Neural Calibrator arm (Pico W, Pico SDK).
//
// Actuators (run once at start-up, then the sensor loop starts):
//   1. Servos   - home all six to 90 deg one at a time, then sweep each joint
//                 inside its limits at 2 deg / 20 ms.
//   2. LEDs     - red, yellow, green one at a time, then all three fade up and down.
//   3. Motor    - ramp forward to 70 %, back to 0, ramp reverse to 70 %, brake.
// Sensors (forever, verified on the LEDs as the brief asks):
//   - FSR force    -> grip traffic light: RED (light) -> YELLOW (close) -> GREEN (good),
//                     flashing RED above FSR_SAFETY (too hard)
//   - Mic loudness -> while the FSR is NOT pressed, the LEDs are a sound meter:
//                     GREEN (some sound) -> +YELLOW (louder) -> +RED (loud)
//   Both values are also printed over USB serial every ~100 ms.

#include <stdio.h>
#include "pico/stdlib.h"

#include "pins.h"
#include "servo.h"
#include "fsr.h"
#include "leds.h"
#include "motor.h"
#include "mic.h"

// ---- Tunable settings -------------------------------------------------------
#define HOME_ANGLE      90.0f
#define HOME_GAP_MS     400u     // pause between homing one joint and the next
#define JOINT_MIN       5.0f     // waist / shoulder / elbow / wrist limits
#define JOINT_MAX       175.0f
#define GRIPPER_CLOSED  0.0f     // gripper limits (0 = closed on this arm)
#define GRIPPER_OPEN    120.0f
#define SWEEP_SPAN      30.0f   // test sweep: 90 -> 90-span -> 90+span -> 90
#define STEP_DEG        2.0f     // at most 2 deg ...
#define STEP_MS         20u      // ... every 20 ms

#define MOTOR_TEST_SPEED 70      // percent

#define FSR_YELLOW_AT   100u     // force (0-1000) where the grip light turns yellow
#define FSR_GREEN_AT    175u     // ... turns green (= start of the web page's hold zone)
#define FSR_SAFETY      400u     // ... flashes red (= the web page's safety limit)
#define MIC_QUIET       10u      // loudness (above room noise) shown as LED off  (tune from the serial output)
#define MIC_LOUD        400u     // loudness shown as LED full

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
    servo_init(&servos[GRIPPER],     PIN_SERVO_GRIPPER,     GRIPPER_CLOSED, GRIPPER_OPEN);
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
            // Gripper (SG90): fully open, then closed (0 = closed on this arm).
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

// ---- LEDs -------------------------------------------------------------------
static void test_leds(void) {
    printf("[led] red, yellow, green, then fade\n");
    leds_set(255, 0, 0); sleep_ms(500);
    leds_set(0, 255, 0); sleep_ms(500);
    leds_set(0, 0, 255); sleep_ms(500);
    for (int v = 0; v <= 255; v += 5)  { leds_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }
    for (int v = 255; v >= 0; v -= 5)  { leds_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }
    leds_off();
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
// Sound meter on the three LEDs (bar graph): green, then +yellow, then +red.
static void leds_sound_meter(uint32_t level) {
    uint8_t green  = level > MIC_QUIET          ? 255 : 0;
    uint8_t yellow = level > MIC_LOUD / 3u      ? 255 : 0;
    uint8_t red    = level > MIC_LOUD * 2u / 3u ? 255 : 0;
    leds_set(red, yellow, green);
}

int main(void) {
    stdio_init_all();

    // Actuators off before anything else moves
    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);
    leds_init(PIN_LED_RED, PIN_LED_YELLOW, PIN_LED_GREEN);
    servos_init();
    fsr_init(PIN_FSR_ADC);

    sleep_ms(2000);   // time to open the USB serial monitor
    printf("\n=== Neural Calibrator - MS2 driver test ===\n");

    test_servos();
    test_leds();
    test_motor();

    bool mic_ok = mic_init(PIN_MIC_ADC);
    printf("[mic] %s (raw ADC %u, ~1551 = 1.25 V expected; room noise floor %lu)\n",
           mic_ok ? "MAX9814 on ADC running" : "INIT FAILED (pin must be GP26-28)",
           mic_ok ? mic_read_raw() : 0u, mic_ok ? (unsigned long)mic_noise_floor() : 0ul);

    printf("[sensors] press the FSR (red -> yellow -> green, flashing red = too hard); let go and talk/clap for the sound meter\n");
    while (true) {
        uint16_t force = fsr_read_smooth();                      // 0-1000, smoothed, light touch = 0
        uint32_t loud  = mic_ok ? mic_read_loudness(1600) : 0;   // 1600 samples = 100 ms, room noise removed

        if (force > 0) {
            leds_grip(force, FSR_YELLOW_AT, FSR_GREEN_AT, FSR_SAFETY);   // gripping: traffic light
        } else {
            leds_sound_meter(loud);                                       // not gripping: sound meter
        }

        printf("FSR raw %4u  units %4u  |  mic level %4lu\n",
               fsr_read_raw(), force, (unsigned long)loud);
        if (!mic_ok) sleep_ms(100);
    }
}
