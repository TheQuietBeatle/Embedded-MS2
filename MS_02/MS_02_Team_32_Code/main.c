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

#include <stdio.h>                      // printf
#include "pico/stdlib.h"                // Pico SDK basics: GPIO, sleep_ms, stdio

#include "pins.h"                       // which GPIO each part is wired to
#include "servo.h"                      // arm servo driver
#include "gripper.h"                    // gripper driver (limits in us)
#include "fsr.h"                        // force sensor driver
#include "leds.h"                       // status LED driver
#include "motor.h"                      // DC reward motor driver
#include "mic.h"                        // microphone driver

// ---- Tunable settings -------------------------------------------------------
#define HOME_ANGLE      90.0f           // home position of every joint except the gripper
#define HOME_GAP_MS     400u     // pause between homing one joint and the next
#define JOINT_MIN       5.0f     // waist / shoulder / elbow / wrist limits
#define JOINT_MAX       175.0f          // highest angle a joint may go to
// Gripper limits are in gripper.h (microseconds, bigger pulse = more open)
#define GRIPPER_STEP_US 22u      // gripper test: at most 22 us every STEP_MS (~2 deg)
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
enum { WAIST, SHOULDER, ELBOW, WRIST_PITCH, WRIST_ROLL, GRIPPER, NUM_SERVOS };  // joint numbers 0..5; NUM_SERVOS = 6

static const char *servo_name[NUM_SERVOS] = {           // names for the serial output
    "waist", "shoulder", "elbow", "wrist pitch", "wrist roll", "gripper"  // same order as the enum above
};                                      // end of name table
static servo_t servos[NUM_SERVOS];   // servos[GRIPPER] is unused: the gripper is driven by gripper.c

// Blocking smooth gripper move, through gripper_set_us() (clamped).
static void gripper_move_smooth(int32_t target_us) {    // see the comment above
    while (!gripper_step_toward(target_us, GRIPPER_STEP_US)) {          // step until the target is reached
        sleep_ms(STEP_MS);              // wait for the next step
    }                                   // end of step loop
}                                       // end of gripper_move_smooth

static void servos_init(void) {         // set up all six servo pins
    // All outputs stay silent here; homing below powers them one at a time.
    servo_init(&servos[WAIST],       PIN_SERVO_WAIST,       JOINT_MIN, JOINT_MAX);  // base rotation
    servo_init(&servos[SHOULDER],    PIN_SERVO_SHOULDER,    JOINT_MIN, JOINT_MAX);  // shoulder lift
    servo_init(&servos[ELBOW],       PIN_SERVO_ELBOW,       JOINT_MIN, JOINT_MAX);  // elbow bend
    servo_init(&servos[WRIST_PITCH], PIN_SERVO_WRIST_PITCH, JOINT_MIN, JOINT_MAX);  // wrist up / down
    servo_init(&servos[WRIST_ROLL],  PIN_SERVO_WRIST_ROLL,  JOINT_MIN, JOINT_MAX);  // wrist twist
    gripper_init(PIN_SERVO_GRIPPER);    // gripper has its own driver
}                                       // end of servos_init

static void test_servos(void) {         // home every joint, then sweep it
    printf("[servo] homing one joint at a time\n");     // say what is happening
    for (int i = 0; i < NUM_SERVOS; i++) {              // every joint in turn
        if (i == GRIPPER) {             // the gripper works in microseconds
            printf("  gripper -> %u us (centre)\n", gripper_set_us(GRIPPER_CENTRE_US));  // centre it and print the pulse sent
        } else {                        // every other joint works in degrees
            printf("  %s -> %.0f deg\n", servo_name[i], HOME_ANGLE);    // say which joint moves
            servo_write(&servos[i], HOME_ANGLE);        // jump to 90 deg
        }                               // end of joint type check
        sleep_ms(HOME_GAP_MS);          // pause so the currents don't add up
    }                                   // end of homing loop

    printf("[servo] sweeping each joint\n");            // say what is happening
    for (int i = 0; i < NUM_SERVOS; i++) {              // every joint in turn
        if (i == GRIPPER) {             // the gripper has its own sweep
            // Gripper (MG90S): open -> closed -> centre, all clamped in gripper.c.
            printf("  gripper: %u us (open) -> %u us (closed) -> %u us\n",  // print the planned moves
                   GRIPPER_OPEN_US, GRIPPER_CLOSE_US, GRIPPER_CENTRE_US);  // values for the line above
            gripper_move_smooth(GRIPPER_OPEN_US);       // open the jaws
            gripper_move_smooth(GRIPPER_CLOSE_US);      // close the jaws
            gripper_move_smooth(GRIPPER_CENTRE_US);     // back to the centre
            printf("    now at %u us\n", gripper_get_us());             // report the final pulse
            continue;                   // gripper done: next joint
        }                               // end of gripper sweep
        servo_t *s = &servos[i];        // shortcut to this joint's servo
        printf("  %s: %.0f -> %.0f -> 90\n", servo_name[i],             // print the planned moves
               HOME_ANGLE - SWEEP_SPAN, HOME_ANGLE + SWEEP_SPAN);       // 60 deg and 120 deg
        servo_move_smooth(s, HOME_ANGLE - SWEEP_SPAN, STEP_DEG, STEP_MS);  // down to 60 deg
        servo_move_smooth(s, HOME_ANGLE + SWEEP_SPAN, STEP_DEG, STEP_MS);  // up to 120 deg
        servo_move_smooth(s, HOME_ANGLE, STEP_DEG, STEP_MS);            // back to 90 deg
        printf("    now at %.0f deg\n", servo_read(s));                 // report the final angle
    }                                   // end of sweep loop
    gripper_release();   // test done, arm idle from here on: no gripper pulses
    printf("[servo] gripper released (no pulses)\n");   // say so
}                                       // end of test_servos

// ---- LEDs -------------------------------------------------------------------
static void test_leds(void) {           // show every LED works
    printf("[led] red, yellow, green, then fade\n");    // say what is happening
    leds_set(255, 0, 0); sleep_ms(500);                 // red only, 0.5 s
    leds_set(0, 255, 0); sleep_ms(500);                 // yellow only, 0.5 s
    leds_set(0, 0, 255); sleep_ms(500);                 // green only, 0.5 s
    for (int v = 0; v <= 255; v += 5)  { leds_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }  // all three fade up
    for (int v = 255; v >= 0; v -= 5)  { leds_set((uint8_t)v, (uint8_t)v, (uint8_t)v); sleep_ms(10); }  // all three fade down
    leds_off();                         // finish with all off
}                                       // end of test_leds

// ---- Motor ------------------------------------------------------------------
static void ramp_motor(int8_t from, int8_t to) {        // change speed gradually
    int8_t step = (to > from) ? 5 : -5;                 // 5 % per step, up or down
    for (int8_t v = from; v != to; v += step) {         // from the start speed until the end speed
        motor_set(v);                   // apply this speed
        sleep_ms(60);                   // 60 ms per step
    }                                   // end of ramp loop
    motor_set(to);                      // finish exactly on the end speed
}                                       // end of ramp_motor

static void test_motor(void) {          // run the motor both ways
    printf("[motor] forward ramp to %d %%\n", MOTOR_TEST_SPEED);        // say what is happening
    ramp_motor(0, MOTOR_TEST_SPEED);    // speed up forward
    sleep_ms(1000);                     // hold for 1 s
    ramp_motor(MOTOR_TEST_SPEED, 0);    // slow down to 0
    sleep_ms(300);                      // short rest before reversing
    printf("[motor] reverse ramp to %d %%\n", MOTOR_TEST_SPEED);        // say what is happening
    ramp_motor(0, -MOTOR_TEST_SPEED);   // speed up in reverse
    sleep_ms(1000);                     // hold for 1 s
    ramp_motor(-MOTOR_TEST_SPEED, 0);   // slow down to 0
    motor_brake();                      // active brake
    sleep_ms(200);                      // hold the brake briefly
    motor_coast();                      // then release it
    printf("[motor] stopped\n");        // say so
}                                       // end of test_motor

// ---- Sensors -> LED -----------------------------------------------------------
// Sound meter on the three LEDs (bar graph): green, then +yellow, then +red.
static void leds_sound_meter(uint32_t level) {          // level = loudness above the room noise
    uint8_t green  = level > MIC_QUIET          ? 255 : 0;              // any sound
    uint8_t yellow = level > MIC_LOUD / 3u      ? 255 : 0;              // above 1/3 of "loud"
    uint8_t red    = level > MIC_LOUD * 2u / 3u ? 255 : 0;              // above 2/3 of "loud"
    leds_set(red, yellow, green);       // show the bar
}                                       // end of leds_sound_meter

int main(void) {                        // program starts here
    stdio_init_all();                   // enable printf over USB serial

    // Actuators off before anything else moves
    motor_init(PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_ENA);            // motor off
    leds_init(PIN_LED_RED, PIN_LED_YELLOW, PIN_LED_GREEN);              // LEDs off
    servos_init();                      // servos silent
    fsr_init(PIN_FSR_ADC);              // force sensor ready

    sleep_ms(2000);   // time to open the USB serial monitor
    printf("\n=== Neural Calibrator - MS2 driver test ===\n");          // start-up banner

    test_servos();                      // test 1: servos
    test_leds();                        // test 2: LEDs
    test_motor();                       // test 3: motor

    bool mic_ok = mic_init(PIN_MIC_ADC);                // start the mic (stay quiet meanwhile)
    printf("[mic] %s (raw ADC %u, ~1551 = 1.25 V expected; room noise floor %lu)\n",  // report the result
           mic_ok ? "MAX9814 on ADC running" : "INIT FAILED (pin must be GP26-28)",  // status text
           mic_ok ? mic_read_raw() : 0u, mic_ok ? (unsigned long)mic_noise_floor() : 0ul);  // raw reading and room noise (0 if it failed)

    printf("[sensors] press the FSR (red -> yellow -> green, flashing red = too hard); let go and talk/clap for the sound meter\n");  // tell the user what to do
    while (true) {                      // sensor loop, runs forever
        uint16_t force = fsr_read_smooth();                      // 0-1000, smoothed, light touch = 0
        uint32_t loud  = mic_ok ? mic_read_loudness(1600) : 0;   // 1600 samples = 100 ms, room noise removed

        if (force > 0) {                // is the FSR pressed?
            leds_grip(force, FSR_YELLOW_AT, FSR_GREEN_AT, FSR_SAFETY);   // gripping: traffic light
        } else {                        // not pressed
            leds_sound_meter(loud);                                       // not gripping: sound meter
        }                               // end of LED choice

        printf("FSR raw %4u  units %4u  |  mic level %4lu\n",           // log both sensors
               fsr_read_raw(), force, (unsigned long)loud);             // values for the line above
        if (!mic_ok) sleep_ms(100);     // without the mic nothing else paces the loop
    }                                   // end of sensor loop
}                                       // end of main
