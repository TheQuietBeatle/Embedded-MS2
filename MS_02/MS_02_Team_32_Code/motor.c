// motor.c - DC motor driver for one L298N channel (IN1, IN2, ENA).
#include "motor.h"                      // our own public interface

#include "pico/stdlib.h"                // GPIO and timer functions
#include "hardware/pwm.h"               // Pico SDK PWM
#include "hardware/clocks.h"            // clock_get_hz()

#define MOTOR_PWM_FREQ_HZ 1000u         // speed PWM frequency: 1 kHz
#define MOTOR_WRAP        999u      // speed % * 10 = PWM level

static uint8_t in1, in2, ena;           // GPIO pins connected to the L298N
static int8_t  current_speed;           // last speed set, -100..+100

// Timed run (reward chute): the motor stops by itself at run_end.
static bool           timed_run;        // true while a timed run is active
static absolute_time_t run_end;         // moment the timed run must stop

void motor_init(uint8_t pin_in1, uint8_t pin_in2, uint8_t pin_ena) {    // see motor.h
    in1 = pin_in1;                      // remember direction pin 1
    in2 = pin_in2;                      // remember direction pin 2
    ena = pin_ena;                      // remember the enable (speed) pin

    gpio_init(in1);                     // IN1 as a plain GPIO...
    gpio_set_dir(in1, GPIO_OUT);        // ...output...
    gpio_put(in1, 0);                   // ...low
    gpio_init(in2);                     // IN2 as a plain GPIO...
    gpio_set_dir(in2, GPIO_OUT);        // ...output...
    gpio_put(in2, 0);                   // ...low

    gpio_set_function(ena, GPIO_FUNC_PWM);              // ENA is driven by the PWM hardware
    uint slice = pwm_gpio_to_slice_num(ena);            // which PWM slice drives ENA
    pwm_config cfg = pwm_get_default_config();          // start from the default settings
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / (MOTOR_PWM_FREQ_HZ * (MOTOR_WRAP + 1u)));  // divide the clock to get 1 kHz
    pwm_config_set_wrap(&cfg, MOTOR_WRAP);              // count 0-999 per period
    pwm_init(slice, &cfg, true);        // apply the settings and start the slice
    pwm_set_gpio_level(ena, 0);       // motor off at start-up

    current_speed = 0;                  // stopped
}                                       // end of motor_init

void motor_set(int8_t speed) {          // run at a speed until told otherwise
    timed_run = false;        // a direct command always overrides a timed run
    if (speed > 100)  speed = 100;      // limit forward speed
    if (speed < -100) speed = -100;     // limit reverse speed
    current_speed = speed;              // remember it for motor_get()

    if (speed > 0) {          // forward
        gpio_put(in1, 1);               // IN1 high...
        gpio_put(in2, 0);               // ...IN2 low = forward
    } else if (speed < 0) {   // reverse
        gpio_put(in1, 0);               // IN1 low...
        gpio_put(in2, 1);               // ...IN2 high = reverse
    } else {                            // speed 0
        motor_coast();                  // let the motor spin down
        return;                         // nothing more to do
    }                                   // end of direction choice
    uint16_t magnitude = (uint16_t)(speed < 0 ? -speed : speed);        // speed without its sign
    pwm_set_gpio_level(ena, magnitude * (MOTOR_WRAP + 1u) / 100u);      // percent -> PWM level (0-1000)
}                                       // end of motor_set

void motor_coast(void) {                // outputs off, motor spins down freely
    timed_run = false;                  // cancel any timed run
    current_speed = 0;                  // speed is now 0
    pwm_set_gpio_level(ena, 0);         // driver disabled
    gpio_put(in1, 0);                   // IN1 low: no direction
    gpio_put(in2, 0);                   // IN2 low: no direction
}                                       // end of motor_coast

void motor_brake(void) {                // stop quickly
    timed_run = false;                  // cancel any timed run
    current_speed = 0;                  // speed is now 0
    gpio_put(in1, 1);                   // both inputs the same...
    gpio_put(in2, 1);                   // ...so both motor terminals are tied together
    pwm_set_gpio_level(ena, MOTOR_WRAP + 1u);   // full enable = short both terminals
}                                       // end of motor_brake

int8_t motor_get(void) {                // last speed set
    return current_speed;               // -100..+100, 0 = stopped
}                                       // end of motor_get

// ---- Timed runs and the reward chute ----------------------------------------

void motor_run_for(int8_t speed, uint32_t ms) {         // run for a limited time, without blocking
    if (speed == 0 || ms == 0) {        // nothing to run
        motor_coast();                  // just stop
        return;                         // no timer needed
    }                                   // end of nothing-to-run check
    motor_set(speed);                       // start turning (also clears any old timed run)
    run_end   = make_timeout_time_ms(ms);   // when to stop
    timed_run = true;                   // motor_update() will now watch the clock
}                                       // end of motor_run_for

void motor_update(void) {               // call regularly from the main loop
    if (timed_run && time_reached(run_end)) {           // timed run active and time is up?
        motor_coast();                      // time is up: let the chute stop
    }                                   // end of time check
}                                       // end of motor_update

bool motor_is_busy(void) {              // is a timed run going on?
    return timed_run;                   // cleared by coast, brake and motor_set
}                                       // end of motor_is_busy

void reward_release(void) {             // success: drop the reward
    motor_run_for((int8_t)(MOTOR_CW_SIGN * REWARD_SPEED), REWARD_RELEASE_MS);  // clockwise for 2 s
}                                       // end of reward_release

void reward_fail(void) {                // failure: error cue
    motor_run_for((int8_t)(-MOTOR_CW_SIGN * REWARD_SPEED), FAIL_CUE_MS);  // counter-clockwise for 5 s
}                                       // end of reward_fail
