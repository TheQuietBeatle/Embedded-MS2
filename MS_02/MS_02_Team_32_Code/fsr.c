// fsr.c - FSR 402 force sensor driver (voltage divider into the RP2040 ADC).
#include "fsr.h"                        // our own public interface

#include "hardware/adc.h"               // Pico SDK analog-to-digital converter

#define FSR_AVG_SAMPLES 16u             // readings averaged for one units / voltage value
#define FSR_DEADBAND    25u   // units below this count as "not pressed"
#define FSR_SMOOTH_SHIFT 2    // new reading weighs 1/4 (bigger = calmer but slower)

static uint8_t fsr_input;   // ADC input number (GP26 = 0, GP27 = 1, GP28 = 2)

void fsr_init(uint8_t pin) {            // see fsr.h
    adc_init();                         // power up the ADC
    adc_gpio_init(pin);              // disable digital functions on the pin
    fsr_input = (uint8_t)(pin - 26u);   // GP26 -> input 0, GP27 -> 1, GP28 -> 2
}                                       // end of fsr_init

uint16_t fsr_read_raw(void) {           // one raw reading
    adc_select_input(fsr_input);     // re-select each time in case another driver used the ADC
    return adc_read();                  // convert once: 0-4095
}                                       // end of fsr_read_raw

uint16_t fsr_read_avg(uint8_t n) {      // average of n readings
    if (n == 0) n = 1;                  // avoid dividing by zero
    uint32_t sum = 0;                   // total of all readings
    for (uint8_t i = 0; i < n; i++) {   // take n readings
        sum += fsr_read_raw();          // add one reading
    }                                   // end of reading loop
    return (uint16_t)(sum / n);         // total / count = average
}                                       // end of fsr_read_avg

uint16_t fsr_read_units(void) {         // force as 0-1000
    return (uint16_t)((uint32_t)fsr_read_avg(FSR_AVG_SAMPLES) * 1000u / FSR_ADC_MAX);  // scale 0-4095 to 0-1000
}                                       // end of fsr_read_units

float fsr_read_voltage(void) {          // force as a voltage
    return fsr_read_avg(FSR_AVG_SAMPLES) * 3.3f / FSR_ADC_MAX;          // scale 0-4095 to 0-3.3 V
}                                       // end of fsr_read_voltage

uint16_t fsr_read_smooth(void) {        // smoothed force, light touches read as 0
    static int32_t smooth_q8 = -1;               // smoothed value x 256 (-1 = no reading yet)
    int32_t u = (int32_t)fsr_read_units() * 256;        // new reading x 256 (keeps the fraction)
    if (smooth_q8 < 0) smooth_q8 = u;            // first call: start at the current reading
    else               smooth_q8 += (u - smooth_q8) >> FSR_SMOOTH_SHIFT;  // later calls: move 1/4 of the way to it
    uint16_t v = (uint16_t)(smooth_q8 / 256);           // back to plain units
    return v < FSR_DEADBAND ? 0 : v;    // below the deadband counts as not pressed
}                                       // end of fsr_read_smooth
