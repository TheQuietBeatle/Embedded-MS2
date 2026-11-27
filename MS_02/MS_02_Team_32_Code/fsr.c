// fsr.c - FSR 402 force sensor driver (voltage divider into the RP2040 ADC).
#include "fsr.h"

#include "hardware/adc.h"

#define FSR_AVG_SAMPLES 16u
#define FSR_DEADBAND    25u   // units below this count as "not pressed"
#define FSR_SMOOTH_SHIFT 2    // new reading weighs 1/4 (bigger = calmer but slower)

static uint8_t fsr_input;   // ADC input number (GP26 = 0, GP27 = 1, GP28 = 2)

void fsr_init(uint8_t pin) {
    adc_init();
    adc_gpio_init(pin);              // disable digital functions on the pin
    fsr_input = (uint8_t)(pin - 26u);
}

uint16_t fsr_read_raw(void) {
    adc_select_input(fsr_input);     // re-select each time in case another driver used the ADC
    return adc_read();
}

uint16_t fsr_read_avg(uint8_t n) {
    if (n == 0) n = 1;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += fsr_read_raw();
    }
    return (uint16_t)(sum / n);
}

uint16_t fsr_read_units(void) {
    return (uint16_t)((uint32_t)fsr_read_avg(FSR_AVG_SAMPLES) * 1000u / FSR_ADC_MAX);
}

float fsr_read_voltage(void) {
    return fsr_read_avg(FSR_AVG_SAMPLES) * 3.3f / FSR_ADC_MAX;
}

uint16_t fsr_read_smooth(void) {
    static int32_t smooth_q8 = -1;               // smoothed value x 256 (-1 = no reading yet)
    int32_t u = (int32_t)fsr_read_units() * 256;
    if (smooth_q8 < 0) smooth_q8 = u;            // first call: start at the current reading
    else               smooth_q8 += (u - smooth_q8) >> FSR_SMOOTH_SHIFT;
    uint16_t v = (uint16_t)(smooth_q8 / 256);
    return v < FSR_DEADBAND ? 0 : v;
}
