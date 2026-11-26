// mic.c - MAX9814 electret microphone amplifier driver (analog, read through the RP2040 ADC).
#include "mic.h"

#include "pico/stdlib.h"
#include "hardware/adc.h"

#define MIC_ADC_MID        1551     // 1.25 V offset in ADC counts: 1.25 / 3.3 * 4095
#define MIC_PERIOD_HALF_US 125u     // 1 / 16 kHz = 62.5 us = 125 half-microseconds
#define MIC_SETTLE_SAMPLES 1600u    // ~100 ms to learn the real DC offset at start-up

static uint8_t  mic_input;          // ADC input number (GP26 = 0, GP27 = 1, GP28 = 2)
static uint64_t next_slot;          // time of the next sample, in half-microseconds
static int32_t  dc_q8;              // measured DC offset x 256 (fixed point, keeps the fraction)

bool mic_init(uint8_t pin) {
    if (pin < 26u || pin > 28u) {
        return false;                       // only GP26-28 are connected to the ADC
    }
    mic_input = (uint8_t)(pin - 26u);
    adc_init();
    adc_gpio_init(pin);                     // turn off the digital input on this pin

    // Learn the real offset (the module's 1.25 V and the ADC are not exact).
    int64_t sum = 0;
    next_slot = 0;
    for (uint32_t i = 0; i < MIC_SETTLE_SAMPLES; i++) {
        sum += mic_read_sample();
    }
    dc_q8 = (int32_t)((sum * 256) / MIC_SETTLE_SAMPLES);
    return true;
}

uint16_t mic_read_raw(void) {
    adc_select_input(mic_input);            // re-select each time: the FSR shares the same ADC
    return adc_read();
}

int32_t mic_read_sample(void) {
    uint64_t now = time_us_64() * 2u;
    if (next_slot + MIC_PERIOD_HALF_US < now) {
        next_slot = now;                    // caller paused for a while: restart the timing
    }
    while (time_us_64() * 2u < next_slot) {
        tight_loop_contents();              // wait for this sample's time slot
    }
    next_slot += MIC_PERIOD_HALF_US;
    return (int32_t)mic_read_raw() - MIC_ADC_MID;
}

uint32_t mic_read_level(uint16_t n) {
    if (n == 0) n = 1;
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        int32_t s = mic_read_sample();
        dc_q8 += (s * 256 - dc_q8) >> 10;     // very slow low-pass = DC offset (follows drift, not sound)
        int32_t ac = s - (dc_q8 >> 8);      // the sound itself
        sum += (uint32_t)(ac < 0 ? -ac : ac);
    }
    return sum / n;
}
