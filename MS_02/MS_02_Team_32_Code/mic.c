// mic.c - MAX9814 electret microphone amplifier driver (analog, read through the RP2040 ADC).
#include "mic.h"                        // our own public interface

#include "pico/stdlib.h"                // time_us_64(), tight_loop_contents()
#include "hardware/adc.h"               // Pico SDK analog-to-digital converter

#define MIC_ADC_MID        1551     // 1.25 V offset in ADC counts: 1.25 / 3.3 * 4095
#define MIC_PERIOD_HALF_US 125u     // 1 / 16 kHz = 62.5 us = 125 half-microseconds
#define MIC_SETTLE_SAMPLES 1600u    // ~100 ms to learn the real DC offset at start-up
#define MIC_FLOOR_SAMPLES  8000u    // ~0.5 s to measure the room's background noise
#define MIC_FLOOR_MARGIN   2u       // ignore anything below 2x the background noise
#define MIC_RELEASE_SHIFT  3u       // loudness falls 1/8 of the way per call (slow release)

static uint8_t  mic_input;          // ADC input number (GP26 = 0, GP27 = 1, GP28 = 2)
static uint64_t next_slot;          // time of the next sample, in half-microseconds
static int32_t  dc_q8;              // measured DC offset x 256 (fixed point, keeps the fraction)
static uint32_t noise_floor;        // room background level, measured at start-up
static uint32_t loud_smooth;        // last value returned by mic_read_loudness()

bool mic_init(uint8_t pin) {            // see mic.h
    if (pin < 26u || pin > 28u) {       // not an ADC pin?
        return false;                       // only GP26-28 are connected to the ADC
    }                                   // end of pin check
    mic_input = (uint8_t)(pin - 26u);   // GP26 -> input 0, GP27 -> 1, GP28 -> 2
    adc_init();                         // power up the ADC
    adc_gpio_init(pin);                     // turn off the digital input on this pin

    // Learn the real offset (the module's 1.25 V and the ADC are not exact).
    int64_t sum = 0;                    // total of the settling samples
    next_slot = 0;                      // no sample scheduled yet: start at once
    for (uint32_t i = 0; i < MIC_SETTLE_SAMPLES; i++) {                 // about 100 ms of samples
        sum += mic_read_sample();       // add one sample
    }                                   // end of settling loop
    dc_q8 = (int32_t)((sum * 256) / MIC_SETTLE_SAMPLES);                // average x 256 = the remaining DC offset

    noise_floor = mic_read_level(MIC_FLOOR_SAMPLES);   // what "silence" looks like in this room
    loud_smooth = 0;                    // meter starts at 0
    return true;                        // mic is ready
}                                       // end of mic_init

uint16_t mic_read_raw(void) {           // one raw reading
    adc_select_input(mic_input);            // re-select each time: the FSR shares the same ADC
    return adc_read();                  // convert once: 0-4095
}                                       // end of mic_read_raw

int32_t mic_read_sample(void) {         // one sample at a steady 16 kHz
    uint64_t now = time_us_64() * 2u;   // current time in half-microseconds
    if (next_slot + MIC_PERIOD_HALF_US < now) {         // more than one sample slot late?
        next_slot = now;                    // caller paused for a while: restart the timing
    }                                   // end of late check
    while (time_us_64() * 2u < next_slot) {             // too early for the next sample?
        tight_loop_contents();              // wait for this sample's time slot
    }                                   // end of wait
    next_slot += MIC_PERIOD_HALF_US;    // schedule the following sample
    return (int32_t)mic_read_raw() - MIC_ADC_MID;       // remove the nominal 1.25 V offset
}                                       // end of mic_read_sample

uint32_t mic_read_level(uint16_t n) {   // average loudness over n samples
    if (n == 0) n = 1;                  // avoid dividing by zero
    uint32_t sum = 0;                   // total loudness
    for (uint16_t i = 0; i < n; i++) {  // for each sample...
        int32_t s = mic_read_sample();  // read it (waits for its time slot)
        dc_q8 += (s * 256 - dc_q8) >> 10;     // very slow low-pass = DC offset (follows drift, not sound)
        int32_t ac = s - (dc_q8 >> 8);      // the sound itself
        sum += (uint32_t)(ac < 0 ? -ac : ac);           // add its size, ignoring the sign
    }                                   // end of sample loop
    return sum / n;                     // mean absolute value
}                                       // end of mic_read_level

uint32_t mic_noise_floor(void) {        // room noise level
    return noise_floor;                 // measured once in mic_init()
}                                       // end of mic_noise_floor

uint32_t mic_read_loudness(uint16_t n) {                // loudness above the room noise
    uint32_t level = mic_read_level(n);                 // measure how loud it is now
    uint32_t ignore = noise_floor * MIC_FLOOR_MARGIN;   // everything up to this counts as room noise
    uint32_t above = level > ignore ? level - ignore : 0;   // only what's louder than the room

    if (above > loud_smooth) {          // louder than the meter shows
        loud_smooth = above;                                 // fast attack: react at once
    } else {                            // quieter: let the meter fall
        loud_smooth -= (loud_smooth - above) >> MIC_RELEASE_SHIFT;   // slow release
        if (loud_smooth < 2u) loud_smooth = 0;               // settle exactly on 0
    }                                   // end of attack / release
    return loud_smooth;                 // the smoothed value
}                                       // end of mic_read_loudness
