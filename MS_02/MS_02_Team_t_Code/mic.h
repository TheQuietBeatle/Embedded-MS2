// mic.h - INMP441 I2S MEMS microphone driver (read through PIO).
//
// Wiring: VDD -> 3V3, GND -> GND, L/R -> GND (left channel),
//         SCK -> PIN_MIC_SCK, WS -> PIN_MIC_WS (= SCK + 1), SD -> PIN_MIC_SD.
// Samples are 24-bit signed, returned in an int32_t.
#ifndef MIC_H
#define MIC_H

#include <stdbool.h>
#include <stdint.h>

#define MIC_SAMPLE_RATE_HZ 16000u

// Start the PIO I2S receiver. pin_ws must equal pin_sck + 1.
// Returns false if no PIO state machine / program space is free or the pins are wrong.
bool     mic_init(uint8_t pin_sck, uint8_t pin_ws, uint8_t pin_sd);

// Blocking read of one 24-bit sample (range about -8388608..8388607).
int32_t  mic_read_sample(void);

// Read n samples and return the average loudness (mean absolute value,
// DC offset removed). Silence ~ a few hundred, speech near the mic ~ 10k-100k+.
uint32_t mic_read_level(uint16_t n);

#endif // MIC_H
