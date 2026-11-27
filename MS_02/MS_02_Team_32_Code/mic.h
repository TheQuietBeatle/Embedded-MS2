// mic.h - MAX9814 electret microphone amplifier driver (analog, read through the RP2040 ADC).
//
// Wiring: Vdd -> 3V3, GND -> GND, Out -> PIN_MIC_ADC (GP26/27/28),
//         Gain -> left floating (60 dB) or GND (50 dB) or Vdd (40 dB),
//         AR   -> left floating (attack/release ratio 1:4000).
// The module has automatic gain control (AGC): the output never goes over
// about 2 V peak-to-peak, centred on a 1.25 V DC offset (0.25 V .. 2.25 V),
// so it fits the Pico's 0-3.3 V ADC without any extra parts.
//
// The ADC is 12-bit, so one sample is 0..4095. mic_read_sample() removes the
// nominal 1.25 V offset and returns about -1241..+1241 (1 count ~ 0.8 mV).
#ifndef MIC_H
#define MIC_H

#include <stdbool.h>
#include <stdint.h>

#define MIC_SAMPLE_RATE_HZ 16000u   // samples per second (one sample every 62.5 us)

// Set up the ADC on pin (must be GP26, GP27 or GP28).
// Returns false if the pin has no ADC.
bool     mic_init(uint8_t pin);

// Raw ADC value 0..4095 (read immediately, not paced).
uint16_t mic_read_raw(void);

// Blocking: waits for the next 16 kHz sample slot, then returns one sample
// with the 1.25 V offset removed (about -1241..+1241).
int32_t  mic_read_sample(void);

// Read n samples and return the average loudness (mean absolute value,
// exact DC offset removed). Quiet room ~ 5-30, speech near the mic ~ 200-800.
uint32_t mic_read_level(uint16_t n);

// Room noise level measured by mic_init() (keep quiet for ~0.5 s at start-up).
uint32_t mic_noise_floor(void);

// Calmer loudness for the logic and the display:
//  - the room's background noise (noise floor x MIC_FLOOR_MARGIN) is subtracted,
//    so a quiet room reads 0
//  - rises at once on a sound, falls back slowly (no flicker between readings)
uint32_t mic_read_loudness(uint16_t n);

#endif // MIC_H
