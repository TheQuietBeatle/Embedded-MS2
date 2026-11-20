// fsr.h - FSR 402 force sensor driver (voltage divider into the RP2040 ADC).
//
// Wiring: 3V3 -> FSR -> ADC pin, and a 10 k resistor from the ADC pin to GND.
// More force = lower FSR resistance = higher voltage = higher reading.
#ifndef FSR_H
#define FSR_H

#include <stdint.h>

#define FSR_ADC_MAX 4095u   // 12-bit ADC

// Set up the ADC on pin (must be GP26, GP27 or GP28).
void     fsr_init(uint8_t pin);

// One raw reading, 0-4095.
uint16_t fsr_read_raw(void);

// Average of n raw readings (smooths noise), 0-4095.
uint16_t fsr_read_avg(uint8_t n);

// Average reading scaled to 0-1000 "force units" (no load ~0, hard press ~1000).
uint16_t fsr_read_units(void);

// Voltage at the ADC pin in volts (0-3.3 V).
float    fsr_read_voltage(void);

#endif // FSR_H
