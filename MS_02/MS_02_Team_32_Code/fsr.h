// fsr.h - FSR 402 force sensor driver (voltage divider into the RP2040 ADC).
//
// Wiring: 3V3 -> FSR -> ADC pin, and a 10 k resistor from the ADC pin to GND.
// More force = lower FSR resistance = higher voltage = higher reading.
#ifndef FSR_H                           // include guard: skip this file if it was already included
#define FSR_H                           // mark this file as included

#include <stdint.h>                     // uint8_t, uint16_t

#define FSR_ADC_MAX 4095u   // 12-bit ADC

// Set up the ADC on pin (must be GP26, GP27 or GP28).
void     fsr_init(uint8_t pin);         // call once before any reading

// One raw reading, 0-4095.
uint16_t fsr_read_raw(void);            // single ADC conversion

// Average of n raw readings (smooths noise), 0-4095.
uint16_t fsr_read_avg(uint8_t n);       // n = 0 is treated as 1

// Average reading scaled to 0-1000 "force units" (no load ~0, hard press ~1000).
uint16_t fsr_read_units(void);          // uses 16 readings per call

// Voltage at the ADC pin in volts (0-3.3 V).
float    fsr_read_voltage(void);        // also uses 16 readings per call

// Calmer force value for the logic and the display, 0-1000 units:
//  - smoothed over time (each call moves 1/4 of the way to the new reading,
//    so one jumpy reading cannot flip the state)
//  - anything below FSR_DEADBAND (a light brush) reads as 0
// Call it at a steady rate (e.g. every 50 ms).
uint16_t fsr_read_smooth(void);         // the one main.c and web_control.c use for the LEDs

#endif // FSR_H
