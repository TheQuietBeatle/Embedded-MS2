// mic.c - INMP441 I2S MEMS microphone driver (read through PIO).
#include "mic.h"

#include "hardware/pio.h"
#include "i2s_in.pio.h"

#define MIC_SETTLE_SAMPLES 1600u   // ~100 ms: the INMP441 needs time to start after the clock begins

static PIO  mic_pio = pio0;
static uint mic_sm;

bool mic_init(uint8_t pin_sck, uint8_t pin_ws, uint8_t pin_sd) {
    if (pin_ws != pin_sck + 1u) {
        return false;                       // the PIO drives SCK and WS as two consecutive pins
    }
    if (!pio_can_add_program(mic_pio, &i2s_in_program)) {
        return false;
    }
    int sm = pio_claim_unused_sm(mic_pio, false);
    if (sm < 0) {
        return false;
    }
    mic_sm = (uint)sm;
    uint offset = pio_add_program(mic_pio, &i2s_in_program);
    i2s_in_program_init(mic_pio, mic_sm, offset, pin_sck, pin_sd, MIC_SAMPLE_RATE_HZ);

    for (uint32_t i = 0; i < MIC_SETTLE_SAMPLES; i++) {
        (void)mic_read_sample();            // throw away start-up samples
    }
    return true;
}

int32_t mic_read_sample(void) {
    uint32_t raw = pio_sm_get_blocking(mic_pio, mic_sm);
    return ((int32_t)raw) >> 8;             // 24-bit data is left-aligned in 32 bits
}

uint32_t mic_read_level(uint16_t n) {
    if (n == 0) n = 1;
    static int32_t dc;                      // running DC offset estimate
    uint64_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        int32_t s = mic_read_sample();
        dc += (s - dc) / 64;                // slow low-pass = DC level
        int32_t ac = s - dc;
        sum += (uint32_t)(ac < 0 ? -ac : ac);
    }
    return (uint32_t)(sum / n);
}
