#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_I2C_ADDR  0x57
#define MAX30102_PART_ID   0x15

typedef struct {
    uint32_t red; /* 18-bit */
    uint32_t ir;  /* 18-bit */
} max30102_sample_t;

/** Init I2C + chip in SpO2 mode @ 100 sps. Returns 0 on success. */
int max30102_init(void);

bool max30102_is_ready(void);

/** Drain FIFO (up to cap samples). Returns count written. */
int max30102_read_fifo(max30102_sample_t *buf, size_t cap);

/** True (and clears the latch) if the sensor FIFO overflowed — i.e. samples
 * were lost — since the last call. Caller should reset beat-interval state
 * and signal the stream gap. */
bool max30102_take_fifo_overflow(void);

/** Set LED pulse amplitudes (register units, ~0.2 mA/step). */
bool max30102_set_led_current(uint8_t red_pa, uint8_t ir_pa);

/**
 * Auto LED gain: keep IR DC in ~25–75% of 18-bit FS so firm finger
 * contact does not saturate (which kills AC / vitals).
 * Call with latest sample + ms tick; adjusts at most every 500 ms.
 */
void max30102_agc_feed(uint32_t red, uint32_t ir, uint32_t tick_ms);

#ifdef __cplusplus
}
#endif
