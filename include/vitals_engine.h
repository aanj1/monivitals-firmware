#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "max30102.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t wave[32];
    size_t wave_count;
    uint16_t hr_bpm;      /* 0 if not yet valid */
    uint8_t spo2_pct;     /* 0 if not yet valid */
    uint8_t pi_x10;       /* perfusion index in 0.1% units */
    uint16_t r_x100;      /* ratio-of-ratios * 100 (SpO2 calibration input; 0 = none) */
    bool finger_on;
    bool valid;           /* true only when HR+SpO2 from real PPG */
    /* Debug/telemetry for tuning on real hardware (logged every 3 s). */
    uint16_t env;         /* AC envelope magnitude (peak-detect threshold basis) */
    uint8_t beats;        /* beat-intervals buffered (0..MEDIAN_N) */
} vitals_frame_t;

void vitals_engine_reset(void);

/** Clear beat/HR state for a new stream without wiping AGC-warmed DC.
 * NOT task-safe against vitals_engine_process()/peek/consume — must only be
 * called from the task that runs them (stream_task). BLE callbacks set a
 * pending-reset flag instead (see main.c). */
void vitals_engine_soft_reset(void);

/** Upstream sample gap (e.g. sensor FIFO overflow): invalidate state that
 * spans the gap — beat intervals, peak timing, ratio-of-ratios window. */
void vitals_engine_note_gap(void);

void vitals_engine_process(const max30102_sample_t *in, int n, vitals_frame_t *out);

/** Copy up to `cap` of the oldest buffered PPG samples WITHOUT removing them
 * (full 100 sps stream for the app's RR/HRV DSP). Returns the count written.
 * Call vitals_engine_consume_wave() only after the BLE notify succeeds. */
size_t vitals_engine_peek_wave(int16_t *out, size_t cap);

/** Remove `n` oldest buffered PPG samples (those a notify actually queued). */
void vitals_engine_consume_wave(size_t n);

/** True (and clears the latch) if the wave ring overwrote unread samples
 * since the last call — i.e. the BLE consumer fell > 2.5 s behind. */
bool vitals_engine_take_wave_overflow(void);

#ifdef __cplusplus
}
#endif
