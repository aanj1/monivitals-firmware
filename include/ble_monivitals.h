#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Called when host enables/disables the measurement stream (COMMAND 0x02). */
typedef void (*ble_monivitals_stream_cb_t)(bool enabled);

int ble_monivitals_init(void);

void ble_monivitals_set_stream_callback(ble_monivitals_stream_cb_t cb);

bool ble_monivitals_is_connected(void);

/** Host wrote COMMAND [0x02, 0x01] after enabling PPG notify. */
bool ble_monivitals_stream_requested(void);

/** Idempotent stream enable/disable (no soft-reset if already in that state). */
void ble_monivitals_request_stream(bool on);

void ble_monivitals_set_heart_rate_bpm(uint16_t bpm);
void ble_monivitals_set_spo2_pct(uint8_t spo2);
void ble_monivitals_set_perfusion_index_x10(uint8_t pi_x10);
void ble_monivitals_set_contact(bool contact);

int ble_monivitals_notify_quality(void);

/**
 * Notify PPG waveform. Payload: byte[0] = uint8 sequence number (+1 per
 * successfully queued notification, wraps 255→0; a skip of 2 signals a sample
 * gap), bytes[1..] = little-endian int16 samples. Length 1 + 2n (always odd).
 * Sample count is capped to the negotiated ATT MTU.
 * Returns samples actually queued (>0), 0 if not notifying, negative NimBLE
 * error on failure — the caller must only consume what was queued.
 */
int ble_monivitals_notify_ppg(const int16_t *samples, size_t count);

/** Signal lost PPG samples: skips one sequence value so the phone sees a
 * seq jump of 2 and discards the interval spanning the gap. */
void ble_monivitals_ppg_mark_discontinuity(void);

int ble_monivitals_notify_hr(void);
int ble_monivitals_notify_plx(void);

#ifdef __cplusplus
}
#endif
