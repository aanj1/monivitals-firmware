/**
 * MoniVitals — ESP32-C3 Super Mini + MAX30102 (real-use).
 */

#include "board.h"
#include "max30102.h"
#include "ble_monivitals.h"
#include "vitals_engine.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdatomic.h>

static const char *TAG = "main";

#define VITALS_NOTIFY_MS   2000
/* 200 ms * 100 sps = 20 samples/notify; drain covers up to 40 to absorb a
 * missed tick without dropping the contiguous stream the app's DSP relies on. */
#define PPG_NOTIFY_MS       200
#define PPG_MAX_PER_NOTIFY   40
/* Max notifies per drain tick. The wave ring holds <= 256 samples; the smallest
 * notify (MTU 23) carries ~9, so 32 notifies always empties the ring in a tick
 * (prevents the ~45 sps < 100 sps overflow-every-5s discontinuity). Bounds the
 * loop so it can't spin if a notify keeps returning a positive short count. */
#define PPG_DRAIN_MAX_NOTIFIES 32

/* Set from the NimBLE host task; consumed by stream_task. The soft reset must
 * NOT run here — it would race stream_task's process()/wave-drain on the same
 * static engine state (a reset landing mid-drain underflows the ring counter).
 * stream_task applies it at the top of its loop instead. */
static atomic_bool s_pending_soft_reset;

static void on_stream_toggle(bool enabled)
{
    if (enabled) {
        atomic_store(&s_pending_soft_reset, true);
        ESP_LOGI(TAG, "stream ON — soft reset queued");
    } else {
        ESP_LOGI(TAG, "stream OFF");
    }
}

static void stream_task(void *arg)
{
    (void)arg;
    max30102_sample_t fifo[16];
    vitals_frame_t frame;
    TickType_t last_vitals = 0;
    TickType_t last_ppg = 0;
    TickType_t last_log = 0;
    TickType_t now_reinit = 0; /* last sensor re-init attempt (0 = never) */
    bool was_finger = false;

    while (1) {
        /* Apply a BLE-requested soft reset here, serialized with process() /
         * wave peek+consume, so it can never land mid-drain (see finding on
         * s_wave_avail underflow). */
        if (atomic_exchange(&s_pending_soft_reset, false)) {
            vitals_engine_soft_reset();
            /* Ring was cleared — tell the phone the stream is not contiguous. */
            ble_monivitals_ppg_mark_discontinuity();
            ESP_LOGI(TAG, "soft reset applied");
        }

        /* Sensor not ready (init failed at boot, or a transient I2C glitch) —
         * periodically retry init so a boot glitch self-heals instead of the
         * device advertising + accepting subscriptions while streaming nothing
         * forever. On success, re-run the engine reset for a clean start. */
        if (!max30102_is_ready()) {
            if ((now_reinit == 0) ||
                (xTaskGetTickCount() - now_reinit) >= pdMS_TO_TICKS(2000)) {
                now_reinit = xTaskGetTickCount();
                int irc = max30102_init();
                if (irc == 0) {
                    ESP_LOGI(TAG, "MAX30102 re-init OK");
                    vitals_engine_reset();
                } else {
                    ESP_LOGW(TAG, "MAX30102 re-init failed (rc=%d)", irc);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Always drain FIFO so idle/advertising never saturates the sensor path. */
        int n = max30102_read_fifo(fifo, 16);
        if (max30102_take_fifo_overflow()) {
            /* Sensor FIFO overflowed — samples lost. Invalidate gap-spanning
             * beat intervals and skip a PPG seq value so the phone discards
             * the RR/HRV interval across the gap. */
            vitals_engine_note_gap();
            ble_monivitals_ppg_mark_discontinuity();
        }
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        for (int i = 0; i < n; i++) {
            max30102_agc_feed(fifo[i].red, fifo[i].ir, now_ms);
        }

        bool connected = ble_monivitals_is_connected();
        bool streaming = ble_monivitals_stream_requested();

        if (!connected) {
            was_finger = false;
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        vitals_engine_process(fifo, n, &frame);

        if (!streaming) {
            was_finger = false;
            vTaskDelay(pdMS_TO_TICKS(12));
            continue;
        }

        TickType_t now = xTaskGetTickCount();

        /* Contact edge — react within one loop (~12 ms) instead of waiting for the
         * next 2 s vitals tick. Firmware finger_on is already debounced (~0.8 s), so
         * an immediate Quality push lets the app show/clear contact promptly. */
        if (frame.finger_on != was_finger) {
            ble_monivitals_set_perfusion_index_x10(frame.pi_x10);
            ble_monivitals_set_contact(frame.finger_on);
            (void)ble_monivitals_notify_quality();
            if (!frame.finger_on) {
                /* Finger lifted — clear sticky HR immediately. */
                ble_monivitals_set_heart_rate_bpm(0);
                (void)ble_monivitals_notify_hr();
            }
            was_finger = frame.finger_on;
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        if ((now - last_ppg) >= pdMS_TO_TICKS(PPG_NOTIFY_MS)) {
            last_ppg = now;
            /* Consumer fell behind and the ring overwrote unread samples —
             * mark the discontinuity before sending what remains. */
            if (vitals_engine_take_wave_overflow()) {
                ble_monivitals_ppg_mark_discontinuity();
            }
            /* Drain the whole ring this tick. At MTU 23 a single notify carries
             * only ~9 samples (MTU-4)/2, so one notify/tick = ~45 sps < the
             * 100 sps produced → the ring overflowed every ~5 s → perpetual
             * discontinuity → the phone's HRV never accumulated. Loop
             * peek→notify→consume until the ring is empty, a notify carries
             * fewer than we offered (MTU cap reached but nothing left / partial),
             * or a notify fails/queues nothing (leave the rest buffered for the
             * next tick — a failed notify never destroys buffered waveform).
             * Bounded by ring size so it can't spin. */
            int16_t wave[PPG_MAX_PER_NOTIFY];
            /* Guard bound: the ring holds <= 256 samples and each successful
             * notify consumes >= 1, so PPG_DRAIN_MAX_NOTIFIES iterations always
             * empties it even at the MTU-23 minimum (~9 samples/notify). */
            for (int guard = 0; guard < PPG_DRAIN_MAX_NOTIFIES; guard++) {
                size_t wn = vitals_engine_peek_wave(wave, PPG_MAX_PER_NOTIFY);
                if (wn == 0) break;                 /* ring empty */
                int sent = ble_monivitals_notify_ppg(wave, wn);
                if (sent <= 0) break;               /* not notifying / error — keep buffered */
                vitals_engine_consume_wave((size_t)sent);
                if ((size_t)sent < wn) break;       /* MTU-capped this notify; rest next tick */
            }
        }

        if ((now - last_vitals) >= pdMS_TO_TICKS(VITALS_NOTIFY_MS)) {
            last_vitals = now;

            ble_monivitals_set_perfusion_index_x10(frame.pi_x10);
            /* Contact = optical finger detect — not "algorithm fully settled". */
            ble_monivitals_set_contact(frame.finger_on);
            (void)ble_monivitals_notify_quality();
            /* Stagger notifies — NimBLE drops back-to-back custom notifies (EBUSY). */
            vTaskDelay(pdMS_TO_TICKS(30));

            if (frame.finger_on && frame.hr_bpm > 0) {
                ble_monivitals_set_heart_rate_bpm(frame.hr_bpm);
                if (ble_monivitals_notify_hr() != 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    (void)ble_monivitals_notify_hr();
                }
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            if (frame.finger_on && frame.spo2_pct >= 80) {
                ble_monivitals_set_spo2_pct(frame.spo2_pct);
                if (ble_monivitals_notify_plx() != 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                    (void)ble_monivitals_notify_plx();
                }
            }
            /* Finger-off wipe is handled by the contact-edge block above (fires
             * within one loop, not just on the 2 s boundary). */
        }

        if ((now - last_log) >= pdMS_TO_TICKS(3000)) {
            last_log = now;
            /* R (r_x100) is the SpO2 calibration input: log it next to a reference
             * oximeter reading to refit s_lut[] in vitals_engine.c. */
            ESP_LOGI(TAG, "finger=%d valid=%d hr=%u spo2=%u pi=%u R=%u env=%u beats=%u",
                     frame.finger_on, frame.valid,
                     (unsigned)frame.hr_bpm, (unsigned)frame.spo2_pct,
                     (unsigned)frame.pi_x10, (unsigned)frame.r_x100,
                     (unsigned)frame.env, (unsigned)frame.beats);
        }

        vTaskDelay(pdMS_TO_TICKS(12));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "MoniVitals ESP32-C3 Super Mini + MAX30102");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    vitals_engine_reset();

    int rc = max30102_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "MAX30102 init failed (rc=%d) SDA=%d SCL=%d",
                 rc, SB_PIN_I2C_SDA, SB_PIN_I2C_SCL);
    } else {
        ESP_LOGI(TAG, "MAX30102 online");
    }

    ble_monivitals_set_stream_callback(on_stream_toggle);

    if (ble_monivitals_init() != 0) {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    /* Always create stream_task, even if the first max30102_init() failed: the
     * task periodically retries init so a transient boot I2C glitch self-heals
     * instead of leaving a device that advertises + accepts subscriptions but
     * streams nothing forever. (rc is only informational here now.) */
    (void)rc;
    if (xTaskCreate(stream_task, "stream", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "stream task create failed — no vitals streaming");
    }
    ESP_LOGI(TAG, "ready");
}
