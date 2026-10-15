/**
 * Lean MoniVitals GATT for iOS SamaApp / web-bt.
 * Plaintext characteristics (no ENC) so iOS can subscribe without pairing.
 *
 * SECURITY (intentional, demo-only): the link uses Just-Works with bonding,
 * MITM protection and LE Secure Connections all disabled (sm_bonding=0,
 * sm_mitm=0, sm_sc=0 in ble_monivitals_init), and no characteristic requires
 * encryption. Everything is plaintext over the air so iOS / web-bluetooth can
 * subscribe without a pairing prompt. A real product carrying user health
 * data MUST switch to LE Secure Connections with encrypted (and ideally
 * authenticated) characteristic permissions before shipping.
 */

#include "ble_monivitals.h"
#include "board.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "ble_mv";

/*
 * Custom 128-bit base must match the shipping iOS TestFlight GATT map:
 *   425400XX-5441-4542-4F58-3B9A78563412  (legacy "Samabeat" / BEAT encoding)
 * Device name remains MoniVitals-XXXX. Newer app builds also accept the
 * MoniVitals (5348…) base; keep this until TestFlight is updated.
 */
#define MONIVITALS_UUID128(id) \
    BLE_UUID128_INIT(0x12, 0x34, 0x56, 0x78, 0x9A, 0x3B, 0x58, 0x4F, \
                     0x42, 0x45, 0x41, 0x54, (id), 0x00, 0x54, 0x42)

#define SB_CHR_RN   (BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY)
#define SB_CHR_W    (BLE_GATT_CHR_F_WRITE)

static const ble_uuid128_t s_svc_uuid = MONIVITALS_UUID128(0x01);
static const ble_uuid128_t s_quality_uuid = MONIVITALS_UUID128(0x02);
static const ble_uuid128_t s_ppg_uuid = MONIVITALS_UUID128(0x06);
static const ble_uuid128_t s_cmd_uuid = MONIVITALS_UUID128(0x07);

static uint16_t s_hr_handle;
static uint16_t s_plx_handle;
static uint16_t s_quality_handle;
static uint16_t s_ppg_handle;

/* Shared between the NimBLE host task (GAP/GATT callbacks) and stream_task
 * (main.c) — all atomics so cross-task reads/writes are well-defined. */
static _Atomic uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static atomic_bool s_connected;
static atomic_bool s_stream_on;
static atomic_bool s_notify_quality;
static atomic_bool s_notify_ppg;
static atomic_bool s_notify_hr;
static atomic_bool s_notify_plx;

static _Atomic uint16_t s_hr_bpm;
static _Atomic uint8_t s_spo2;
static _Atomic uint8_t s_pi_x10;
static atomic_bool s_contact;
static ble_monivitals_stream_cb_t s_stream_cb;

/* PPG waveform sequence number (payload byte 0). Incremented by 1 per
 * successfully queued notification, wraps 255→0 (uint8). An extra bump via
 * ble_monivitals_ppg_mark_discontinuity() makes the receiver see a skip of 2,
 * signalling lost samples. */
static _Atomic uint8_t s_ppg_seq;

static char s_name[20] = "MoniVitals";

static uint16_t encode_sfloat(int16_t mantissa, int8_t exponent)
{
    uint16_t exp_bits = (uint16_t)((uint8_t)exponent & 0x0F) << 12;
    uint16_t man_bits = (uint16_t)(mantissa & 0x0FFF);
    return (uint16_t)(exp_bits | man_bits);
}

static void start_advertise(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_connected = true;
            s_stream_on = false;
            ESP_LOGI(TAG, "connected handle=%u", s_conn_handle);
        } else {
            ESP_LOGW(TAG, "connect failed status=%d", event->connect.status);
            start_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected reason=%d", event->disconnect.reason);
        s_connected = false;
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_stream_on = false;
        s_notify_quality = false;
        s_notify_ppg = false;
        s_notify_hr = false;
        s_notify_plx = false;
        start_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_quality_handle) {
            s_notify_quality = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "Quality notify=%d", s_notify_quality);
            if (s_notify_quality) ble_monivitals_request_stream(true);
        } else if (event->subscribe.attr_handle == s_ppg_handle) {
            s_notify_ppg = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "PPG notify=%d", s_notify_ppg);
            if (s_notify_ppg) ble_monivitals_request_stream(true);
        } else if (event->subscribe.attr_handle == s_hr_handle) {
            s_notify_hr = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "HR notify=%d", s_notify_hr);
            if (s_notify_hr) {
                ble_monivitals_request_stream(true);
                if (s_hr_bpm > 0) (void)ble_monivitals_notify_hr();
            }
        } else if (event->subscribe.attr_handle == s_plx_handle) {
            s_notify_plx = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "PLX notify=%d", s_notify_plx);
            if (s_notify_plx) {
                ble_monivitals_request_stream(true);
                if (s_spo2 >= 80) (void)ble_monivitals_notify_plx();
            }
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%u", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

static void start_advertise(void)
{
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields rsp = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.uuids128 = &s_svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv fields %d", rc);
        return;
    }

    static ble_uuid16_t uuids16[] = {
        BLE_UUID16_INIT(0x180D),
        BLE_UUID16_INIT(0x1822),
    };
    rsp.name = (uint8_t *)s_name;
    rsp.name_len = (uint8_t)strlen(s_name);
    rsp.name_is_complete = 1;
    rsp.uuids16 = uuids16;
    rsp.num_uuids16 = 2;
    rsp.uuids16_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv.itvl_min = BLE_GAP_ADV_ITVL_MS(100);
    adv.itvl_max = BLE_GAP_ADV_ITVL_MS(150);

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start %d", rc);
    } else {
        ESP_LOGI(TAG, "advertising as %s", s_name);
    }
}

static int hr_access(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint8_t buf[3];
        /* Single atomic load — split the bytes from one snapshot so the two
         * bytes can't come from different concurrent setter values (benign at
         * bpm<=220 today, but correct regardless of range). */
        uint16_t bpm = s_hr_bpm;
        /* bit0=16-bit, bit1=contact detected, bit2=contact supported */
        /* Match the notify path: contact-detected requires a live bpm, so a GATT
         * read can't report detected=true with bpm=0 while a concurrent notify
         * reports otherwise. */
        buf[0] = (uint8_t)(0x05 | (s_contact && bpm > 0 ? 0x02 : 0x00));
        buf[1] = (uint8_t)(bpm & 0xFF);
        buf[2] = (uint8_t)(bpm >> 8);
        if (os_mbuf_append(ctxt->om, buf, 3) != 0) {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    return 0;
}

/* SFLOAT pulse rate for PLX: real HR when known, 0x07FF (SFLOAT NaN) when not. */
static uint16_t hr_sfloat(void)
{
    uint16_t bpm = s_hr_bpm;
    return (bpm > 0) ? encode_sfloat((int16_t)bpm, 0) : 0x07FF;
}

static int plx_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint16_t sf = encode_sfloat((int16_t)(s_spo2 * 10), -1);
        uint16_t pr = hr_sfloat();
        uint8_t buf[5] = {
            0x00,
            (uint8_t)(sf & 0xFF),
            (uint8_t)(sf >> 8),
            (uint8_t)(pr & 0xFF),
            (uint8_t)(pr >> 8),
        };
        if (os_mbuf_append(ctxt->om, buf, sizeof(buf)) != 0) {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    return 0;
}

static int quality_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        /* 2 bytes: [0]=PI (0.1% units), [1]=finger contact (authoritative). */
        uint8_t buf[2] = { s_pi_x10, (uint8_t)(s_contact ? 1 : 0) };
        if (os_mbuf_append(ctxt->om, buf, sizeof(buf)) != 0) {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    return 0;
}

static int ppg_access(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg; (void)ctxt;
    return 0;
}

static int command_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1 || len > 16) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    uint8_t buf[16];
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (buf[0] == 0x02 && len >= 2) {
        ble_monivitals_request_stream(buf[1] != 0);
    }
    return 0;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180D),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2A37),
                .access_cb = hr_access,
                .val_handle = &s_hr_handle,
                .flags = SB_CHR_RN,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1822),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = BLE_UUID16_DECLARE(0x2A5F),
                .access_cb = plx_access,
                .val_handle = &s_plx_handle,
                .flags = SB_CHR_RN,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_quality_uuid.u,
                .access_cb = quality_access,
                .val_handle = &s_quality_handle,
                .flags = SB_CHR_RN,
            },
            {
                .uuid = &s_ppg_uuid.u,
                .access_cb = ppg_access,
                .val_handle = &s_ppg_handle,
                .flags = SB_CHR_RN,
            },
            {
                .uuid = &s_cmd_uuid.u,
                .access_cb = command_access,
                .flags = SB_CHR_W,
            },
            {0},
        },
    },
    {0},
};

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr %d", rc);
        return;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_name, sizeof(s_name), "%s-%02X%02X",
             SB_DEVICE_NAME_PREFIX, mac[1], mac[0]);
    ble_svc_gap_device_name_set(s_name);

    start_advertise();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "nimble reset reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_monivitals_set_stream_callback(ble_monivitals_stream_cb_t cb)
{
    s_stream_cb = cb;
}

void ble_monivitals_request_stream(bool on)
{
    if (s_stream_on == on) {
        return; /* Idempotent — prevents soft-reset on COMMAND after PPG auto-ON. */
    }
    s_stream_on = on;
    ESP_LOGI(TAG, "stream %s", on ? "ON" : "OFF");
    if (s_stream_cb) {
        s_stream_cb(on);
    }
}

int ble_monivitals_init(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init %s", esp_err_to_name(err));
        return -1;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) return rc;
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) return rc;

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "NimBLE MoniVitals GATT ready");
    return 0;
}

bool ble_monivitals_is_connected(void)
{
    return s_connected;
}

bool ble_monivitals_stream_requested(void)
{
    return s_connected && s_stream_on;
}

void ble_monivitals_set_heart_rate_bpm(uint16_t bpm)
{
    /* Pass through measured BPM only — no fabricated floor. */
    if (bpm > 220) bpm = 220;
    s_hr_bpm = bpm;
}

void ble_monivitals_set_spo2_pct(uint8_t spo2)
{
    /* Pass through measured SpO2 — do not clamp up to 80/98. */
    if (spo2 > 100) spo2 = 100;
    s_spo2 = spo2;
}

void ble_monivitals_set_perfusion_index_x10(uint8_t pi_x10)
{
    if (pi_x10 > 200) pi_x10 = 200;
    s_pi_x10 = pi_x10;
}

void ble_monivitals_set_contact(bool contact)
{
    s_contact = contact;
}

int ble_monivitals_notify_quality(void)
{
    if (!s_connected || !s_notify_quality) return 0;
    /* 2 bytes: [0]=PI (0.1% units), [1]=finger contact.
     * Contact byte is firmware's debounced s_finger decision — authoritative for
     * the app, so firm-contact small-AC (low PI) does not read as "no contact". */
    uint8_t buf[2] = { s_pi_x10, (uint8_t)(s_contact ? 1 : 0) };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!om) return BLE_HS_ENOMEM;
    return ble_gatts_notify_custom(s_conn_handle, s_quality_handle, om);
}

void ble_monivitals_ppg_mark_discontinuity(void)
{
    /* Skip one sequence value: the next PPG notify carries seq = last+2, so the
     * phone knows samples were lost and discards the RR/HRV interval spanning
     * the gap. Atomic RMW; uint8 wraps 255→0 naturally. */
    s_ppg_seq++;
}

/* PPG waveform notify — payload format (must match the iOS parser exactly):
 *   byte[0]   = uint8 sequence number; +1 per successfully queued notification,
 *               wraps 255→0. A skip of 2 marks a sample-gap discontinuity.
 *   byte[1..] = little-endian int16 samples (unchanged from legacy).
 * Total length = 1 + 2n — always ODD, which is how the iOS parser tells this
 * format apart from the legacy even-length samples-only payload.
 *
 * Returns the number of samples actually queued (> 0), 0 when not notifying,
 * or a negative NimBLE error. The caller must only consume from its ring the
 * samples this call reports as queued — on failure they stay buffered. */
#define PPG_NOTIFY_MAX_SAMPLES 50

int ble_monivitals_notify_ppg(const int16_t *samples, size_t count)
{
    if (!s_connected || !s_notify_ppg || !samples || count == 0) return 0;
    if (count > PPG_NOTIFY_MAX_SAMPLES) count = PPG_NOTIFY_MAX_SAMPLES;

    /* Cap to the negotiated ATT MTU: notify payload ≤ MTU − 3 (ATT header),
     * minus 1 for the sequence byte. Stock-Android centrals at MTU 23 would
     * otherwise get silently truncated payloads. */
    uint16_t mtu = ble_att_mtu(s_conn_handle);
    if (mtu < BLE_ATT_MTU_DFLT) mtu = BLE_ATT_MTU_DFLT; /* 0 on bad handle */
    size_t max_samples = (size_t)(mtu - 3 - 1) / 2;
    if (count > max_samples) count = max_samples;

    uint8_t payload[1 + PPG_NOTIFY_MAX_SAMPLES * sizeof(int16_t)];
    payload[0] = s_ppg_seq;
    memcpy(&payload[1], samples, count * sizeof(int16_t)); /* LE host order */

    struct os_mbuf *om = ble_hs_mbuf_from_flat(payload, 1 + count * sizeof(int16_t));
    if (!om) return -BLE_HS_ENOMEM;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_ppg_handle, om);
    if (rc != 0) return -rc;
    s_ppg_seq++; /* only after a successful queue */
    return (int)count;
}

int ble_monivitals_notify_hr(void)
{
    if (!s_connected || !s_notify_hr) return 0;
    /* bpm==0 + contact clear is intentional (tell app to drop sticky vitals). */
    uint8_t buf[3];
    /* Single atomic load so the contact bit and both bpm bytes are one
     * consistent snapshot, not three separate loads racing the setter. */
    uint16_t bpm = s_hr_bpm;
    buf[0] = (uint8_t)(0x05 | (s_contact && bpm > 0 ? 0x02 : 0x00));
    buf[1] = (uint8_t)(bpm & 0xFF);
    buf[2] = (uint8_t)(bpm >> 8);
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!om) return BLE_HS_ENOMEM;
    return ble_gatts_notify_custom(s_conn_handle, s_hr_handle, om);
}

int ble_monivitals_notify_plx(void)
{
    if (!s_connected || !s_notify_plx) return 0;
    if (s_spo2 == 0) return 0; /* no placeholder SpO2 */
    uint16_t sf = encode_sfloat((int16_t)(s_spo2 * 10), -1);
    uint16_t pr = hr_sfloat();
    uint8_t buf[5] = {
        0x00,
        (uint8_t)(sf & 0xFF),
        (uint8_t)(sf >> 8),
        (uint8_t)(pr & 0xFF),
        (uint8_t)(pr >> 8),
    };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, sizeof(buf));
    if (!om) return BLE_HS_ENOMEM;
    return ble_gatts_notify_custom(s_conn_handle, s_plx_handle, om);
}
