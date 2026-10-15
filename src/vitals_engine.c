/**
 * Real-use PPG → HR / SpO2 / PI for MAX30102.
 *
 * Signal chain (ESP32-C3 has no FPU, but at 100 Hz soft-float is trivial —
 * ~4 biquads * 100 Hz = negligible):
 *   raw IR/RED ─► Butterworth band-pass 0.5–5 Hz (HP@0.5 + LP@5, RBJ biquads)
 *              ├─► peak detector (adaptive envelope threshold) ─► HR
 *              └─► RMS over a 2 s window ─► ratio-of-ratios R ─► SpO2 (table)
 *   integer DC EMA (kept) feeds the R / PI denominators and the finger gate.
 *
 * Rules for real users:
 *  - Never fabricate SpO2 floors (no clamping junk up to 70/98)
 *  - Finger requires DC sweet-spot + sane PI, with debounce
 *  - Hold last reading only while finger still on (brief quality dip)
 *  - Smooth display values for human-readable UI
 *  - SpO2 is INDICATIVE, not calibrated — see SPO2 CALIBRATION note below.
 */

#include "vitals_engine.h"

#include <string.h>
#include <math.h>

#define SAMPLE_HZ            100
#define HR_MIN_BPM           40
#define HR_MAX_BPM           160
#define HR_MIN_INTERVAL      (SAMPLE_HZ * 60 / HR_MAX_BPM) /* 37 */
#define HR_MAX_INTERVAL      (SAMPLE_HZ * 60 / HR_MIN_BPM) /* 150 */
#define HR_BLANKING          35   /* 350 ms */
#define HR_FLUSH_TIMEOUT     (HR_MAX_INTERVAL * 3)
#define MEDIAN_N             5
#define WARMUP_SAMPLES       100  /* 1.0 s — faster first lock */

#define DC_FINGER_MIN        20000
#define DC_FINGER_MAX        230000
#define DC_SATURATED         250000

#define PI_MIN_X10           1     /* 0.1% — firm contact small AC; DC gate blocks no-finger */
#define PI_MAX_X10           200   /* 20% — match app safe band; keep strong-perfusion fingers */

#define R_X100_MIN           40
#define R_X100_MAX           200

#define DC_SHIFT             5
#define HOLD_SAMPLES         500   /* ~5 s while finger still on */
#define FINGER_ON_NEED       30    /* ~0.3 s consecutive good */
#define FINGER_OFF_NEED      80    /* ~0.8 s consecutive bad — stickier on */

#define ENV_DECAY            0.997f /* AC envelope decay (~333 ms) */
#define RMS_WINDOW           200    /* 2 s of samples for the ratio-of-ratios */

/*
 * SpO2 CALIBRATION (lever 2)
 * --------------------------
 * SpO2 = f(R), R = (AC_red/DC_red)/(AC_ir/DC_ir).  The table below is the
 * widely-cited empirical transmissive curve; it is NOT calibrated to THIS
 * board's LED wavelengths / photodiode geometry, so treat output as indicative.
 *
 * To calibrate for real accuracy:
 *   1. Record R (log s_r_x100) alongside a reference pulse-oximeter reading
 *      across several subjects / SpO2 levels (breath-hold to get 90–100%).
 *   2. Fit SpO2 = a - b*R (or refit the table knots) to your (R, SpO2_ref) pairs.
 *   3. Put the fit in s_lut[] (r_x100 knots) and/or set SPO2_CAL_OFFSET for a
 *      single-point trim after the shape matches.
 */
#define SPO2_CAL_OFFSET      0      /* per-device single-point trim, in % */

typedef struct {
    uint16_t r_x100;
    uint8_t spo2;
} spo2_lut_t;

static const spo2_lut_t s_lut[] = {
    {  40, 100 }, {  50, 100 }, {  60,  99 }, {  70,  99 },
    {  80,  98 }, {  90,  96 }, { 100,  95 }, { 110,  93 },
    { 120,  91 }, { 130,  90 }, { 140,  89 }, { 150,  88 },
    { 163,  86 }, { 175,  85 }, { 188,  83 }, { 200,  80 },
};
#define LUT_N (sizeof(s_lut) / sizeof(s_lut[0]))

/* --- Direct-Form-II-transposed biquad (float) --- */
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} biquad_t;

static inline float biquad_run(biquad_t *f, float x)
{
    float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

/* RBJ cookbook, Butterworth Q=0.7071, fs=100 Hz (a0-normalized). */
static void biquad_hp_05hz(biquad_t *f)
{
    f->b0 =  0.978029f; f->b1 = -1.956058f; f->b2 =  0.978029f;
    f->a1 = -1.955580f; f->a2 =  0.956544f;
    f->z1 = f->z2 = 0.0f;
}

static void biquad_lp_5hz(biquad_t *f)
{
    f->b0 =  0.020084f; f->b1 =  0.040167f; f->b2 =  0.020084f;
    f->a1 = -1.561023f; f->a2 =  0.641348f;
    f->z1 = f->z2 = 0.0f;
}

static int32_t s_dc_ir, s_dc_red;

/* Band-pass filter state: HP(0.5) → LP(5) per channel. */
static biquad_t s_ir_hp, s_ir_lp, s_red_hp, s_red_lp;

/* Peak detection on band-passed IR. */
static float s_env_ir;        /* decaying peak envelope (AC amplitude estimate) */
static float s_bp_prev, s_bp_prev2;

/* Ratio-of-ratios accumulators (RMS over RMS_WINDOW). */
static float s_sumsq_ir, s_sumsq_red;
static uint16_t s_rms_count;
static uint16_t s_r_x100;      /* latest windowed ratio-of-ratios * 100 (0 = none) */

static uint16_t s_since_peak;
static uint16_t s_intervals[MEDIAN_N];
static uint8_t s_iv_idx, s_iv_filled;
static uint32_t s_count;
static uint32_t s_last_valid_count;
static uint32_t s_last_hr_count;
static uint32_t s_last_spo2_count;
static uint16_t s_finger_on_streak;
static uint16_t s_finger_off_streak;

static uint16_t s_hr_raw;
static uint8_t s_spo2_raw;
static uint16_t s_hr_disp;
static uint8_t s_spo2_disp;
static uint8_t s_pi_x10;
static bool s_finger;
static bool s_valid;

/* Contiguous PPG waveform ring — every processed sample lands here so the app
 * receives the FULL 100 sps stream (not just the last loop's tail), which its
 * RR / HRV DSP needs to be accurate. main.c drains it on each PPG notify. */
#define WAVE_RING_N 256
static int16_t s_wave_ring[WAVE_RING_N];
static uint16_t s_wave_head;   /* next write index */
static uint16_t s_wave_avail;  /* unread samples buffered */
static bool s_wave_overflow;   /* ring overwrote unread samples (consumer behind) */

static void filters_init(void)
{
    biquad_hp_05hz(&s_ir_hp);
    biquad_lp_5hz(&s_ir_lp);
    biquad_hp_05hz(&s_red_hp);
    biquad_lp_5hz(&s_red_lp);
}

void vitals_engine_reset(void)
{
    s_dc_ir = s_dc_red = 0;
    filters_init();
    s_env_ir = 0.0f;
    s_bp_prev = s_bp_prev2 = 0.0f;
    s_sumsq_ir = s_sumsq_red = 0.0f;
    s_rms_count = 0;
    s_r_x100 = 0;
    s_since_peak = 0;
    s_iv_idx = s_iv_filled = 0;
    s_count = 0;
    s_last_valid_count = 0;
    s_last_hr_count = 0;
    s_last_spo2_count = 0;
    s_finger_on_streak = s_finger_off_streak = 0;
    s_hr_raw = s_hr_disp = 0;
    s_spo2_raw = s_spo2_disp = 0;
    s_pi_x10 = 0;
    s_finger = false;
    s_valid = false;
    s_wave_head = 0;
    s_wave_avail = 0;
    s_wave_overflow = false;
    memset(s_intervals, 0, sizeof(s_intervals));
}

void vitals_engine_soft_reset(void)
{
    /* Keep DC + warm filter state (AGC-warmed); clear beat lock + display. */
    s_bp_prev = s_bp_prev2 = 0.0f;
    s_sumsq_ir = s_sumsq_red = 0.0f;
    s_rms_count = 0;
    s_r_x100 = 0;
    s_since_peak = 0;
    s_iv_idx = s_iv_filled = 0;
    s_last_valid_count = 0;
    s_last_hr_count = 0;
    s_last_spo2_count = 0;
    s_hr_raw = s_hr_disp = 0;
    s_spo2_raw = s_spo2_disp = 0;
    s_valid = false;
    s_count = 0;
    s_wave_head = 0;
    s_wave_avail = 0;
    s_wave_overflow = false;
    memset(s_intervals, 0, sizeof(s_intervals));
}

/* An upstream sample gap (sensor FIFO overflow) makes any measurement that
 * spans it wrong: beat intervals, the ratio-of-ratios window, and the peak
 * timer. Reset those; keep DC / filters / finger state, which recover fast.
 * Also drop the not-yet-notified pre-gap waveform so the single PPG seq skip
 * the caller emits lands exactly on a notification boundary — otherwise the
 * phone would place the gap one notify too early and stitch across the real
 * one. Overflow is rare; losing ≤ ring (2.5 s) of wave there is acceptable. */
void vitals_engine_note_gap(void)
{
    s_iv_idx = s_iv_filled = 0;
    memset(s_intervals, 0, sizeof(s_intervals));
    /* Large sentinel: the first post-gap interval exceeds HR_MAX_INTERVAL and
     * is rejected by the range gate, so no gap-spanning interval enters. */
    s_since_peak = 0xF000;
    s_bp_prev = s_bp_prev2 = 0.0f;
    s_sumsq_ir = s_sumsq_red = 0.0f;
    s_rms_count = 0;
    s_wave_avail = 0;
    s_wave_overflow = false;
}

/* Peek the oldest buffered PPG samples (up to cap) for a BLE notify WITHOUT
 * removing them — the caller consumes only after the notify succeeds, so a
 * failed notify never destroys buffered waveform. Returns the count written. */
size_t vitals_engine_peek_wave(int16_t *out, size_t cap)
{
    if (!out || cap == 0 || s_wave_avail == 0) return 0;
    size_t take = s_wave_avail < cap ? s_wave_avail : cap;
    uint16_t start = (uint16_t)((s_wave_head + WAVE_RING_N - s_wave_avail) % WAVE_RING_N);
    for (size_t i = 0; i < take; i++) {
        out[i] = s_wave_ring[(start + i) % WAVE_RING_N];
    }
    return take;
}

void vitals_engine_consume_wave(size_t n)
{
    if (n > s_wave_avail) n = s_wave_avail; /* never underflow the ring */
    s_wave_avail = (uint16_t)(s_wave_avail - n);
}

bool vitals_engine_take_wave_overflow(void)
{
    bool v = s_wave_overflow;
    s_wave_overflow = false;
    return v;
}

static int16_t clamp_i16(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static uint16_t median_interval(void)
{
    if (s_iv_filled < MEDIAN_N) return 0;
    uint16_t tmp[MEDIAN_N];
    memcpy(tmp, s_intervals, sizeof(tmp));
    for (int i = 1; i < MEDIAN_N; i++) {
        uint16_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[MEDIAN_N / 2];
}

static bool intervals_stable(void)
{
    if (s_iv_filled < MEDIAN_N) return false;
    uint16_t med = median_interval();
    if (med == 0) return false;
    int spread = 0;
    for (int i = 0; i < MEDIAN_N; i++) {
        int d = (int)s_intervals[i] - (int)med;
        if (d < 0) d = -d;
        if (d > spread) spread = d;
    }
    return spread <= (int)(med / 4); /* ~25% — real finger motion */
}

static uint8_t lookup_spo2(uint16_t r_x100)
{
    int spo2;
    if (r_x100 <= s_lut[0].r_x100) {
        spo2 = s_lut[0].spo2;
    } else if (r_x100 >= s_lut[LUT_N - 1].r_x100) {
        spo2 = s_lut[LUT_N - 1].spo2;
    } else {
        spo2 = 0;
        for (size_t i = 1; i < LUT_N; i++) {
            if (r_x100 <= s_lut[i].r_x100) {
                uint16_t r0 = s_lut[i - 1].r_x100;
                uint16_t r1 = s_lut[i].r_x100;
                int s0 = s_lut[i - 1].spo2;
                int s1 = s_lut[i].spo2;
                spo2 = (r1 == r0) ? s0
                     : s0 - (int)(r_x100 - r0) * (s0 - s1) / (int)(r1 - r0);
                break;
            }
        }
    }
    spo2 += SPO2_CAL_OFFSET;
    if (spo2 < 0) spo2 = 0;
    if (spo2 > 100) spo2 = 100;
    return (uint8_t)spo2;
}

static uint16_t ease_u16(uint16_t disp, uint16_t raw, uint16_t step)
{
    if (disp == 0) return raw;
    if (raw > disp + step) return disp + step;
    if (raw + step < disp) return disp - step;
    return raw;
}

static uint8_t ease_u8(uint8_t disp, uint8_t raw, uint8_t step)
{
    if (disp == 0) return raw;
    if (raw > disp + step) return (uint8_t)(disp + step);
    if (raw + step < disp) return (uint8_t)(disp - step);
    return raw;
}

void vitals_engine_process(const max30102_sample_t *in, int n, vitals_frame_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!in || n <= 0) {
        out->hr_bpm = (s_finger && s_hr_disp > 0) ? s_hr_disp : 0;
        out->spo2_pct = (s_finger && s_spo2_disp >= 80) ? s_spo2_disp : 0;
        out->pi_x10 = s_pi_x10;
        out->r_x100 = s_r_x100;
        out->finger_on = s_finger;
        out->valid = s_valid;
        out->env = (uint16_t)(s_env_ir > 65535.0f ? 65535.0f : s_env_ir);
        out->beats = s_iv_filled;
        return;
    }

    size_t wcount = 0;
    for (int i = 0; i < n; i++) {
        int32_t ir = (int32_t)in[i].ir;
        int32_t red = (int32_t)in[i].red;
        s_count++;

        /* DC EMA (integer) — used by the R / PI denominators + finger gate. */
        if (s_dc_ir == 0) {
            s_dc_ir = ir;
            s_dc_red = red;
        } else {
            s_dc_ir += (ir - s_dc_ir) >> DC_SHIFT;
            s_dc_red += (red - s_dc_red) >> DC_SHIFT;
        }

        /* Band-pass 0.5–5 Hz. HP removes DC + baseline wander; LP kills HF noise.
         * Result is a clean zero-mean pulsatile waveform for peak detection. */
        float bp_ir = biquad_run(&s_ir_lp, biquad_run(&s_ir_hp, (float)ir));
        float bp_red = biquad_run(&s_red_lp, biquad_run(&s_red_hp, (float)red));

        /* AC envelope (peak-hold with decay) for threshold + PI + wave scaling. */
        float mag = bp_ir < 0.0f ? -bp_ir : bp_ir;
        s_env_ir *= ENV_DECAY;
        if (mag > s_env_ir) s_env_ir = mag;

        /* Ratio-of-ratios: accumulate RMS energy of both channels over a window. */
        s_sumsq_ir += bp_ir * bp_ir;
        s_sumsq_red += bp_red * bp_red;
        if (++s_rms_count >= RMS_WINDOW) {
            float rms_ir = sqrtf(s_sumsq_ir / (float)s_rms_count);
            float rms_red = sqrtf(s_sumsq_red / (float)s_rms_count);
            if (rms_ir > 1.0f && s_dc_red > 0 && s_dc_ir > 0) {
                float r = (rms_red * (float)s_dc_ir) / (rms_ir * (float)s_dc_red);
                int rx = (int)(r * 100.0f + 0.5f);
                s_r_x100 = (rx < 0) ? 0 : (rx > 65535 ? 65535 : (uint16_t)rx);
            }
            s_sumsq_ir = s_sumsq_red = 0.0f;
            s_rms_count = 0;
        }

        if (s_since_peak < 0xFFFF) s_since_peak++;

        /* Peak detect on band-passed IR: local max above an adaptive threshold. */
        if (s_count >= 3) {
            float thr = s_env_ir * 0.35f;
            if (thr < 15.0f) thr = 15.0f;
            if (s_bp_prev > s_bp_prev2 && s_bp_prev > bp_ir &&
                s_bp_prev > thr && s_since_peak >= HR_BLANKING) {
                uint16_t interval = s_since_peak;
                if (interval >= HR_MIN_INTERVAL && interval <= HR_MAX_INTERVAL) {
                    s_intervals[s_iv_idx] = interval;
                    s_iv_idx = (uint8_t)((s_iv_idx + 1) % MEDIAN_N);
                    if (s_iv_filled < MEDIAN_N) s_iv_filled++;
                }
                s_since_peak = 0;
            }
            if (s_since_peak >= HR_FLUSH_TIMEOUT) {
                s_iv_filled = 0;
                s_iv_idx = 0;
            }
            s_bp_prev2 = s_bp_prev;
        }
        s_bp_prev = bp_ir;

        float scale = s_env_ir > 20.0f ? s_env_ir : 20.0f;
        /* Negate so the transmitted pulse deflects UPWARD at systole. Raw IR
         * dips at systole (more absorption), so band-passed bp_ir goes negative
         * on the pulse; without this the packed waveform would deflect DOWNWARD,
         * opposite the camera-red "pulse up" convention and the iOS parser's
         * expectation. Sign-only change: peak detect / RR / SpO2 upstream are all
         * sign-independent (peak on bp_ir, RMS uses squares, R uses magnitudes). */
        int16_t wv = clamp_i16((int32_t)(-bp_ir * 600.0f / scale));
        if (wcount < 32) out->wave[wcount++] = wv;   /* per-call snapshot (compat) */
        /* Every sample into the contiguous ring so the app gets full 100 sps. */
        s_wave_ring[s_wave_head] = wv;
        s_wave_head = (uint16_t)((s_wave_head + 1) % WAVE_RING_N);
        if (s_wave_avail < WAVE_RING_N) {
            s_wave_avail++;
        } else {
            s_wave_overflow = true; /* oldest unread sample lost */
        }
    }
    out->wave_count = wcount;

    int32_t pi_x10 = 0;
    if (s_dc_ir > 0) {
        pi_x10 = (int32_t)((int64_t)(s_env_ir) * 1000 / s_dc_ir);
        if (pi_x10 < 0) pi_x10 = 0;
        if (pi_x10 > 200) pi_x10 = 200;
    }

    bool sample_good = (s_dc_ir >= DC_FINGER_MIN) && (s_dc_ir <= DC_FINGER_MAX)
                       && (s_dc_ir < DC_SATURATED)
                       && (pi_x10 >= PI_MIN_X10) && (pi_x10 <= PI_MAX_X10);

    if (sample_good) {
        if (s_finger_on_streak < 0xFFFF) s_finger_on_streak++;
        s_finger_off_streak = 0;
        if (!s_finger && s_finger_on_streak >= FINGER_ON_NEED) {
            s_finger = true;
        }
    } else {
        if (s_finger_off_streak < 0xFFFF) s_finger_off_streak++;
        s_finger_on_streak = 0;
        if (s_finger && s_finger_off_streak >= FINGER_OFF_NEED) {
            s_finger = false;
        }
    }

    /* PI byte is display only; contact authority is the Quality contact byte
     * (s_finger). Keep a 0.1% floor/sentinel for legacy 1-byte web-bt readers. */
    s_pi_x10 = s_finger ? (uint8_t)(pi_x10 < 1 ? 1 : pi_x10) : 1;

    bool fresh_valid = false;
    if (s_finger && s_count >= WARMUP_SAMPLES
        && s_dc_red > DC_FINGER_MIN / 2 && s_env_ir > 30.0f) {
        /* HR straight from the robust median once we have MEDIAN_N intervals.
         * The median already rejects outlier beats, so we do NOT also demand the
         * strict intervals_stable() spread — real HRV + skin/motion noise rarely
         * satisfy it, which previously left HR/SpO2 stuck "settling" forever. */
        uint16_t med = median_interval();
        if (med > 0) {
            uint16_t bpm = (uint16_t)((SAMPLE_HZ * 60u) / med);
            if (bpm >= HR_MIN_BPM && bpm <= HR_MAX_BPM) {
                s_hr_raw = (s_hr_raw == 0)
                    ? bpm
                    : (uint16_t)((s_hr_raw * 7 + bpm) / 8);
            }
        }

        /* SpO2 from the windowed ratio-of-ratios — independent of HR stability. */
        if (s_r_x100 >= R_X100_MIN && s_r_x100 <= R_X100_MAX) {
            uint8_t spo2 = lookup_spo2(s_r_x100);
            if (spo2 >= 80 && spo2 <= 100) {
                s_spo2_raw = (s_spo2_raw == 0)
                    ? spo2
                    : (uint8_t)((s_spo2_raw * 7 + spo2) / 8);
            }
        }

        if (s_hr_raw >= HR_MIN_BPM) {
            s_hr_disp = ease_u16(s_hr_disp, s_hr_raw, 2);
            s_last_hr_count = s_count;
        }
        if (s_spo2_raw >= 80) {
            s_spo2_disp = ease_u8(s_spo2_disp, s_spo2_raw, 1);
            s_last_spo2_count = s_count;
        }
        /* HR alone is enough to be a valid, held reading; SpO2 fills in when it
         * locks (main.c streams each independently). */
        if (s_hr_raw >= HR_MIN_BPM) {
            fresh_valid = true;
            s_last_valid_count = s_count;
        }
    }

    if (!s_finger) {
        s_hr_raw = 0;
        s_spo2_raw = 0;
        s_iv_filled = 0;
        s_iv_idx = 0;
        s_hr_disp = 0;
        s_spo2_disp = 0;
        s_valid = false;
        s_last_valid_count = 0;
        s_last_hr_count = 0;
        s_last_spo2_count = 0;
        /* Restart the ratio-of-ratios window on a fresh finger. */
        s_sumsq_ir = s_sumsq_red = 0.0f;
        s_rms_count = 0;
        s_r_x100 = 0;
    } else if (fresh_valid) {
        s_valid = true;
    } else if (s_last_valid_count > 0 &&
               (s_count - s_last_valid_count) <= HOLD_SAMPLES &&
               s_hr_disp > 0 && s_spo2_disp >= 80) {
        /* Finger still on, brief algorithm dip — hold last smooth reading. */
        s_valid = true;
    } else {
        s_valid = false;
        /* Independent stale wipe — HR can survive without SpO2 lock. */
        if (s_last_hr_count == 0 || (s_count - s_last_hr_count) > HOLD_SAMPLES) {
            s_hr_disp = 0;
            s_hr_raw = 0;
        }
        if (s_last_spo2_count == 0 || (s_count - s_last_spo2_count) > HOLD_SAMPLES) {
            s_spo2_disp = 0;
            s_spo2_raw = 0;
        }
    }

    /* Publish HR/SpO2 independently while finger is on so the app can show
     * HR during SpO2 settle (and keep sticky values across brief dips). */
    out->hr_bpm = (s_finger && s_hr_disp > 0) ? s_hr_disp : 0;
    out->spo2_pct = (s_finger && s_spo2_disp >= 80) ? s_spo2_disp : 0;
    out->pi_x10 = s_pi_x10;
    out->r_x100 = s_r_x100;
    out->finger_on = s_finger;
    out->valid = s_valid;
    out->env = (uint16_t)(s_env_ir > 65535.0f ? 65535.0f : s_env_ir);
    out->beats = s_iv_filled;
}
