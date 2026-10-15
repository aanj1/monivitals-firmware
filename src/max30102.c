/**
 * MAX30102 I2C driver (ESP-IDF) — SpO2 mode, 100 sps, 18-bit RED+IR.
 * Register sequence matches the Samabeat STM32 driver.
 */

#include "max30102.h"
#include "board.h"

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h>
#include <string.h>

static const char *TAG = "max30102";

#define I2C_PORT            I2C_NUM_0

#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PART_ID         0xFF

#define MODE_RESET          0x40
#define MODE_SPO2           0x03

#define FIFO_SMP_AVE_NONE   (0x00 << 5)
#define FIFO_ROLLOVER_EN    (1 << 4)
#define FIFO_A_FULL(n)      ((n) & 0x0F)

#define SPO2_ADC_RANGE_4096 (0x01 << 5)
#define SPO2_SR_100         (0x01 << 2)
#define SPO2_LED_PW_411     0x03

static bool s_ready;
static bool s_ovf_latch; /* FIFO overflow seen since last take (stream_task only) */

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, MAX30102_I2C_ADDR, buf, 2,
                                      pdMS_TO_TICKS(50));
}

static esp_err_t rd(uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(I2C_PORT, MAX30102_I2C_ADDR,
                                        &reg, 1, val, 1, pdMS_TO_TICKS(50));
}

static esp_err_t rd_n(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MAX30102_I2C_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

int max30102_init(void)
{
    if (s_ready) return 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SB_PIN_I2C_SDA,
        .scl_io_num = SB_PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = SB_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) return -EIO;
    err = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return -EIO;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << SB_PIN_MAX30102_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    uint8_t part = 0;
    if (rd(REG_PART_ID, &part) != ESP_OK) {
        ESP_LOGE(TAG, "I2C read PART_ID failed (SDA=%d SCL=%d)",
                 SB_PIN_I2C_SDA, SB_PIN_I2C_SCL);
        return -EIO;
    }
    if (part != MAX30102_PART_ID) {
        ESP_LOGE(TAG, "unexpected PART_ID=0x%02X (want 0x%02X)", part, MAX30102_PART_ID);
        return -ENODEV;
    }

    if (wr(REG_MODE_CONFIG, MODE_RESET) != ESP_OK) return -EIO;
    vTaskDelay(pdMS_TO_TICKS(10));
    bool reset_cleared = false;
    for (int i = 0; i < 100; i++) {
        uint8_t mode = 0;
        if (rd(REG_MODE_CONFIG, &mode) != ESP_OK) return -EIO;
        if (!(mode & MODE_RESET)) { reset_cleared = true; break; }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if (!reset_cleared) {
        /* Reset bit never self-cleared: the chip is wedged / not responding
         * correctly. Fail instead of falling through and configuring a chip
         * that never reset — the caller retries init so a transient glitch
         * self-heals rather than silently producing a dead sensor. */
        ESP_LOGE(TAG, "MODE_RESET never cleared — chip not responding");
        return -EIO;
    }

    uint8_t dummy;
    rd(REG_INTR_STATUS_1, &dummy);
    rd(REG_INTR_STATUS_2, &dummy);

    /* Every config write checked — never mark a misconfigured chip ready. */
    if (wr(REG_INTR_ENABLE_1, 0x80) != ESP_OK) return -EIO; /* A_FULL */
    if (wr(REG_FIFO_CONFIG,
           FIFO_SMP_AVE_NONE | FIFO_ROLLOVER_EN | FIFO_A_FULL(0x0F)) != ESP_OK) return -EIO;
    if (wr(REG_SPO2_CONFIG,
           SPO2_ADC_RANGE_4096 | SPO2_SR_100 | SPO2_LED_PW_411) != ESP_OK) return -EIO;
    /* Start moderate — AGC raises/lowers so firm contact does not saturate. */
    if (wr(REG_LED1_PA, 0x24) != ESP_OK) return -EIO;
    if (wr(REG_LED2_PA, 0x24) != ESP_OK) return -EIO;
    if (wr(REG_FIFO_WR_PTR, 0) != ESP_OK) return -EIO;
    if (wr(REG_OVF_COUNTER, 0) != ESP_OK) return -EIO;
    if (wr(REG_FIFO_RD_PTR, 0) != ESP_OK) return -EIO;
    if (wr(REG_MODE_CONFIG, MODE_SPO2) != ESP_OK) return -EIO;

    s_ready = true;
    ESP_LOGI(TAG, "ready PART_ID=0x%02X SpO2@100sps SDA=%d SCL=%d",
             part, SB_PIN_I2C_SDA, SB_PIN_I2C_SCL);
    return 0;
}

bool max30102_set_led_current(uint8_t red_pa, uint8_t ir_pa)
{
    if (!s_ready) return false;
    if (wr(REG_LED1_PA, red_pa) != ESP_OK) return false;
    return wr(REG_LED2_PA, ir_pa) == ESP_OK;
}

/* --- AGC: keep IR DC in 25–75% of 18-bit full scale --- */
#define AGC_LED_MIN   0x08
#define AGC_LED_MAX   0x32
#define AGC_DC_LOW    65535U    /* 25% FS */
#define AGC_DC_HIGH   196605U   /* 75% FS */
#define AGC_PERIOD_MS 500

static uint32_t s_agc_ema_ir;
static uint8_t s_agc_pa = 0x24;
static uint32_t s_agc_last_ms;

void max30102_agc_feed(uint32_t red, uint32_t ir, uint32_t tick_ms)
{
    (void)red;
    if (!s_ready) return;
    if (s_agc_ema_ir == 0) {
        s_agc_ema_ir = ir;
    } else {
        s_agc_ema_ir = (s_agc_ema_ir * 15u + ir) / 16u;
    }
    if (tick_ms - s_agc_last_ms < AGC_PERIOD_MS) return;
    s_agc_last_ms = tick_ms;

    uint8_t pa = s_agc_pa;
    /* Bigger steps when deeply saturated so firm finger recovers quickly. */
    if (s_agc_ema_ir > 230000U && pa > AGC_LED_MIN + 4) {
        pa = (uint8_t)(pa - 4);
    } else if (s_agc_ema_ir > AGC_DC_HIGH && pa > AGC_LED_MIN) {
        pa--;
    } else if (s_agc_ema_ir < AGC_DC_LOW && pa < AGC_LED_MAX) {
        pa = (uint8_t)(pa + (s_agc_ema_ir < 40000U ? 2 : 1));
        if (pa > AGC_LED_MAX) pa = AGC_LED_MAX;
    }
    if (pa != s_agc_pa) {
        /* Commit the shadow (s_agc_pa) only after BOTH LEDs actually took the
         * new current. max30102_set_led_current() writes LED1 (RED) then LED2
         * (IR); if it returns false, LED1 may have succeeded while LED2 failed,
         * leaving RED != IR — which skews the ratio-of-ratios (SpO2) silently.
         * On failure, leave s_agc_pa unchanged and retry the write next period
         * (which re-writes BOTH LEDs), so RED and IR reconverge instead of
         * desyncing from an ignored return. */
        if (max30102_set_led_current(pa, pa)) {
            s_agc_pa = pa;
            ESP_LOGI(TAG, "AGC LED_PA=0x%02X dc_ir≈%lu", pa, (unsigned long)s_agc_ema_ir);
        } else {
            ESP_LOGW(TAG, "AGC LED write failed (target 0x%02X) — retry next period", pa);
        }
    }
}

bool max30102_is_ready(void)
{
    return s_ready;
}

bool max30102_take_fifo_overflow(void)
{
    bool v = s_ovf_latch;
    s_ovf_latch = false;
    return v;
}

int max30102_read_fifo(max30102_sample_t *buf, size_t cap)
{
    if (!s_ready || !buf || cap == 0) return 0;
    if (cap > 16) cap = 16;

    uint8_t ovf = 0;
    if (rd(REG_OVF_COUNTER, &ovf) != ESP_OK) return 0;
    if (ovf) {
        wr(REG_OVF_COUNTER, 0);
        /* Samples were lost — latch it so the caller can invalidate beat
         * intervals and signal the gap downstream (PPG seq skip). */
        s_ovf_latch = true;
        ESP_LOGW(TAG, "FIFO overflow: %u samples lost", (unsigned)ovf);
    }

    uint8_t wr_ptr = 0, rd_ptr = 0;
    if (rd(REG_FIFO_WR_PTR, &wr_ptr) != ESP_OK) return 0;
    if (rd(REG_FIFO_RD_PTR, &rd_ptr) != ESP_OK) return 0;

    int n = (int)wr_ptr - (int)rd_ptr;
    if (n < 0) n += 32;
    if (n == 0) return 0;
    if ((size_t)n > cap) n = (int)cap;

    uint8_t raw[16 * 6];
    if (rd_n(REG_FIFO_DATA, raw, (size_t)n * 6) != ESP_OK) return 0;

    for (int i = 0; i < n; i++) {
        const uint8_t *p = &raw[i * 6];
        buf[i].red = ((uint32_t)(p[0] & 0x03) << 16) |
                     ((uint32_t)p[1] << 8) |
                     (uint32_t)p[2];
        buf[i].ir  = ((uint32_t)(p[3] & 0x03) << 16) |
                     ((uint32_t)p[4] << 8) |
                     (uint32_t)p[5];
    }
    return n;
}
