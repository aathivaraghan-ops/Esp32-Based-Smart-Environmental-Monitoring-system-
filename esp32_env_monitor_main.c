/*
 * ESP32 Smart Environmental Monitoring System - Embedded C (ESP-IDF)
 * Author : B. Aathi Varaghan
 *
 * Measures:
 *   - Temperature and humidity  : DHT22 (single-wire, bit-banged driver below)
 *   - Air quality (CO2-equiv)   : MQ-135  (analog, ADC1)
 *   - Gas level (LPG / smoke)   : MQ-2    (analog, ADC1)
 *   - Light intensity           : LDR + 10k divider (analog, ADC1)
 * Alerts: buzzer + LED when a limit is crossed. Results are printed on the serial monitor.
 *
 * Target     : ESP32 (ESP32-WROOM-32 DevKit), ESP-IDF v5.2 or newer
 * Build/Flash: idf.py set-target esp32 && idf.py build flash monitor
 *
 * Wiring (change the macros below if yours differs):
 *   DHT22 DATA -> GPIO4  (10k pull-up to 3.3 V)
 *   MQ-135 AO  -> GPIO34 (ADC1_CH6) through a 10k/20k divider (module outputs up to 5 V)
 *   MQ-2   AO  -> GPIO32 (ADC1_CH4) through a 10k/20k divider
 *   LDR        -> GPIO35 (ADC1_CH7): 3V3 - LDR - GPIO35 - 10k - GND
 *   Buzzer     -> GPIO25 (via BC547 transistor), LED -> GPIO2 (with 220 ohm)
 *
 * NOTE: ADC1 pins are used because ADC2 cannot be used reliably while Wi-Fi is on.
 * MQ sensor values are approximate and need pre-heating and calibration in clean air.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "ENV_MON";

/* ---------------- Pin and channel map ---------------- */
#define PIN_DHT22        GPIO_NUM_4
#define PIN_BUZZER       GPIO_NUM_25
#define PIN_LED          GPIO_NUM_2
#define CH_MQ135         ADC_CHANNEL_6    /* GPIO34 */
#define CH_LDR           ADC_CHANNEL_7    /* GPIO35 */
#define CH_MQ2           ADC_CHANNEL_4    /* GPIO32 */

/* ---------------- Settings ---------------- */
#define SAMPLE_PERIOD_MS 2000             /* DHT22 needs >= 2 s between reads */
#define ADC_SAMPLES      16               /* averaging to reduce noise */
#define MQ_WARMUP_S      20               /* pre-heat time (use 24 h for first calibration) */

#define DIVIDER_RATIO    1.5f             /* 10k/20k divider: Vsensor = Vadc * 1.5 */
#define MQ_VCC_V         5.0f
#define MQ_RL_KOHM       10.0f            /* load resistor on the MQ module */
#define MQ135_R0_KOHM    76.63f           /* calibrate: Rs in clean air / 3.6 */
#define MQ2_R0_KOHM      9.83f            /* calibrate: Rs in clean air / 9.8 */

/* Alert limits */
#define TEMP_MAX_C       35.0f
#define HUM_MAX_PCT      90.0f
#define AQ_MAX_PPM       1000.0f          /* MQ-135 CO2-equivalent */
#define GAS_MAX_PPM      1000.0f          /* MQ-2 LPG-equivalent */

typedef struct {
    float temp_c;
    float hum_pct;
    float aq_ppm;
    float gas_ppm;
    float light_pct;
} env_data_t;

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;

/* ===================== ADC helpers ===================== */
static void adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, CH_MQ135, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, CH_MQ2,   &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, CH_LDR,   &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        s_cali = NULL;
        ESP_LOGW(TAG, "ADC calibration not available, using linear estimate");
    }
}

/* Average of ADC_SAMPLES readings, returned in millivolts at the ADC pin */
static float adc_read_mv(adc_channel_t ch)
{
    int32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    int raw_avg = (int)(sum / ADC_SAMPLES);
    int mv = 0;
    if (s_cali && adc_cali_raw_to_voltage(s_cali, raw_avg, &mv) == ESP_OK) {
        return (float)mv;
    }
    return raw_avg * 3100.0f / 4095.0f;     /* rough fallback for 12 dB attenuation */
}

/* Sensor resistance Rs (kohm) of an MQ module from its analog output */
static float mq_rs_kohm(adc_channel_t ch)
{
    float v_sensor = (adc_read_mv(ch) / 1000.0f) * DIVIDER_RATIO;
    if (v_sensor < 0.05f) {
        return 1e6f;                         /* no signal: treat as very high resistance */
    }
    return ((MQ_VCC_V / v_sensor) - 1.0f) * MQ_RL_KOHM;
}

/* MQ-135: approximate CO2-equivalent ppm (datasheet power-law curve) */
static float read_air_quality_ppm(void)
{
    float ratio = mq_rs_kohm(CH_MQ135) / MQ135_R0_KOHM;
    return 116.6020682f * powf(ratio, -2.769034857f);
}

/* MQ-2: approximate LPG-equivalent ppm (datasheet power-law curve) */
static float read_gas_level_ppm(void)
{
    float ratio = mq_rs_kohm(CH_MQ2) / MQ2_R0_KOHM;
    return 574.25f * powf(ratio, -2.222f);
}

/* LDR: 0 % = dark, 100 % = bright */
static float read_light_percent(void)
{
    float pct = adc_read_mv(CH_LDR) / 3100.0f * 100.0f;
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

/* ===================== DHT22 driver ===================== */
/* Wait until the pin reaches 'level'; returns elapsed microseconds or -1 on timeout */
static int dht_wait_level(int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(PIN_DHT22) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
    }
    return (int)(esp_timer_get_time() - start);
}

static bool dht22_read(float *temp_c, float *hum_pct)
{
    uint8_t data[5] = {0};
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    bool ok = true;

    gpio_set_direction(PIN_DHT22, GPIO_MODE_INPUT_OUTPUT_OD);

    portENTER_CRITICAL(&mux);                 /* timing-critical: no interrupts */
    gpio_set_level(PIN_DHT22, 0);             /* start signal: pull low >= 1 ms */
    esp_rom_delay_us(2000);
    gpio_set_level(PIN_DHT22, 1);             /* release the line */
    esp_rom_delay_us(30);

    /* sensor response: ~80 us low, ~80 us high */
    if (dht_wait_level(0, 100) < 0 || dht_wait_level(1, 100) < 0 || dht_wait_level(0, 100) < 0) {
        ok = false;
    }

    /* 40 data bits: 50 us low, then 26-28 us high = 0 or ~70 us high = 1 */
    for (int i = 0; ok && i < 40; i++) {
        if (dht_wait_level(1, 100) < 0) { ok = false; break; }
        int high_us = dht_wait_level(0, 120);
        if (high_us < 0) { ok = false; break; }
        data[i / 8] <<= 1;
        if (high_us > 40) {
            data[i / 8] |= 1;
        }
    }
    portEXIT_CRITICAL(&mux);

    if (!ok) {
        return false;
    }
    if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        return false;                         /* checksum error */
    }

    *hum_pct = (float)((data[0] << 8) | data[1]) / 10.0f;
    float t  = (float)(((data[2] & 0x7F) << 8) | data[3]) / 10.0f;
    *temp_c  = (data[2] & 0x80) ? -t : t;
    return true;
}

/* ===================== Classification helpers ===================== */
static const char *air_quality_label(float ppm)
{
    if (ppm < 800.0f)  return "GOOD";
    if (ppm < 1200.0f) return "MODERATE";
    if (ppm < 2000.0f) return "POOR";
    return "HAZARDOUS";
}

static const char *gas_level_label(float ppm)
{
    if (ppm < 300.0f)  return "NORMAL";
    if (ppm < 1000.0f) return "WARNING";
    return "DANGER";
}

static const char *light_label(float pct)
{
    if (pct < 20.0f) return "DARK";
    if (pct < 60.0f) return "DIM";
    return "BRIGHT";
}

/* ===================== Alert output ===================== */
static void gpio_outputs_setup(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_BUZZER) | (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(PIN_BUZZER, 0);
    gpio_set_level(PIN_LED, 0);
}

static bool check_alert(const env_data_t *d)
{
    bool alert = (d->temp_c  > TEMP_MAX_C)  ||
                 (d->hum_pct > HUM_MAX_PCT) ||
                 (d->aq_ppm  > AQ_MAX_PPM)  ||
                 (d->gas_ppm > GAS_MAX_PPM);
    gpio_set_level(PIN_BUZZER, alert);
    gpio_set_level(PIN_LED, alert);
    return alert;
}

/* ===================== Main sensing task ===================== */
static void sensor_task(void *arg)
{
    env_data_t d = { .temp_c = 0, .hum_pct = 0 };

    ESP_LOGI(TAG, "Warming up MQ sensors for %d s...", MQ_WARMUP_S);
    vTaskDelay(pdMS_TO_TICKS(MQ_WARMUP_S * 1000));

    for (;;) {
        float t, h;
        if (dht22_read(&t, &h)) {              /* keep last good value if a read fails */
            d.temp_c  = t;
            d.hum_pct = h;
        } else {
            ESP_LOGW(TAG, "DHT22 read failed (timeout/checksum), keeping last value");
        }

        d.aq_ppm    = read_air_quality_ppm();
        d.gas_ppm   = read_gas_level_ppm();
        d.light_pct = read_light_percent();

        bool alert = check_alert(&d);

        ESP_LOGI(TAG, "----------------------------------------");
        ESP_LOGI(TAG, "Temperature : %.1f C", d.temp_c);
        ESP_LOGI(TAG, "Humidity    : %.1f %%RH", d.hum_pct);
        ESP_LOGI(TAG, "Air quality : %.0f ppm (%s)", d.aq_ppm, air_quality_label(d.aq_ppm));
        ESP_LOGI(TAG, "Gas level   : %.0f ppm (%s)", d.gas_ppm, gas_level_label(d.gas_ppm));
        ESP_LOGI(TAG, "Light       : %.0f %% (%s)", d.light_pct, light_label(d.light_pct));
        ESP_LOGI(TAG, "Alert       : %s", alert ? "ACTIVE" : "none");

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void app_main(void)
{
    gpio_outputs_setup();
    adc_setup();
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
