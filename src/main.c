/*
 * MKS SERVO42C control demo for ESP32 (ESP-IDF).
 *
 * Wiring (see include/servo_config.h):
 *   ESP32 GPIO17 (TX)  ->  SERVO42C RX
 *   ESP32 GPIO16 (RX)  <-  SERVO42C TX
 *   ESP32 GND          <-> SERVO42C GND
 *
 * The servo must be set to Mode = CR_UART for the motion commands to work.
 */

#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mks_servo42c.h"
#include "servo_config.h"

static const char *TAG = "app";

static mks_t servo;

/* Generous completion timeout for a move: the time the move should take at
 * the requested speed, plus margin for acceleration and the reply itself. */
static uint32_t move_timeout_ms(float revolutions, float rpm)
{
    if (rpm < 0.1f) {
        rpm = 0.1f;
    }
    float seconds = (revolutions < 0 ? -revolutions : revolutions) / rpm * 60.0f;
    return (uint32_t)(seconds * 1000.0f) + 3000u;
}

static const char *en_state_str(mks_en_state_t state)
{
    switch (state) {
    case MKS_EN_STATE_ENABLED:  return "enabled";
    case MKS_EN_STATE_DISABLED: return "disabled";
    default:                    return "error";
    }
}

static const char *protect_state_str(mks_protect_state_t state)
{
    switch (state) {
    case MKS_PROTECT_TRIPPED:     return "tripped (shaft was blocked)";
    case MKS_PROTECT_NOT_TRIPPED: return "ok";
    default:                      return "error";
    }
}

/* Reads back everything the servo will tell us about its current state. */
static void log_servo_state(void)
{
    int32_t carry = 0;
    uint16_t encoder = 0;
    if (mks_read_encoder(&servo, &carry, &encoder) == ESP_OK) {
        ESP_LOGI(TAG, "encoder: carry=%" PRId32 " value=%u", carry, encoder);
    }

    float angle = 0.0f;
    if (mks_read_angle_deg(&servo, &angle) == ESP_OK) {
        ESP_LOGI(TAG, "shaft angle: %.2f deg", angle);
    }

    float error = 0.0f;
    if (mks_read_angle_error_deg(&servo, &error) == ESP_OK) {
        ESP_LOGI(TAG, "angle error: %.3f deg", error);
    }

    int32_t pulses = 0;
    if (mks_read_pulses(&servo, &pulses) == ESP_OK) {
        ESP_LOGI(TAG, "pulses received: %" PRId32, pulses);
    }

    mks_en_state_t en = MKS_EN_STATE_ERROR;
    if (mks_read_en_state(&servo, &en) == ESP_OK) {
        ESP_LOGI(TAG, "En pin: %s", en_state_str(en));
    }

    mks_protect_state_t protect = MKS_PROTECT_ERROR;
    if (mks_read_protect_state(&servo, &protect) == ESP_OK) {
        ESP_LOGI(TAG, "stall protection: %s", protect_state_str(protect));
        if (protect == MKS_PROTECT_TRIPPED) {
            ESP_LOGW(TAG, "clearing locked-rotor protection");
            mks_release_protect(&servo);
        }
    }
}

/* Confirms the link is alive before we try to move anything. */
static esp_err_t probe_servo(void)
{
    for (int attempt = 1; attempt <= 5; attempt++) {
        int32_t carry = 0;
        uint16_t value = 0;
        esp_err_t err = mks_read_encoder(&servo, &carry, &value);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "servo responded on attempt %d (encoder %u)", attempt, value);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "probe attempt %d failed: %s", attempt, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    return ESP_ERR_TIMEOUT;
}

#if SERVO_APPLY_SETTINGS_ON_BOOT
/* Writes the servo's EEPROM. Off by default, see servo_config.h. */
static void apply_settings(void)
{
    ESP_LOGI(TAG, "writing servo settings to EEPROM");

    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_motor_type(
        &servo, SERVO_STEP_ANGLE_IS_1_8 ? MKS_MOTOR_1_8_DEG : MKS_MOTOR_0_9_DEG));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_mode(&servo, MKS_MODE_CR_UART));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_current_ma(&servo, SERVO_BOOT_CURRENT_MA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_microsteps(&servo, SERVO_MICROSTEPS));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_stall_protection(&servo, true));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_interpolation(&servo, true));
}
#endif

/* One pass of the demo: position moves in both directions, then a timed
 * constant-speed run. */
static void run_demo_cycle(void)
{
    const uint32_t timeout = move_timeout_ms(DEMO_REVOLUTIONS, DEMO_RPM);

    ESP_LOGI(TAG, "--- moving %+.2f rev at %.1f RPM (CW) ---",
             DEMO_REVOLUTIONS, DEMO_RPM);
    esp_err_t err = mks_move_revolutions(&servo, DEMO_REVOLUTIONS, DEMO_RPM, true, timeout);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "move failed: %s", esp_err_to_name(err));
    }
    log_servo_state();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "--- moving %+.2f rev at %.1f RPM (CCW) ---",
             -DEMO_REVOLUTIONS, DEMO_RPM);
    err = mks_move_revolutions(&servo, -DEMO_REVOLUTIONS, DEMO_RPM, true, timeout);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "move failed: %s", esp_err_to_name(err));
    }
    log_servo_state();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "--- constant speed %.1f RPM for 3 s ---", DEMO_RPM);
    err = mks_run_rpm(&servo, MKS_DIR_CW, DEMO_RPM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "constant-speed run failed: %s", esp_err_to_name(err));
    } else {
        vTaskDelay(pdMS_TO_TICKS(3000));
        err = mks_stop(&servo);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "stop failed: %s", esp_err_to_name(err));
        }
    }
    log_servo_state();
}

void app_main(void)
{
    const mks_config_t cfg = {
        .uart_num         = SERVO_UART_PORT,
        .tx_gpio          = SERVO_TX_GPIO,
        .rx_gpio          = SERVO_RX_GPIO,
        .baud_rate        = SERVO_BAUD_RATE,
        .address          = SERVO_ADDRESS,
        .microsteps       = SERVO_MICROSTEPS,
        .step_angle_1_8   = (SERVO_STEP_ANGLE_IS_1_8 != 0),
        .reply_timeout_ms = SERVO_REPLY_TIMEOUT_MS,
    };

    ESP_ERROR_CHECK(mks_init(&servo, &cfg));

    ESP_LOGI(TAG, "%" PRIu32 " pulses per revolution, speed code 1 = %.3f RPM",
             mks_pulses_per_rev(&servo), mks_speed_code_to_rpm(&servo, 1));

    if (probe_servo() != ESP_OK) {
        ESP_LOGE(TAG, "no reply from the servo. Check:");
        ESP_LOGE(TAG, "  - GPIO%d -> servo RX, GPIO%d <- servo TX, and a common GND",
                 SERVO_TX_GPIO, SERVO_RX_GPIO);
        ESP_LOGE(TAG, "  - servo UartBaud == %d and UartAddr == 0x%02X",
                 SERVO_BAUD_RATE, SERVO_ADDRESS);
        ESP_LOGE(TAG, "  - servo is powered from its 12-24 V supply");
        ESP_LOGE(TAG, "halting");
        return;
    }

#if SERVO_APPLY_SETTINGS_ON_BOOT
    apply_settings();
#endif

    ESP_LOGI(TAG, "initial state:");
    log_servo_state();

    ESP_LOGI(TAG, "enabling motor");
    esp_err_t err = mks_enable(&servo, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable failed: %s. Is the servo in CR_UART mode?",
                 esp_err_to_name(err));
        return;
    }

    for (uint32_t cycle = 1;; cycle++) {
        ESP_LOGI(TAG, "===== demo cycle %" PRIu32 " =====", cycle);
        run_demo_cycle();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
