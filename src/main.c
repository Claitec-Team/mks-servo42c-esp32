/*
 * MKS SERVO42C control firmware for ESP32 (ESP-IDF).
 *
 * Wiring (see include/servo_config.h):
 *   ESP32 GPIO32 (TX)  ->  SERVO42C Rx
 *   ESP32 GPIO33 (RX)  <-  SERVO42C Tx
 *   ESP32 GND          <-> SERVO42C G
 *
 * The servo must be set to Mode = CR_UART for the motion commands to work.
 *
 * Brings the link up, then hands control to the command console on the USB
 * serial port. Type 'help' there for the command list.
 */

#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console_serial.h"
#include "console_tcp.h"
#include "diag.h"
#include "mks_servo42c.h"
#include "servo_cmd.h"
#include "servo_config.h"
#include "servo_ctl.h"
#include "wifi_link.h"

static const char *TAG = "app";

static mks_t servo;

/* Says why we last rebooted. A brownout leaves no panic output, because it is
 * handled by an interrupt that resets the chip in software, so without this it
 * looks like a mysterious crash. */
static void log_reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        ESP_LOGI(TAG, "reset: power-on");
        break;
    case ESP_RST_SW:
        ESP_LOGI(TAG, "reset: software restart");
        break;
    case ESP_RST_PANIC:
        ESP_LOGW(TAG, "reset: panic or exception on the previous run");
        break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
        ESP_LOGW(TAG, "reset: watchdog");
        break;
    case ESP_RST_BROWNOUT:
        ESP_LOGE(TAG, "reset: BROWNOUT - the 3.3 V rail sagged, this is a power");
        ESP_LOGE(TAG, "  problem, not a firmware one. Enabling WiFi makes it far");
        ESP_LOGE(TAG, "  more likely because RF calibration draws a large spike.");
        ESP_LOGE(TAG, "  Try: a different USB port or a shorter/thicker cable, a");
        ESP_LOGE(TAG, "  bulk capacitor (470-1000 uF) across 3V3-GND at the module,");
        ESP_LOGE(TAG, "  a powered hub or 5 V supply on VIN, or lower");
        ESP_LOGE(TAG, "  SERVO_WIFI_MAX_TX_POWER in servo_config.h.");
        break;
    default:
        ESP_LOGI(TAG, "reset: reason %d", (int)esp_reset_reason());
        break;
    }
}

/* Confirms the link is alive before we accept commands. */
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

#if SERVO_APPLY_MICROSTEPS_ON_BOOT
/* Puts the servo on SERVO_MICROSTEPS and says so, loudly enough that the number
 * in use is never a guess. The servo's own MStep menu also updates, so the
 * board's screen agrees with the firmware. */
static void apply_microsteps(void)
{
    esp_err_t err = mks_set_microsteps(&servo, SERVO_MICROSTEPS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not set MStep to %d: %s",
                 SERVO_MICROSTEPS, esp_err_to_name(err));
        ESP_LOGE(TAG, "  every distance and speed will be scaled wrongly unless");
        ESP_LOGE(TAG, "  the servo is already on MStep %d", SERVO_MICROSTEPS);
        return;
    }
    ESP_LOGI(TAG, "MStep set to %d: %" PRIu32 " pulses/rev, %.2f deg/pulse",
             SERVO_MICROSTEPS, mks_pulses_per_rev(&servo),
             360.0f / (float)mks_pulses_per_rev(&servo));
    ESP_LOGI(TAG, "  speed range %.2f rpm (code 1) to %.1f rpm (code 127)",
             mks_speed_code_to_rpm(&servo, 1),
             mks_speed_code_to_rpm(&servo, 127));
    ESP_LOGI(TAG, "  the servo's MStep menu now shows %d too", SERVO_MICROSTEPS);
}
#endif

#if SERVO_APPLY_SETTINGS_ON_BOOT
/* Writes the rest of the servo's EEPROM settings. Off by default, see
 * servo_config.h. The console's 'set' commands do the same thing on demand.
 * Microsteps are handled by apply_microsteps() above. */
static void apply_settings(void)
{
    ESP_LOGI(TAG, "writing servo settings to EEPROM");

    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_motor_type(
        &servo, SERVO_STEP_ANGLE_IS_1_8 ? MKS_MOTOR_1_8_DEG : MKS_MOTOR_0_9_DEG));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_mode(&servo, MKS_MODE_CR_UART));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_current_ma(&servo, SERVO_BOOT_CURRENT_MA));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_stall_protection(&servo, true));
    ESP_ERROR_CHECK_WITHOUT_ABORT(mks_set_interpolation(&servo, true));
}
#endif

void app_main(void)
{
#if defined(RUN_DIAGNOSTICS) && RUN_DIAGNOSTICS
    /* pio run -e diag -t upload && pio device monitor -e diag */
    diag_run();
    return;
#endif

    log_reset_reason();

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
        ESP_LOGE(TAG, "  - GPIO%d -> servo Rx, GPIO%d <- servo Tx, and a common GND",
                 SERVO_TX_GPIO, SERVO_RX_GPIO);
        ESP_LOGE(TAG, "  - servo UartBaud == %d and UartAddr == 0x%02X",
                 SERVO_BAUD_RATE, SERVO_ADDRESS);
        ESP_LOGE(TAG, "  - servo is powered from its 12-24 V supply");
        ESP_LOGE(TAG, "starting the console anyway so you can probe by hand");
    } else {
        /* Only worth attempting once the link is known to work. */
#if SERVO_APPLY_MICROSTEPS_ON_BOOT
        apply_microsteps();
#endif
#if SERVO_APPLY_SETTINGS_ON_BOOT
        apply_settings();
#endif
    }

    /* A disabled motor silently ignores every move while the servo still
     * acknowledges it, so it is worth saying up front rather than leaving it to
     * be discovered by a shaft that will not turn. */
    mks_en_state_t en = MKS_EN_STATE_ERROR;
    if (mks_read_en_state(&servo, &en) == ESP_OK) {
        if (en == MKS_EN_STATE_DISABLED) {
            ESP_LOGW(TAG, "the motor is DISABLED: moves will do nothing until "
                          "you run 'enable 1'");
        } else {
            ESP_LOGI(TAG, "motor is enabled");
        }
    }

    /* The servo task takes ownership of `servo` from here on; nothing else may
     * touch it. Transports submit command lines through servo_ctl_submit(). */
    ESP_ERROR_CHECK(servo_ctl_start(&servo));
    ESP_ERROR_CHECK(console_serial_start());

    /* Remote control, if servo_config.h has a network configured. The TCP
     * listener retries until the stack is up, so ordering does not matter. */
    if (wifi_link_start() == ESP_OK) {
        ESP_ERROR_CHECK(console_tcp_start(SERVO_TCP_PORT));
    }

    /* Everything from here on happens in the servo, console and network tasks. */
}
