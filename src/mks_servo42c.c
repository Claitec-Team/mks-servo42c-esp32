#include "mks_servo42c.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mks42c";

/* Command codes, in manual order. */
#define CMD_READ_ENCODER      0x30
#define CMD_READ_PULSES       0x33
#define CMD_READ_ANGLE        0x36
#define CMD_READ_ANGLE_ERROR  0x39
#define CMD_READ_EN           0x3A
#define CMD_RELEASE_PROTECT   0x3D
#define CMD_READ_PROTECT      0x3E
#define CMD_RESTORE_DEFAULTS  0x3F
#define CMD_CALIBRATE         0x80
#define CMD_SET_MOTOR_TYPE    0x81
#define CMD_SET_MODE          0x82
#define CMD_SET_CURRENT       0x83
#define CMD_SET_MICROSTEP     0x84
#define CMD_SET_EN_ACTIVE     0x85
#define CMD_SET_DIRECTION     0x86
#define CMD_SET_AUTO_SDD      0x87
#define CMD_SET_PROTECT       0x88
#define CMD_SET_INTERPOLATION 0x89
#define CMD_SET_BAUD          0x8A
#define CMD_SET_ADDRESS       0x8B
#define CMD_SET_ZERO_MODE     0x90
#define CMD_SET_ZERO_HERE     0x91
#define CMD_SET_ZERO_SPEED    0x92
#define CMD_SET_ZERO_DIR      0x93
#define CMD_GOTO_ZERO         0x94
#define CMD_SET_KP            0xA1
#define CMD_SET_KI            0xA2
#define CMD_SET_KD            0xA3
#define CMD_SET_ACC           0xA4
#define CMD_SET_MAX_TORQUE    0xA5
#define CMD_SET_EN            0xF3
#define CMD_RUN_SPEED         0xF6
#define CMD_STOP              0xF7
#define CMD_MOVE_PULSES       0xFD
#define CMD_SAVE_STATE        0xFF

/* Largest frame we ever build: addr + cmd + 4 data bytes + checksum. */
#define MKS_TX_MAX 8
/* Largest reply: addr + 6 data bytes (0x30) + checksum. */
#define MKS_RX_MAX 8

#define MKS_UART_RX_BUF 256

/* A status byte of 1 means "accepted"; every set/control command uses it. */
#define MKS_STATUS_OK 1

/* ------------------------------------------------------------------ */
/* Framing                                                             */
/* ------------------------------------------------------------------ */

static uint8_t mks_checksum(const uint8_t *bytes, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += bytes[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* Reads exactly `len` bytes or fails with ESP_ERR_TIMEOUT. */
static esp_err_t read_exact(mks_t *h, uint8_t *dst, size_t len, uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    size_t got = 0;

    while (got < len) {
        int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        TickType_t ticks = pdMS_TO_TICKS((uint32_t)(left_us / 1000) + 1);
        if (ticks == 0) {
            ticks = 1;
        }
        int n = uart_read_bytes(h->cfg.uart_num, dst + got, len - got, ticks);
        if (n < 0) {
            return ESP_FAIL;
        }
        got += (size_t)n;
    }
    return ESP_OK;
}

/* Reads one reply frame: [addr][rx_len data bytes][checksum].
 * Leading bytes that are not our address are discarded, which lets the link
 * recover from boot noise on a shared UART. */
static esp_err_t read_reply(mks_t *h, uint8_t *rx_data, size_t rx_len, uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    uint8_t frame[MKS_RX_MAX];

    for (;;) {
        int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        uint32_t left_ms = (uint32_t)(left_us / 1000) + 1;

        esp_err_t err = read_exact(h, &frame[0], 1, left_ms);
        if (err != ESP_OK) {
            return err;
        }
        if (frame[0] != h->cfg.address) {
            ESP_LOGD(TAG, "discarding stray byte 0x%02X", frame[0]);
            continue;
        }

        left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        /* Once the address byte has arrived the rest of the frame follows
         * back-to-back, so allow at least a short grace period for it. */
        left_ms = (uint32_t)(left_us / 1000) + 1;
        if (left_ms < 50) {
            left_ms = 50;
        }
        err = read_exact(h, &frame[1], rx_len + 1, left_ms);
        if (err != ESP_OK) {
            return err;
        }

        uint8_t expected = mks_checksum(frame, rx_len + 1);
        if (frame[rx_len + 1] != expected) {
            ESP_LOGW(TAG, "checksum mismatch: got 0x%02X, expected 0x%02X",
                     frame[rx_len + 1], expected);
            return ESP_ERR_INVALID_CRC;
        }
        if (rx_data != NULL && rx_len > 0) {
            memcpy(rx_data, &frame[1], rx_len);
        }
        return ESP_OK;
    }
}

esp_err_t mks_transfer(mks_t *h, uint8_t cmd,
                       const uint8_t *tx_data, size_t tx_len,
                       uint8_t *rx_data, size_t rx_len,
                       uint32_t timeout_ms)
{
    if (h == NULL || !h->uart_owned) {
        return ESP_ERR_INVALID_STATE;
    }
    if (tx_len + 3 > MKS_TX_MAX || rx_len + 2 > MKS_RX_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[MKS_TX_MAX];
    size_t n = 0;
    frame[n++] = h->cfg.address;
    frame[n++] = cmd;
    for (size_t i = 0; i < tx_len; i++) {
        frame[n++] = tx_data[i];
    }
    frame[n] = mks_checksum(frame, n);
    n++;

    /* Drop anything stale so the next byte we read belongs to this reply. */
    uart_flush_input(h->cfg.uart_num);

    int written = uart_write_bytes(h->cfg.uart_num, frame, n);
    if (written != (int)n) {
        ESP_LOGE(TAG, "uart_write_bytes returned %d for cmd 0x%02X", written, cmd);
        return ESP_FAIL;
    }
    esp_err_t err = uart_wait_tx_done(h->cfg.uart_num, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    err = read_reply(h, rx_data, rx_len, timeout_ms);
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "no reply to cmd 0x%02X within %" PRIu32 " ms", cmd, timeout_ms);
    }
    return err;
}

/* Sends a command whose reply is a single status byte and maps
 * status != 1 to ESP_FAIL. */
static esp_err_t mks_command_status(mks_t *h, uint8_t cmd,
                                   const uint8_t *tx_data, size_t tx_len,
                                   uint32_t timeout_ms)
{
    uint8_t status = 0;
    esp_err_t err = mks_transfer(h, cmd, tx_data, tx_len, &status, 1, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (status != MKS_STATUS_OK) {
        ESP_LOGE(TAG, "cmd 0x%02X rejected (status %u)", cmd, status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Same, for the many commands that take exactly one data byte. */
static esp_err_t mks_command_u8(mks_t *h, uint8_t cmd, uint8_t value)
{
    return mks_command_status(h, cmd, &value, 1, h->cfg.reply_timeout_ms);
}

/* Same, for the PID/ACC/torque commands that take a big-endian uint16. */
static esp_err_t mks_command_u16(mks_t *h, uint8_t cmd, uint16_t value)
{
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return mks_command_status(h, cmd, data, sizeof(data), h->cfg.reply_timeout_ms);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

esp_err_t mks_init(mks_t *handle, const mks_config_t *cfg)
{
    if (handle == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->address < 0xE0 || cfg->address > 0xE9) {
        ESP_LOGE(TAG, "address 0x%02X is outside the valid 0xE0..0xE9 range", cfg->address);
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->microsteps < 1 || cfg->microsteps > 256) {
        ESP_LOGE(TAG, "microsteps %u is outside the valid 1..256 range", cfg->microsteps);
        return ESP_ERR_INVALID_ARG;
    }

    handle->cfg = *cfg;
    handle->uart_owned = false;

    const uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg->uart_num, MKS_UART_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->uart_num);
        return err;
    }
    err = uart_set_pin(cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        uart_driver_delete(cfg->uart_num);
        return err;
    }

    handle->uart_owned = true;
    uart_flush_input(cfg->uart_num);

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d RX=GPIO%d %d baud, servo addr 0x%02X",
             (int)cfg->uart_num, cfg->tx_gpio, cfg->rx_gpio, cfg->baud_rate, cfg->address);
    return ESP_OK;
}

esp_err_t mks_deinit(mks_t *handle)
{
    if (handle == NULL || !handle->uart_owned) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = uart_driver_delete(handle->cfg.uart_num);
    handle->uart_owned = false;
    return err;
}

esp_err_t mks_reconfigure_link(mks_t *h, int baud_rate, uint8_t address)
{
    if (h == NULL || !h->uart_owned) {
        return ESP_ERR_INVALID_STATE;
    }
    if (address != 0 && (address < 0xE0 || address > 0xE9)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (baud_rate > 0) {
        esp_err_t err = uart_set_baudrate(h->cfg.uart_num, baud_rate);
        if (err != ESP_OK) {
            return err;
        }
        h->cfg.baud_rate = baud_rate;
    }
    if (address != 0) {
        h->cfg.address = address;
    }
    uart_flush_input(h->cfg.uart_num);
    ESP_LOGI(TAG, "link now %d baud, addr 0x%02X", h->cfg.baud_rate, h->cfg.address);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Unit conversion                                                     */
/* ------------------------------------------------------------------ */

uint32_t mks_pulses_per_rev(const mks_t *handle)
{
    uint32_t full_steps = handle->cfg.step_angle_1_8 ? 200u : 400u;
    return full_steps * (uint32_t)handle->cfg.microsteps;
}

float mks_speed_code_to_rpm(const mks_t *handle, uint8_t code)
{
    /* Vrpm = code * 30000 / pulses_per_rev */
    return ((float)(code & 0x7F) * 30000.0f) / (float)mks_pulses_per_rev(handle);
}

uint8_t mks_rpm_to_speed_code(const mks_t *handle, float rpm)
{
    if (rpm < 0.0f) {
        rpm = -rpm;
    }
    float code = (rpm * (float)mks_pulses_per_rev(handle)) / 30000.0f;
    long rounded = lroundf(code);
    if (rounded < 1) {
        rounded = 1;
    }
    if (rounded > 127) {
        rounded = 127;
    }
    return (uint8_t)rounded;
}

uint32_t mks_degrees_to_pulses(const mks_t *handle, float degrees)
{
    if (degrees < 0.0f) {
        degrees = -degrees;
    }
    float pulses = degrees / 360.0f * (float)mks_pulses_per_rev(handle);
    long rounded = lroundf(pulses);
    return (rounded < 0) ? 0u : (uint32_t)rounded;
}

esp_err_t mks_discard_pending(mks_t *h, uint32_t quiet_ms)
{
    if (h == NULL || !h->uart_owned) {
        return ESP_ERR_INVALID_STATE;
    }
    if (quiet_ms == 0) {
        quiet_ms = 1;
    }

    /* Stop after a few quiet windows' worth of chatter so a servo that talks
     * continuously cannot pin us here forever. */
    const int64_t give_up_at = esp_timer_get_time() + (int64_t)quiet_ms * 1000 * 5;
    uint8_t scratch[16];
    size_t dropped = 0;

    for (;;) {
        int n = uart_read_bytes(h->cfg.uart_num, scratch, sizeof(scratch),
                               pdMS_TO_TICKS(quiet_ms));
        if (n <= 0) {
            break;                       /* silent for a full window: done */
        }
        dropped += (size_t)n;
        if (esp_timer_get_time() > give_up_at) {
            break;
        }
    }
    uart_flush_input(h->cfg.uart_num);
    if (dropped > 0) {
        ESP_LOGD(TAG, "discarded %u stale byte(s)", (unsigned)dropped);
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 5.1  Read parameters                                                */
/* ------------------------------------------------------------------ */

esp_err_t mks_read_encoder(mks_t *h, int32_t *carry, uint16_t *value)
{
    uint8_t data[6];
    esp_err_t err = mks_transfer(h, CMD_READ_ENCODER, NULL, 0, data, sizeof(data),
                                 h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (carry != NULL) {
        *carry = (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                           ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
    }
    if (value != NULL) {
        *value = (uint16_t)(((uint16_t)data[4] << 8) | data[5]);
    }
    return ESP_OK;
}

esp_err_t mks_read_pulses(mks_t *h, int32_t *pulses)
{
    uint8_t data[4];
    esp_err_t err = mks_transfer(h, CMD_READ_PULSES, NULL, 0, data, sizeof(data),
                                 h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (pulses != NULL) {
        *pulses = (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                            ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
    }
    return ESP_OK;
}

esp_err_t mks_read_angle_raw(mks_t *h, int32_t *angle)
{
    uint8_t data[4];
    esp_err_t err = mks_transfer(h, CMD_READ_ANGLE, NULL, 0, data, sizeof(data),
                                 h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (angle != NULL) {
        *angle = (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                           ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
    }
    return ESP_OK;
}

esp_err_t mks_read_angle_deg(mks_t *h, float *degrees)
{
    int32_t raw = 0;
    esp_err_t err = mks_read_angle_raw(h, &raw);
    if (err != ESP_OK) {
        return err;
    }
    if (degrees != NULL) {
        *degrees = (float)raw * 360.0f / 65536.0f;
    }
    return ESP_OK;
}

esp_err_t mks_read_angle_error_raw(mks_t *h, int16_t *error)
{
    uint8_t data[2];
    esp_err_t err = mks_transfer(h, CMD_READ_ANGLE_ERROR, NULL, 0, data, sizeof(data),
                                 h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (error != NULL) {
        *error = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    }
    return ESP_OK;
}

esp_err_t mks_read_angle_error_deg(mks_t *h, float *degrees)
{
    int16_t raw = 0;
    esp_err_t err = mks_read_angle_error_raw(h, &raw);
    if (err != ESP_OK) {
        return err;
    }
    if (degrees != NULL) {
        *degrees = (float)raw * 360.0f / 65536.0f;
    }
    return ESP_OK;
}

esp_err_t mks_read_en_state(mks_t *h, mks_en_state_t *state)
{
    uint8_t data = 0;
    esp_err_t err = mks_transfer(h, CMD_READ_EN, NULL, 0, &data, 1, h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (state != NULL) {
        *state = (mks_en_state_t)data;
    }
    return ESP_OK;
}

esp_err_t mks_read_protect_state(mks_t *h, mks_protect_state_t *state)
{
    uint8_t data = 0;
    esp_err_t err = mks_transfer(h, CMD_READ_PROTECT, NULL, 0, &data, 1, h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (state != NULL) {
        *state = (mks_protect_state_t)data;
    }
    return ESP_OK;
}

esp_err_t mks_release_protect(mks_t *h)
{
    return mks_command_status(h, CMD_RELEASE_PROTECT, NULL, 0, h->cfg.reply_timeout_ms);
}

/* ------------------------------------------------------------------ */
/* 5.2  Set parameters                                                 */
/* ------------------------------------------------------------------ */

esp_err_t mks_calibrate_encoder(mks_t *h, uint32_t timeout_ms)
{
    uint8_t data = 0x00;
    return mks_command_status(h, CMD_CALIBRATE, &data, 1, timeout_ms);
}

esp_err_t mks_set_motor_type(mks_t *h, mks_motor_type_t type)
{
    esp_err_t err = mks_command_u8(h, CMD_SET_MOTOR_TYPE, (uint8_t)type);
    if (err == ESP_OK) {
        h->cfg.step_angle_1_8 = (type == MKS_MOTOR_1_8_DEG);
    }
    return err;
}

esp_err_t mks_set_mode(mks_t *h, mks_mode_t mode)
{
    return mks_command_u8(h, CMD_SET_MODE, (uint8_t)mode);
}

esp_err_t mks_set_current_ma(mks_t *h, uint16_t current_ma)
{
    if (current_ma > 3000) {
        current_ma = 3000;
    }
    return mks_command_u8(h, CMD_SET_CURRENT, (uint8_t)(current_ma / 200));
}

esp_err_t mks_set_microsteps(mks_t *h, uint16_t microsteps)
{
    if (microsteps < 1 || microsteps > 256) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 256 is encoded as 0. */
    esp_err_t err = mks_command_u8(h, CMD_SET_MICROSTEP, (uint8_t)(microsteps & 0xFF));
    if (err == ESP_OK) {
        h->cfg.microsteps = microsteps;
    }
    return err;
}

esp_err_t mks_set_en_active(mks_t *h, mks_en_active_t active)
{
    return mks_command_u8(h, CMD_SET_EN_ACTIVE, (uint8_t)active);
}

esp_err_t mks_set_direction(mks_t *h, mks_dir_t dir)
{
    return mks_command_u8(h, CMD_SET_DIRECTION, (uint8_t)dir);
}

esp_err_t mks_set_auto_screen_off(mks_t *h, bool enable)
{
    return mks_command_u8(h, CMD_SET_AUTO_SDD, enable ? 1 : 0);
}

esp_err_t mks_set_stall_protection(mks_t *h, bool enable)
{
    return mks_command_u8(h, CMD_SET_PROTECT, enable ? 1 : 0);
}

esp_err_t mks_set_interpolation(mks_t *h, bool enable)
{
    return mks_command_u8(h, CMD_SET_INTERPOLATION, enable ? 1 : 0);
}

esp_err_t mks_set_baud_code(mks_t *h, mks_baud_code_t code)
{
    return mks_command_u8(h, CMD_SET_BAUD, (uint8_t)code);
}

esp_err_t mks_set_address(mks_t *h, uint8_t address)
{
    if (address < 0xE0 || address > 0xE9) {
        return ESP_ERR_INVALID_ARG;
    }
    /* The command carries the offset from 0xE0, not the address itself. */
    return mks_command_u8(h, CMD_SET_ADDRESS, (uint8_t)(address - 0xE0));
}

esp_err_t mks_restore_defaults(mks_t *h)
{
    return mks_command_status(h, CMD_RESTORE_DEFAULTS, NULL, 0, h->cfg.reply_timeout_ms);
}

/* ------------------------------------------------------------------ */
/* 5.3  Zero point                                                     */
/* ------------------------------------------------------------------ */

esp_err_t mks_set_zero_mode(mks_t *h, mks_zero_mode_t mode)
{
    return mks_command_u8(h, CMD_SET_ZERO_MODE, (uint8_t)mode);
}

esp_err_t mks_set_zero_here(mks_t *h, uint32_t timeout_ms)
{
    uint8_t data = 0x00;
    return mks_command_status(h, CMD_SET_ZERO_HERE, &data, 1, timeout_ms);
}

esp_err_t mks_set_zero_speed(mks_t *h, uint8_t speed)
{
    if (speed > 4) {
        return ESP_ERR_INVALID_ARG;
    }
    return mks_command_u8(h, CMD_SET_ZERO_SPEED, speed);
}

esp_err_t mks_set_zero_direction(mks_t *h, mks_dir_t dir)
{
    return mks_command_u8(h, CMD_SET_ZERO_DIR, (uint8_t)dir);
}

esp_err_t mks_goto_zero(mks_t *h, uint32_t timeout_ms)
{
    uint8_t data = 0x00;
    return mks_command_status(h, CMD_GOTO_ZERO, &data, 1, timeout_ms);
}

/* ------------------------------------------------------------------ */
/* 5.4  PID / ACC / torque                                             */
/* ------------------------------------------------------------------ */

esp_err_t mks_set_kp(mks_t *h, uint16_t kp)
{
    return mks_command_u16(h, CMD_SET_KP, kp);
}

esp_err_t mks_set_ki(mks_t *h, uint16_t ki)
{
    return mks_command_u16(h, CMD_SET_KI, ki);
}

esp_err_t mks_set_kd(mks_t *h, uint16_t kd)
{
    return mks_command_u16(h, CMD_SET_KD, kd);
}

esp_err_t mks_set_acceleration(mks_t *h, uint16_t acc)
{
    return mks_command_u16(h, CMD_SET_ACC, acc);
}

esp_err_t mks_set_max_torque(mks_t *h, uint16_t max_torque)
{
    if (max_torque > 0x4B0) {
        return ESP_ERR_INVALID_ARG;
    }
    return mks_command_u16(h, CMD_SET_MAX_TORQUE, max_torque);
}

/* ------------------------------------------------------------------ */
/* 5.5  Motion                                                         */
/* ------------------------------------------------------------------ */

esp_err_t mks_enable(mks_t *h, bool enable)
{
    return mks_command_u8(h, CMD_SET_EN, enable ? 1 : 0);
}

esp_err_t mks_run_constant_speed(mks_t *h, mks_dir_t dir, uint8_t speed_code)
{
    /* bit7 = direction, bits 6..0 = speed */
    uint8_t val = (uint8_t)((speed_code & 0x7F) | (dir == MKS_DIR_CCW ? 0x80 : 0x00));
    return mks_command_u8(h, CMD_RUN_SPEED, val);
}

esp_err_t mks_run_rpm(mks_t *h, mks_dir_t dir, float rpm)
{
    return mks_run_constant_speed(h, dir, mks_rpm_to_speed_code(h, rpm));
}

esp_err_t mks_stop(mks_t *h)
{
    return mks_command_status(h, CMD_STOP, NULL, 0, h->cfg.reply_timeout_ms);
}

esp_err_t mks_save_speed_state(mks_t *h, bool save)
{
    return mks_command_u8(h, CMD_SAVE_STATE, save ? 0xC8 : 0xCA);
}

esp_err_t mks_wait_move_complete(mks_t *h, uint32_t timeout_ms)
{
    uint8_t status = 0;
    esp_err_t err = read_reply(h, &status, 1, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (status != MKS_RUN_COMPLETE) {
        ESP_LOGE(TAG, "move ended with status %u instead of complete", status);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mks_move_pulses(mks_t *h, mks_dir_t dir, uint8_t speed_code,
                          uint32_t pulses, bool wait_complete,
                          uint32_t move_timeout_ms)
{
    mks_run_status_t status = MKS_RUN_FAIL;
    esp_err_t err = mks_start_move_pulses(h, dir, speed_code, pulses, &status);
    if (err != ESP_OK) {
        return err;
    }
    /* Nothing more will arrive if the servo already reported completion. */
    if (status == MKS_RUN_COMPLETE || !wait_complete) {
        return ESP_OK;
    }
    return mks_wait_move_complete(h, move_timeout_ms);
}

esp_err_t mks_start_move_pulses(mks_t *h, mks_dir_t dir, uint8_t speed_code,
                                uint32_t pulses, mks_run_status_t *status_out)
{
    uint8_t data[5];
    data[0] = (uint8_t)((speed_code & 0x7F) | (dir == MKS_DIR_CCW ? 0x80 : 0x00));
    data[1] = (uint8_t)(pulses >> 24);
    data[2] = (uint8_t)(pulses >> 16);
    data[3] = (uint8_t)(pulses >> 8);
    data[4] = (uint8_t)(pulses);

    uint8_t status = 0;
    esp_err_t err = mks_transfer(h, CMD_MOVE_PULSES, data, sizeof(data), &status, 1,
                                 h->cfg.reply_timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (status_out != NULL) {
        *status_out = (mks_run_status_t)status;
    }
    if (status == MKS_RUN_FAIL) {
        ESP_LOGE(TAG, "move rejected: is the servo in CR_UART mode and enabled?");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mks_move_degrees(mks_t *h, float degrees, float rpm,
                           bool wait_complete, uint32_t move_timeout_ms)
{
    mks_dir_t dir = (degrees < 0.0f) ? MKS_DIR_CCW : MKS_DIR_CW;

    return mks_move_pulses(h, dir, mks_rpm_to_speed_code(h, rpm),
                           mks_degrees_to_pulses(h, degrees),
                           wait_complete, move_timeout_ms);
}

esp_err_t mks_move_revolutions(mks_t *h, float revolutions, float rpm,
                               bool wait_complete, uint32_t move_timeout_ms)
{
    return mks_move_degrees(h, revolutions * 360.0f, rpm, wait_complete, move_timeout_ms);
}
