/*
 * MKS SERVO42C serial (UART) driver for ESP-IDF.
 *
 * Protocol reference: "MKS SERVO42C V1.1.2 User Manual", parts 4-6, and
 * https://github.com/makerbase-mks/MKS-SERVO42C/wiki
 *
 * Frame layout
 *   host -> servo : [addr][cmd][data ...][chk]
 *   servo -> host : [addr][data ...][chk]
 *   chk = (sum of all preceding bytes of the frame) & 0xFF   ("CHECKSUM-8")
 *
 * Multi-byte values are big-endian. Addresses are 0xE0..0xE9.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Enumerations mirroring the on-screen menu options                   */
/* ------------------------------------------------------------------ */

typedef enum {
    MKS_DIR_CW  = 0,
    MKS_DIR_CCW = 1,
} mks_dir_t;

/* Menu -> Mode. Serial motion commands (F3/F6/F7/FD) require CR_UART. */
typedef enum {
    MKS_MODE_CR_OPEN = 0,
    MKS_MODE_CR_VFOC = 1,
    MKS_MODE_CR_UART = 2,
} mks_mode_t;

/* Menu -> MotType. */
typedef enum {
    MKS_MOTOR_0_9_DEG = 0,
    MKS_MOTOR_1_8_DEG = 1,
} mks_motor_type_t;

/* Menu -> En. */
typedef enum {
    MKS_EN_ACTIVE_LOW  = 0,
    MKS_EN_ACTIVE_HIGH = 1,
    MKS_EN_ACTIVE_HOLD = 2,
} mks_en_active_t;

/* Menu -> UartBaud. Argument of mks_set_baud_code(). */
typedef enum {
    MKS_BAUD_9600   = 1,
    MKS_BAUD_19200  = 2,
    MKS_BAUD_25000  = 3,
    MKS_BAUD_38400  = 4,
    MKS_BAUD_57600  = 5,
    MKS_BAUD_115200 = 6,
} mks_baud_code_t;

/* Menu -> 0_Mode. */
typedef enum {
    MKS_ZERO_MODE_DISABLE = 0,
    MKS_ZERO_MODE_DIR     = 1,
    MKS_ZERO_MODE_NEAR    = 2,
} mks_zero_mode_t;

/* Data byte returned by the FD (move by pulses) command. */
typedef enum {
    MKS_RUN_FAIL     = 0,
    MKS_RUN_STARTING = 1,
    MKS_RUN_COMPLETE = 2,
} mks_run_status_t;

/* Data byte returned by command 0x3A (read En pin state). */
typedef enum {
    MKS_EN_STATE_ERROR    = 0,
    MKS_EN_STATE_ENABLED  = 1,
    MKS_EN_STATE_DISABLED = 2,
} mks_en_state_t;

/* Data byte returned by command 0x3E (read locked-rotor protection). */
typedef enum {
    MKS_PROTECT_ERROR       = 0,
    MKS_PROTECT_TRIPPED     = 1,
    MKS_PROTECT_NOT_TRIPPED = 2,
} mks_protect_state_t;

/* ------------------------------------------------------------------ */
/* Handle                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    uart_port_t uart_num;
    int         tx_gpio;
    int         rx_gpio;
    int         baud_rate;
    uint8_t     address;         /* 0xE0 .. 0xE9 */
    uint16_t    microsteps;      /* servo's MStep setting, 1..256 */
    bool        step_angle_1_8;  /* true: 1.8 deg/step, false: 0.9 deg/step */
    uint32_t    reply_timeout_ms;
} mks_config_t;

typedef struct {
    mks_config_t cfg;
    bool         uart_owned;     /* true if this driver installed the UART */
} mks_t;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/* Configures and installs the UART. `cfg` is copied into `handle`. */
esp_err_t mks_init(mks_t *handle, const mks_config_t *cfg);

/* Releases the UART driver. */
esp_err_t mks_deinit(mks_t *handle);

/* ------------------------------------------------------------------ */
/* Unit conversion helpers (pure, no I/O)                              */
/* ------------------------------------------------------------------ */

/* Pulses for one output revolution: 200 (or 400) x microsteps. */
uint32_t mks_pulses_per_rev(const mks_t *handle);

/* Vrpm = (code x 30000) / (MStep x 200)  for a 1.8 deg motor
 * Vrpm = (code x 30000) / (MStep x 400)  for a 0.9 deg motor */
float mks_speed_code_to_rpm(const mks_t *handle, uint8_t code);

/* Inverse of the above, clamped to the usable 1..127 code range. */
uint8_t mks_rpm_to_speed_code(const mks_t *handle, float rpm);

/* ------------------------------------------------------------------ */
/* 5.1  Read parameters                                                */
/* ------------------------------------------------------------------ */

/* 0x30: raw encoder reading. Total position is carry * 65536 + value. */
esp_err_t mks_read_encoder(mks_t *h, int32_t *carry, uint16_t *value);

/* 0x33: number of pulses received by the driver. */
esp_err_t mks_read_pulses(mks_t *h, int32_t *pulses);

/* 0x36: shaft angle, 65536 counts per revolution. */
esp_err_t mks_read_angle_raw(mks_t *h, int32_t *angle);

/* 0x36 expressed in degrees. */
esp_err_t mks_read_angle_deg(mks_t *h, float *degrees);

/* 0x39: commanded minus actual angle, 65536 counts per revolution. */
esp_err_t mks_read_angle_error_raw(mks_t *h, int16_t *error);

/* 0x39 expressed in degrees. */
esp_err_t mks_read_angle_error_deg(mks_t *h, float *degrees);

/* 0x3A: En pin state. */
esp_err_t mks_read_en_state(mks_t *h, mks_en_state_t *state);

/* 0x3E: locked-rotor protection state. */
esp_err_t mks_read_protect_state(mks_t *h, mks_protect_state_t *state);

/* 0x3D: clear a tripped locked-rotor protection. */
esp_err_t mks_release_protect(mks_t *h);

/* ------------------------------------------------------------------ */
/* 5.2  Set parameters (these write the servo's EEPROM)                */
/* ------------------------------------------------------------------ */

/* 0x80: encoder calibration. The motor must be unloaded and this takes
 * several seconds, so pass a generous timeout (e.g. 20000 ms). */
esp_err_t mks_calibrate_encoder(mks_t *h, uint32_t timeout_ms);

esp_err_t mks_set_motor_type(mks_t *h, mks_motor_type_t type);       /* 0x81 */
esp_err_t mks_set_mode(mks_t *h, mks_mode_t mode);                   /* 0x82 */

/* 0x83: current in mA, rounded down to the nearest 200 mA step,
 * clamped to 0..3000 mA. */
esp_err_t mks_set_current_ma(mks_t *h, uint16_t current_ma);

/* 0x84: microsteps 1..256 (256 is sent as 0). Also updates the value used
 * by the conversion helpers. */
esp_err_t mks_set_microsteps(mks_t *h, uint16_t microsteps);

esp_err_t mks_set_en_active(mks_t *h, mks_en_active_t active);        /* 0x85 */
esp_err_t mks_set_direction(mks_t *h, mks_dir_t dir);                /* 0x86 */
esp_err_t mks_set_auto_screen_off(mks_t *h, bool enable);            /* 0x87 */
esp_err_t mks_set_stall_protection(mks_t *h, bool enable);           /* 0x88 */
esp_err_t mks_set_interpolation(mks_t *h, bool enable);              /* 0x89 */

/* 0x8A / 0x8B: the servo answers at the old setting and applies the new one
 * afterwards. Reconfigure this side with mks_reconfigure_link() to match. */
esp_err_t mks_set_baud_code(mks_t *h, mks_baud_code_t code);
esp_err_t mks_set_address(mks_t *h, uint8_t address);

/* 0x3F: restore factory defaults (this resets baud rate and address too). */
esp_err_t mks_restore_defaults(mks_t *h);

/* Re-points this side of the link at a new baud rate and/or address without
 * touching the servo. Pass 0 to keep the current value. */
esp_err_t mks_reconfigure_link(mks_t *h, int baud_rate, uint8_t address);

/* ------------------------------------------------------------------ */
/* 5.3  Zero-point (homing) parameters                                 */
/* ------------------------------------------------------------------ */

esp_err_t mks_set_zero_mode(mks_t *h, mks_zero_mode_t mode);         /* 0x90 */
esp_err_t mks_set_zero_here(mks_t *h);                               /* 0x91 */

/* 0x92: 0..4, lower is faster. */
esp_err_t mks_set_zero_speed(mks_t *h, uint8_t speed);

esp_err_t mks_set_zero_direction(mks_t *h, mks_dir_t dir);           /* 0x93 */

/* 0x94: start homing. */
esp_err_t mks_goto_zero(mks_t *h, uint32_t timeout_ms);

/* ------------------------------------------------------------------ */
/* 5.4  PID / acceleration / torque                                    */
/* ------------------------------------------------------------------ */

esp_err_t mks_set_kp(mks_t *h, uint16_t kp);                         /* 0xA1, default 0x650 */
esp_err_t mks_set_ki(mks_t *h, uint16_t ki);                         /* 0xA2, default 1 */
esp_err_t mks_set_kd(mks_t *h, uint16_t kd);                         /* 0xA3, default 0x650 */

/* 0xA4, default 0x11E. The manual warns that an excessive value can damage
 * the driver board. */
esp_err_t mks_set_acceleration(mks_t *h, uint16_t acc);

/* 0xA5: 0..0x4B0, default 0x4B0. */
esp_err_t mks_set_max_torque(mks_t *h, uint16_t max_torque);

/* ------------------------------------------------------------------ */
/* 5.5  Motion commands - all require Mode == CR_UART                  */
/* ------------------------------------------------------------------ */

/* 0xF3: energise / de-energise the motor. */
esp_err_t mks_enable(mks_t *h, bool enable);

/* 0xF6: run continuously. `speed_code` is 1..127; 0 idles the motor. */
esp_err_t mks_run_constant_speed(mks_t *h, mks_dir_t dir, uint8_t speed_code);

/* 0xF6 with the speed expressed in RPM. */
esp_err_t mks_run_rpm(mks_t *h, mks_dir_t dir, float rpm);

/* 0xF7: stop. */
esp_err_t mks_stop(mks_t *h);

/* 0xFF: store (true) or clear (false) the current F6 state as the
 * power-on behaviour. After a successful save the driver disables itself
 * and must be re-enabled. */
esp_err_t mks_save_speed_state(mks_t *h, bool save);

/* 0xFD: move a fixed number of pulses.
 *
 * The servo replies twice: status 1 when the move starts and status 2 when
 * it finishes. If `wait_complete` is true this blocks for the second reply
 * up to `move_timeout_ms`; otherwise it returns after the first and the
 * caller must consume the completion with mks_wait_move_complete().
 */
esp_err_t mks_move_pulses(mks_t *h, mks_dir_t dir, uint8_t speed_code,
                          uint32_t pulses, bool wait_complete,
                          uint32_t move_timeout_ms);

/* Waits for the trailing "run complete" reply of a non-blocking
 * mks_move_pulses(). Returns ESP_ERR_TIMEOUT if it does not arrive. */
esp_err_t mks_wait_move_complete(mks_t *h, uint32_t timeout_ms);

/* mks_move_pulses() with the distance given in degrees and the speed in RPM.
 * A negative `degrees` moves counter-clockwise. */
esp_err_t mks_move_degrees(mks_t *h, float degrees, float rpm,
                           bool wait_complete, uint32_t move_timeout_ms);

/* Convenience wrapper around mks_move_degrees(). */
esp_err_t mks_move_revolutions(mks_t *h, float revolutions, float rpm,
                               bool wait_complete, uint32_t move_timeout_ms);

/* ------------------------------------------------------------------ */
/* Escape hatch for commands not wrapped above                          */
/* ------------------------------------------------------------------ */

/* Sends [addr][cmd][tx_data..][chk] and reads [addr][rx_data..][chk].
 * `rx_len` is the number of data bytes expected, excluding address and
 * checksum. Fails with ESP_ERR_INVALID_CRC on a checksum mismatch. */
esp_err_t mks_transfer(mks_t *h, uint8_t cmd,
                       const uint8_t *tx_data, size_t tx_len,
                       uint8_t *rx_data, size_t rx_len,
                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
