/*
 * Board / wiring configuration for the MKS SERVO42C link.
 *
 * Everything you may need to change for your setup lives in this file.
 */
#pragma once

/* ------------------------------------------------------------------ */
/* Servo UART port and pins                                            */
/* ------------------------------------------------------------------ */
/*
 * UART2 is used so that UART0 stays free for flashing and the log console
 * over USB.
 *
 *   ESP32 GPIO17 (TX)  ->  SERVO42C RX
 *   ESP32 GPIO16 (RX)  <-  SERVO42C TX
 *   ESP32 GND          <-> SERVO42C GND     (required, common ground)
 */
#define SERVO_UART_PORT   2
#define SERVO_TX_GPIO     17  /* ESP32 TX -> servo RX */
#define SERVO_RX_GPIO     16  /* ESP32 RX <- servo TX */

/* Menu -> UartBaud on the servo. Must match exactly. */
#define SERVO_BAUD_RATE   19200

/* Menu -> UartAddr on the servo. Valid range 0xE0 .. 0xE9. */
#define SERVO_ADDRESS     0xE0

/* ------------------------------------------------------------------ */
/* Motor characteristics (must match the servo's own settings)         */
/* ------------------------------------------------------------------ */

/* Menu -> MStep. Used to convert degrees <-> pulses and speed <-> RPM. */
#define SERVO_MICROSTEPS  16

/* 1 for a 1.8 deg/step motor (200 full steps/rev, the usual NEMA17),
 * 0 for a 0.9 deg/step motor (400 full steps/rev). */
#define SERVO_STEP_ANGLE_IS_1_8  1

/* How long to wait for the 3-byte acknowledge of a command. */
#define SERVO_REPLY_TIMEOUT_MS   300

/* ------------------------------------------------------------------ */
/* Demo behaviour (src/main.c)                                         */
/* ------------------------------------------------------------------ */

/* Set to 1 to push mode/current/microstep settings into the servo's
 * EEPROM on every boot. Leave at 0 and configure via the on-board screen
 * instead: writing flash on each boot wears it out needlessly. */
#define SERVO_APPLY_SETTINGS_ON_BOOT  0

/* Only used when SERVO_APPLY_SETTINGS_ON_BOOT is 1. */
#define SERVO_BOOT_CURRENT_MA         800

/* Speed used by the demo moves, in RPM at the motor shaft. */
#define DEMO_RPM                      60.0f

/* Revolutions per demo positioning move. */
#define DEMO_REVOLUTIONS              1.0f
