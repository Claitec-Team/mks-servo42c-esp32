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
 * Any GPIO works: the ESP32 routes UART signals through its GPIO matrix, so
 * changing these two defines is all that a pin move requires.
 *
 *   ESP32 GPIO19 (TX)  ->  SERVO42C Rx
 *   ESP32 GPIO18 (RX)  <-  SERVO42C Tx
 *   ESP32 GND          <-> SERVO42C G       (required, common ground)
 */
#define SERVO_UART_PORT   2
#define SERVO_TX_GPIO     32  /* ESP32 TX -> servo Rx (servo's input) */
#define SERVO_RX_GPIO     33  /* ESP32 RX <- servo Tx (servo's output) */

/* Menu -> UartBaud on the servo. Must match exactly. */
#define SERVO_BAUD_RATE   38400

/* Menu -> UartAddr on the servo. Valid range 0xE0 .. 0xE9. */
#define SERVO_ADDRESS     0xE0

/* ------------------------------------------------------------------ */
/* Motor characteristics (must match the servo's own settings)         */
/* ------------------------------------------------------------------ */

/*
 * Menu -> MStep. Used to convert degrees <-> pulses and speed <-> RPM.
 *
 * Higher means slower and finer, because speed is a 7-bit code, not an RPM
 * figure: Vrpm = code * 30000 / (MStep * 200). At MStep 16 the slowest possible
 * speed is code 1 = 9.375 rpm; at 128 it is 1.17 rpm, with a 148.8 rpm ceiling.
 * The screen's MStep menu only offers powers of two, 1 to 256.
 */
#define SERVO_MICROSTEPS  128

/*
 * Push SERVO_MICROSTEPS into the servo during startup, so it always comes up in
 * a known state even if the screen buttons were used or the board was swapped.
 *
 * The servo has no command to read MStep back, so this cannot be a
 * write-only-if-it-differs: it is one flash write per boot on the servo. Set to
 * 0 to leave whatever the servo already has, in which case SERVO_MICROSTEPS must
 * be edited to match or every distance and speed will be scaled wrongly.
 */
#define SERVO_APPLY_MICROSTEPS_ON_BOOT  1

/* 1 for a 1.8 deg/step motor (200 full steps/rev, the usual NEMA17),
 * 0 for a 0.9 deg/step motor (400 full steps/rev). */
#define SERVO_STEP_ANGLE_IS_1_8  1

/* How long to wait for the 3-byte acknowledge of a command. */
#define SERVO_REPLY_TIMEOUT_MS   300

/*
 * Which way the reported shaft angle counts: 1 if turning clockwise makes
 * 'read angle' increase, 0 if it makes it decrease.
 *
 * This depends on the servo's Dir setting and on the motor's coil wiring, so it
 * cannot be known in advance. It only affects 'goto', which has to decide which
 * way to turn to reach an absolute angle. Get it wrong and goto moves away from
 * its target instead of towards it - it says so and tells you to flip this,
 * and because each goto is one bounded move it cannot run away.
 */
#define SERVO_ANGLE_INCREASES_CW  1

/* ------------------------------------------------------------------ */
/* WiFi remote control                                                 */
/* ------------------------------------------------------------------ */
/*
 * Credentials come from the environment at build time, so they are never
 * committed. platformio.ini maps them:
 *
 *     export WIFI_CLAITEC_SSID='your-ssid'
 *     export WIFI_CLAITEC_PASS='your-password'
 *     pio run -t upload
 *
 * With either variable unset the macro below stays empty, the radio is never
 * started, and the firmware behaves exactly as it did without WiFi.
 *
 * SECURITY: the command port is unauthenticated and unencrypted. Anyone who
 * can reach this port can drive the motor. Use it on a network you trust, and
 * do not forward the port through a router.
 */
/* Written by scripts/wifi_credentials.py before each build. Guarded so that an
 * editor or a checkout that has not been built yet still parses this header. */
#if defined(__has_include)
#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#endif
#endif

#ifndef SERVO_WIFI_SSID
#define SERVO_WIFI_SSID      ""
#endif
#ifndef SERVO_WIFI_PASSWORD
#define SERVO_WIFI_PASSWORD  ""
#endif

/* Advertised over DHCP, so you can use it instead of hunting for the IP. */
#define SERVO_WIFI_HOSTNAME  "mks-servo42c"

/*
 * Transmit power cap, in quarter-dBm: 8 = 2 dBm, 52 = 13 dBm, 84 = 20 dBm.
 * 0 leaves the driver default (20 dBm, the maximum).
 *
 * Turning the radio on draws a large current spike, and on a board powered
 * through a thin USB cable that can sag the 3.3 V rail far enough to brown out
 * the chip - which shows up as a reset during PHY calibration, before WiFi ever
 * associates. Capping the power shrinks the spike; 52 (13 dBm) is still plenty
 * for a local network. The real fix is a better supply: see the brownout
 * message the firmware prints on the next boot.
 */
#define SERVO_WIFI_MAX_TX_POWER  0

/* TCP port for the command interface: `nc <ip> 3333`. */
#define SERVO_TCP_PORT       3333

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
/* Speed used when a move/rev command omits one, and by the demo. */
#define SERVO_DEFAULT_RPM             30.0f

/* Revolutions per demo positioning move. */
#define DEMO_REVOLUTIONS              1.0f
