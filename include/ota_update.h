/*
 * Over-the-air firmware update receiver.
 *
 * Listens on its own TCP port and accepts a pushed firmware image, so the
 * device can be updated over WiFi without a USB cable. The counterpart is
 * `tools/servoctl <host> --ota firmware.bin`.
 *
 * Wire protocol (one connection, one update):
 *
 *   client -> "OTA <size>\n"        size = firmware byte count, decimal
 *   server -> "OK begin <slot> <size>\n"   once the target slot is erased
 *          or "ERR <reason>\n"      and the connection is closed
 *   client -> <size> raw bytes      the firmware.bin, streamed
 *   server -> "OK done <slot>, rebooting\n"   image accepted; device reboots
 *          or "ERR <reason>\n"
 *
 * The image lands in whichever OTA slot is not currently running; only on a
 * complete, valid transfer does the bootloader switch to it, and the new image
 * still has to prove itself (see the rollback logic in main.c) before it sticks.
 * A partial or corrupt push therefore cannot break the running firmware.
 *
 * There is no authentication, exactly like the command port. See the SECURITY
 * note in servo_config.h.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Starts the OTA listener task. Safe to call before WiFi has associated; the
 * listener retries until the network stack is up. Only one update runs at a
 * time; a second connection while one is in progress is refused. */
esp_err_t ota_update_start(uint16_t port);
