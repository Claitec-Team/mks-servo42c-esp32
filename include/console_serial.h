/*
 * Serial transport for the command interface.
 *
 * Reads command lines from UART0 (the USB console) and feeds them to
 * servo_cmd. A WiFi transport would do the same thing with a socket: accept
 * a line, hand it to servo_cmd_execute_line() with its own cmd_sink_t, and
 * write the reply back.
 */
#pragma once

#include "esp_err.h"

/* Starts the console task. servo_cmd_init() must have been called first. */
esp_err_t console_serial_start(void);
