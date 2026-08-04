/*
 * TCP transport for the command interface.
 *
 * One line in, one or more reply lines out, exactly as on the serial console:
 *
 *     nc mks-servo42c.local 3333
 *     enable 1
 *     OK enable
 *     rev 20 10
 *     .. moving 20.000 rev at 10.0 rpm
 *     stop
 *     OK move aborted
 *     OK stop
 *
 * Several clients may connect at once, and the serial console keeps working
 * alongside them: every transport funnels through servo_ctl, so commands are
 * serialised and a stop from any of them pre-empts a move started by another.
 *
 * There is no authentication. See the warning in servo_config.h.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

/* Starts the listener task. Safe to call before WiFi has associated. */
esp_err_t console_tcp_start(uint16_t port);

/* Number of connected clients. */
unsigned console_tcp_clients(void);
