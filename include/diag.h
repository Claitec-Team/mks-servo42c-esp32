/*
 * Link diagnostics for bringing up the SERVO42C UART.
 *
 * Build and flash with:
 *     pio run -e diag -t upload && pio device monitor -e diag
 *
 * Runs before any driver setup and never gives up on an error, so it is
 * usable when nothing works yet.
 */
#pragma once

/* Listens, then sweeps baud rates / addresses / framing variants and dumps
 * every raw byte the servo sends back. Returns when finished. */
void diag_run(void);
