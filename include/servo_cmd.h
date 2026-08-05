/*
 * Text command interface for the SERVO42C.
 *
 * The command layer knows nothing about how commands arrive or where replies
 * go: a transport supplies a cmd_sink_t and calls one of the entry points
 * below. The serial console (console_serial.c) is one such transport; a WiFi
 * TCP or HTTP handler can reuse the same parser without changes.
 *
 * Replies are line-oriented and start with "OK" or "ERR" so a remote client
 * can parse them.
 */
#pragma once

#include "mks_servo42c.h"

/* Where a command's reply text goes, plus the two optional hooks the servo
 * task uses. A transport only has to fill in `write` and `ctx`; servo_ctl
 * installs `abort_requested` itself before dispatching.
 */
typedef struct {
    /* Receives NUL-terminated chunks of reply text. */
    void (*write)(void *ctx, const char *text);
    void *ctx;

    /* Polled by long-running commands. Returning true makes them stop early
     * and hand the servo over to whatever is waiting. NULL never aborts. */
    bool (*abort_requested)(void *ctx);

    /* Written after each command completes, if non-NULL. The serial console
     * uses it for its prompt; a socket transport leaves it NULL. */
    const char *prompt;
} cmd_sink_t;

/* printf-style helper for sinks. Lines longer than 256 bytes are truncated. */
void cmd_printf(const cmd_sink_t *sink, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* Binds the command layer to an initialised servo handle. Call once. */
void servo_cmd_init(mks_t *servo);

/* Writes a one-line summary of the geometry in force - microsteps, pulses per
 * revolution, usable speed range - for transports to greet a new client with,
 * so the numbers their distances and speeds are scaled by are never a guess. */
void servo_cmd_banner(const cmd_sink_t *sink);

/* Tokenises `line` on whitespace and dispatches it. `line` is modified in
 * place, so pass a mutable buffer. Blank lines and '#' comments are ignored. */
void servo_cmd_execute_line(char *line, const cmd_sink_t *sink);

/* Dispatches an already-tokenised command. argv[0] is the command name. */
void servo_cmd_dispatch(int argc, char **argv, const cmd_sink_t *sink);

/* True for commands that must jump the queue and pre-empt a running move:
 * "stop" and "disable". Used by servo_ctl to pick a queue; does not modify
 * or consume `line`. */
bool servo_cmd_is_urgent(const char *line);
