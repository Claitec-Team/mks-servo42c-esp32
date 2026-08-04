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

/* Where a command's reply text goes. `write` receives NUL-terminated chunks. */
typedef struct {
    void (*write)(void *ctx, const char *text);
    void *ctx;
} cmd_sink_t;

/* printf-style helper for sinks. Lines longer than 256 bytes are truncated. */
void cmd_printf(const cmd_sink_t *sink, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* Binds the command layer to an initialised servo handle. Call once. */
void servo_cmd_init(mks_t *servo);

/* Tokenises `line` on whitespace and dispatches it. `line` is modified in
 * place, so pass a mutable buffer. Blank lines and '#' comments are ignored. */
void servo_cmd_execute_line(char *line, const cmd_sink_t *sink);

/* Dispatches an already-tokenised command. argv[0] is the command name. */
void servo_cmd_dispatch(int argc, char **argv, const cmd_sink_t *sink);
