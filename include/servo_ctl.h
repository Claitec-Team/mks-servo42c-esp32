/*
 * Serialised access to the servo.
 *
 * mks_t is not thread-safe: one request/reply exchange has to finish before the
 * next begins, or two callers read each other's replies. Every transport
 * therefore submits command lines here instead of calling servo_cmd directly,
 * and a single task owns the servo handle and executes them one at a time.
 *
 * There are two queues. "stop" and "disable" go to an urgent queue that is
 * always served first, and long-running commands poll it so that a stop
 * arriving mid-move pre-empts the move instead of waiting behind it. Everything
 * else runs in submission order.
 *
 *   transport --> servo_ctl_submit() --> [urgent | normal] --> servo task --> mks_*
 *                                                                  |
 *                                             replies <------------+ via cmd_sink_t
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "mks_servo42c.h"
#include "servo_cmd.h"

/* Creates the queues and starts the servo task. `servo` must already be
 * initialised, and must not be touched by any other task afterwards. */
esp_err_t servo_ctl_start(mks_t *servo);

/* Queues one command line for execution, from any task. Returns false if the
 * queue is full, having already written an ERR line to `sink`.
 *
 * Replies are delivered later, from the servo task, through `sink`. The sink is
 * copied, but its `ctx` must stay valid until the command has run.
 */
bool servo_ctl_submit(const char *line, const cmd_sink_t *sink);

/* Number of commands waiting, for transports that want to show backpressure. */
unsigned servo_ctl_pending(void);
