#include "servo_cmd.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "console_tcp.h"
#include "servo_config.h"
#include "wifi_link.h"

#define CMD_MAX_ARGS 12
#define CMD_LINE_OUT 256

static mks_t *g_servo;

/* ------------------------------------------------------------------ */
/* Output                                                              */
/* ------------------------------------------------------------------ */

void cmd_printf(const cmd_sink_t *sink, const char *fmt, ...)
{
    if (sink == NULL || sink->write == NULL) {
        return;
    }
    char buf[CMD_LINE_OUT];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sink->write(sink->ctx, buf);
}

static void report(const cmd_sink_t *s, const char *what, esp_err_t err)
{
    if (err == ESP_OK) {
        cmd_printf(s, "OK %s\r\n", what);
    } else {
        cmd_printf(s, "ERR %s: %s\r\n", what, esp_err_to_name(err));
    }
}

/* ------------------------------------------------------------------ */
/* Argument parsing                                                    */
/* ------------------------------------------------------------------ */

static bool parse_long(const char *s, long *out)
{
    char *end = NULL;
    long v = strtol(s, &end, 0);          /* base 0 accepts 0x.. and decimal */
    if (end == s || *end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

static bool parse_float(const char *s, float *out)
{
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

static bool parse_dir(const char *s, mks_dir_t *out)
{
    if (strcasecmp(s, "cw") == 0)  { *out = MKS_DIR_CW;  return true; }
    if (strcasecmp(s, "ccw") == 0) { *out = MKS_DIR_CCW; return true; }
    return false;
}

static bool parse_bool(const char *s, bool *out)
{
    if (strcasecmp(s, "1") == 0 || strcasecmp(s, "on") == 0 ||
        strcasecmp(s, "true") == 0 || strcasecmp(s, "yes") == 0) {
        *out = true;
        return true;
    }
    if (strcasecmp(s, "0") == 0 || strcasecmp(s, "off") == 0 ||
        strcasecmp(s, "false") == 0 || strcasecmp(s, "no") == 0) {
        *out = false;
        return true;
    }
    return false;
}

/* How often a long-running command checks for a pre-empting stop. */
#define ABORT_POLL_MS 50

static bool abort_requested(const cmd_sink_t *s)
{
    return s != NULL && s->abort_requested != NULL && s->abort_requested(s->ctx);
}

/* Sleeps in short slices, returning early if a pre-empting command arrives. */
static bool sleep_or_abort(const cmd_sink_t *s, uint32_t ms)
{
    while (ms > 0) {
        if (abort_requested(s)) {
            return true;
        }
        uint32_t slice = (ms < ABORT_POLL_MS) ? ms : ABORT_POLL_MS;
        vTaskDelay(pdMS_TO_TICKS(slice));
        ms -= slice;
    }
    return abort_requested(s);
}

/* Runs an already-started FD move to completion.
 *
 * The servo answers an FD command twice: status 1 when the move starts and
 * status 2 when it finishes. Rather than blocking on the second reply, poll
 * for it so that a stop arriving mid-move can take over. On abort we halt the
 * motor here and then swallow whatever the interrupted move reports, so the
 * next command does not read a stale frame as its own reply.
 */
static void await_move(const cmd_sink_t *s, const char *what, uint32_t timeout_ms)
{
    uint32_t polls = (timeout_ms + ABORT_POLL_MS - 1) / ABORT_POLL_MS;
    if (polls == 0) {
        polls = 1;
    }

    for (uint32_t i = 0; i < polls; i++) {
        if (abort_requested(s)) {
            esp_err_t stop_err = mks_stop(g_servo);
            mks_discard_pending(g_servo, 250);
            if (stop_err == ESP_OK) {
                cmd_printf(s, "OK %s aborted\r\n", what);
            } else {
                cmd_printf(s, "ERR %s abort: stop failed: %s\r\n",
                           what, esp_err_to_name(stop_err));
            }
            return;
        }

        esp_err_t err = mks_wait_move_complete(g_servo, ABORT_POLL_MS);
        if (err == ESP_OK) {
            report(s, what, ESP_OK);
            return;
        }
        if (err != ESP_ERR_TIMEOUT) {
            report(s, what, err);
            mks_discard_pending(g_servo, 100);
            return;
        }
    }

    cmd_printf(s, "ERR %s: no completion reply within %" PRIu32 " ms\r\n",
               what, timeout_ms);
    cmd_printf(s, ".. the servo accepted the move but never reported finishing;"
                  " check 'read en' and 'read protect'\r\n");
    mks_discard_pending(g_servo, 100);
}

/* Starts a pulse move and tracks it to completion. */
static void start_and_await(const cmd_sink_t *s, const char *what,
                            mks_dir_t dir, uint8_t code, uint32_t pulses,
                            uint32_t timeout_ms)
{
    if (pulses == 0) {
        cmd_printf(s, "OK %s (zero distance, nothing to do)\r\n", what);
        return;
    }
    /* A stop may have arrived between dequeuing this command and starting it.
     * Check before commanding motion, so the motor does not twitch. */
    if (abort_requested(s)) {
        cmd_printf(s, "OK %s pre-empted before start\r\n", what);
        return;
    }
    mks_run_status_t status = MKS_RUN_FAIL;
    esp_err_t err = mks_start_move_pulses(g_servo, dir, code, pulses, &status);
    if (err != ESP_OK) {
        report(s, what, err);
        return;
    }
    /* A short move can be over before it answers, and then there is no second
     * reply to wait for. */
    if (status == MKS_RUN_COMPLETE) {
        report(s, what, ESP_OK);
        return;
    }
    await_move(s, what, timeout_ms);
}

/* Completion timeout for a move, from its length and speed plus margin. */
static uint32_t move_timeout_ms(float revolutions, float rpm)
{
    if (rpm < 0.1f) {
        rpm = 0.1f;
    }
    if (revolutions < 0) {
        revolutions = -revolutions;
    }
    return (uint32_t)(revolutions / rpm * 60.0f * 1000.0f) + 3000u;
}

/* ------------------------------------------------------------------ */
/* Readback                                                            */
/* ------------------------------------------------------------------ */

static const char *en_state_str(mks_en_state_t st)
{
    switch (st) {
    case MKS_EN_STATE_ENABLED:  return "enabled";
    case MKS_EN_STATE_DISABLED: return "disabled";
    default:                    return "error";
    }
}

static const char *protect_state_str(mks_protect_state_t st)
{
    switch (st) {
    case MKS_PROTECT_TRIPPED:     return "tripped";
    case MKS_PROTECT_NOT_TRIPPED: return "ok";
    default:                      return "error";
    }
}

static void cmd_status(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;

    int32_t carry = 0;
    uint16_t enc = 0;
    if (mks_read_encoder(g_servo, &carry, &enc) == ESP_OK) {
        cmd_printf(s, "encoder   carry=%" PRId32 " value=%u\r\n", carry, enc);
    } else {
        cmd_printf(s, "encoder   unavailable\r\n");
    }

    float angle = 0.0f;
    if (mks_read_angle_deg(g_servo, &angle) == ESP_OK) {
        cmd_printf(s, "angle     %.2f deg\r\n", angle);
    }

    float err_deg = 0.0f;
    if (mks_read_angle_error_deg(g_servo, &err_deg) == ESP_OK) {
        cmd_printf(s, "error     %.3f deg\r\n", err_deg);
    }

    int32_t pulses = 0;
    if (mks_read_pulses(g_servo, &pulses) == ESP_OK) {
        cmd_printf(s, "pulses    %" PRId32 "\r\n", pulses);
    }

    mks_en_state_t en = MKS_EN_STATE_ERROR;
    if (mks_read_en_state(g_servo, &en) == ESP_OK) {
        cmd_printf(s, "en        %s\r\n", en_state_str(en));
    }

    mks_protect_state_t prot = MKS_PROTECT_ERROR;
    if (mks_read_protect_state(g_servo, &prot) == ESP_OK) {
        cmd_printf(s, "protect   %s\r\n", protect_state_str(prot));
    }
    cmd_printf(s, "OK status\r\n");
}

static void cmd_read(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc;
    const char *what = argv[1];
    esp_err_t err;

    if (strcasecmp(what, "encoder") == 0) {
        int32_t carry = 0; uint16_t value = 0;
        err = mks_read_encoder(g_servo, &carry, &value);
        if (err == ESP_OK) {
            cmd_printf(s, "OK encoder %" PRId32 " %u\r\n", carry, value);
            return;
        }
    } else if (strcasecmp(what, "angle") == 0) {
        float deg = 0.0f;
        err = mks_read_angle_deg(g_servo, &deg);
        if (err == ESP_OK) { cmd_printf(s, "OK angle %.3f\r\n", deg); return; }
    } else if (strcasecmp(what, "error") == 0) {
        float deg = 0.0f;
        err = mks_read_angle_error_deg(g_servo, &deg);
        if (err == ESP_OK) { cmd_printf(s, "OK error %.3f\r\n", deg); return; }
    } else if (strcasecmp(what, "pulses") == 0) {
        int32_t p = 0;
        err = mks_read_pulses(g_servo, &p);
        if (err == ESP_OK) { cmd_printf(s, "OK pulses %" PRId32 "\r\n", p); return; }
    } else if (strcasecmp(what, "en") == 0) {
        mks_en_state_t st = MKS_EN_STATE_ERROR;
        err = mks_read_en_state(g_servo, &st);
        if (err == ESP_OK) { cmd_printf(s, "OK en %s\r\n", en_state_str(st)); return; }
    } else if (strcasecmp(what, "protect") == 0) {
        mks_protect_state_t st = MKS_PROTECT_ERROR;
        err = mks_read_protect_state(g_servo, &st);
        if (err == ESP_OK) {
            cmd_printf(s, "OK protect %s\r\n", protect_state_str(st));
            return;
        }
    } else {
        cmd_printf(s, "ERR read: unknown field '%s'\r\n", what);
        return;
    }
    report(s, "read", err);
}

/* ------------------------------------------------------------------ */
/* Motion                                                              */
/* ------------------------------------------------------------------ */

static void cmd_enable(int argc, char **argv, const cmd_sink_t *s)
{
    bool on = true;
    if (argc > 1 && !parse_bool(argv[1], &on)) {
        cmd_printf(s, "ERR enable: expected 0|1\r\n");
        return;
    }
    report(s, on ? "enable" : "disable", mks_enable(g_servo, on));
}

static void cmd_disable(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    report(s, "disable", mks_enable(g_servo, false));
}

static void cmd_stop(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    report(s, "stop", mks_stop(g_servo));
}

static void cmd_move(int argc, char **argv, const cmd_sink_t *s)
{
    float degrees = 0.0f;
    float rpm = DEMO_RPM;
    if (!parse_float(argv[1], &degrees)) {
        cmd_printf(s, "ERR move: bad degrees '%s'\r\n", argv[1]);
        return;
    }
    if (argc > 2 && !parse_float(argv[2], &rpm)) {
        cmd_printf(s, "ERR move: bad rpm '%s'\r\n", argv[2]);
        return;
    }
    cmd_printf(s, ".. moving %.2f deg at %.1f rpm\r\n", degrees, rpm);
    start_and_await(s, "move",
                    (degrees < 0.0f) ? MKS_DIR_CCW : MKS_DIR_CW,
                    mks_rpm_to_speed_code(g_servo, rpm),
                    mks_degrees_to_pulses(g_servo, degrees),
                    move_timeout_ms(degrees / 360.0f, rpm));
}

static void cmd_rev(int argc, char **argv, const cmd_sink_t *s)
{
    float revs = 0.0f;
    float rpm = DEMO_RPM;
    if (!parse_float(argv[1], &revs)) {
        cmd_printf(s, "ERR rev: bad revolutions '%s'\r\n", argv[1]);
        return;
    }
    if (argc > 2 && !parse_float(argv[2], &rpm)) {
        cmd_printf(s, "ERR rev: bad rpm '%s'\r\n", argv[2]);
        return;
    }
    cmd_printf(s, ".. moving %.3f rev at %.1f rpm\r\n", revs, rpm);
    start_and_await(s, "rev",
                    (revs < 0.0f) ? MKS_DIR_CCW : MKS_DIR_CW,
                    mks_rpm_to_speed_code(g_servo, rpm),
                    mks_degrees_to_pulses(g_servo, revs * 360.0f),
                    move_timeout_ms(revs, rpm));
}

static void cmd_pulses(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc;
    mks_dir_t dir;
    long code = 0, count = 0;
    if (!parse_dir(argv[1], &dir)) {
        cmd_printf(s, "ERR pulses: expected cw|ccw\r\n");
        return;
    }
    if (!parse_long(argv[2], &code) || code < 0 || code > 127) {
        cmd_printf(s, "ERR pulses: speed code must be 0..127\r\n");
        return;
    }
    if (!parse_long(argv[3], &count) || count < 0) {
        cmd_printf(s, "ERR pulses: bad pulse count\r\n");
        return;
    }
    float rpm = mks_speed_code_to_rpm(g_servo, (uint8_t)code);
    float revs = (float)count / (float)mks_pulses_per_rev(g_servo);
    cmd_printf(s, ".. %ld pulses %s at code %ld (%.1f rpm)\r\n",
               count, argv[1], code, rpm);
    start_and_await(s, "pulses", dir, (uint8_t)code, (uint32_t)count,
                    move_timeout_ms(revs, rpm));
}

static void cmd_run(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc;
    mks_dir_t dir;
    float rpm = 0.0f;
    if (!parse_dir(argv[1], &dir)) {
        cmd_printf(s, "ERR run: expected cw|ccw\r\n");
        return;
    }
    if (!parse_float(argv[2], &rpm)) {
        cmd_printf(s, "ERR run: bad rpm '%s'\r\n", argv[2]);
        return;
    }
    uint8_t code = mks_rpm_to_speed_code(g_servo, rpm);
    cmd_printf(s, ".. running %s at %.1f rpm (code %u)\r\n",
               argv[1], mks_speed_code_to_rpm(g_servo, code), code);
    report(s, "run", mks_run_constant_speed(g_servo, dir, code));
}

static void cmd_speedcode(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc;
    mks_dir_t dir;
    long code = 0;
    if (!parse_dir(argv[1], &dir)) {
        cmd_printf(s, "ERR speedcode: expected cw|ccw\r\n");
        return;
    }
    if (!parse_long(argv[2], &code) || code < 0 || code > 127) {
        cmd_printf(s, "ERR speedcode: must be 0..127\r\n");
        return;
    }
    cmd_printf(s, ".. code %ld = %.2f rpm\r\n", code,
               mks_speed_code_to_rpm(g_servo, (uint8_t)code));
    report(s, "speedcode", mks_run_constant_speed(g_servo, dir, (uint8_t)code));
}

static void cmd_save(int argc, char **argv, const cmd_sink_t *s)
{
    bool save = true;
    if (argc > 1 && !parse_bool(argv[1], &save)) {
        cmd_printf(s, "ERR save: expected on|off\r\n");
        return;
    }
    if (save) {
        cmd_printf(s, ".. the driver disables itself after a save; re-enable it\r\n");
    }
    report(s, "save", mks_save_speed_state(g_servo, save));
}

/* ------------------------------------------------------------------ */
/* Homing                                                              */
/* ------------------------------------------------------------------ */

/* Per the manual, "Goto 0" requires 0_Mode != Disable and a "Set 0" already
 * done, and "Set 0" itself requires 0_Mode != Disable. 0_Mode defaults to
 * Disable, so a bare 'zero go' is rejected with status 0 on a fresh servo. */
static void zero_sequence_hint(const cmd_sink_t *s)
{
    cmd_printf(s, ".. homing must be set up first, in this order:\r\n");
    cmd_printf(s, "     zero mode <dir|near>   sets 0_Mode (default: Disable)\r\n");
    cmd_printf(s, "     zero here              sets 0_Point, needs a mode first\r\n");
    cmd_printf(s, "     zero go                homes\r\n");
    cmd_printf(s, ".. optional: zero speed <0-4> (0 fastest), zero dir <cw|ccw>\r\n");
}

static void cmd_zero(int argc, char **argv, const cmd_sink_t *s)
{
    if (argc < 2) {
        zero_sequence_hint(s);
        cmd_printf(s, "OK zero\r\n");
        return;
    }
    const char *sub = argv[1];

    if (strcasecmp(sub, "go") == 0) {
        cmd_printf(s, ".. homing\r\n");
        esp_err_t err = mks_goto_zero(g_servo, 30000);
        report(s, "zero go", err);
        if (err == ESP_OK) {
            cmd_printf(s, ".. the servo homes on its own; poll 'read angle'\r\n");
        } else {
            zero_sequence_hint(s);
        }
        return;
    }
    if (strcasecmp(sub, "here") == 0) {
        esp_err_t err = mks_set_zero_here(g_servo);
        report(s, "zero here", err);
        if (err != ESP_OK) {
            cmd_printf(s, ".. 0_Mode must not be Disable: run 'zero mode dir' first\r\n");
        }
        return;
    }
    if (strcasecmp(sub, "mode") == 0) {
        if (argc < 3) { cmd_printf(s, "ERR zero mode: off|dir|near\r\n"); return; }
        mks_zero_mode_t mode;
        if (strcasecmp(argv[2], "off") == 0)       mode = MKS_ZERO_MODE_DISABLE;
        else if (strcasecmp(argv[2], "dir") == 0)  mode = MKS_ZERO_MODE_DIR;
        else if (strcasecmp(argv[2], "near") == 0) mode = MKS_ZERO_MODE_NEAR;
        else { cmd_printf(s, "ERR zero mode: off|dir|near\r\n"); return; }
        esp_err_t err = mks_set_zero_mode(g_servo, mode);
        report(s, "zero mode", err);
        if (err == ESP_OK && mode != MKS_ZERO_MODE_DISABLE) {
            cmd_printf(s, ".. the servo will now home on every power-on\r\n");
            cmd_printf(s, ".. next: 'zero here' to set the zero point\r\n");
        }
        return;
    }
    if (strcasecmp(sub, "speed") == 0) {
        long v = 0;
        if (argc < 3 || !parse_long(argv[2], &v) || v < 0 || v > 4) {
            cmd_printf(s, "ERR zero speed: 0..4 (lower is faster)\r\n");
            return;
        }
        report(s, "zero speed", mks_set_zero_speed(g_servo, (uint8_t)v));
        return;
    }
    if (strcasecmp(sub, "dir") == 0) {
        mks_dir_t dir;
        if (argc < 3 || !parse_dir(argv[2], &dir)) {
            cmd_printf(s, "ERR zero dir: cw|ccw\r\n");
            return;
        }
        report(s, "zero dir", mks_set_zero_direction(g_servo, dir));
        return;
    }
    cmd_printf(s, "ERR zero: expected go|here|mode|speed|dir\r\n");
}

/* ------------------------------------------------------------------ */
/* Parameters (these write the servo's EEPROM)                         */
/* ------------------------------------------------------------------ */

static void cmd_set(int argc, char **argv, const cmd_sink_t *s)
{
    const char *what = argv[1];
    const char *val = (argc > 2) ? argv[2] : NULL;

    if (val == NULL) {
        cmd_printf(s, "ERR set: missing value\r\n");
        return;
    }

    long n = 0;
    if (strcasecmp(what, "current") == 0) {
        if (!parse_long(val, &n) || n < 0 || n > 3000) {
            cmd_printf(s, "ERR set current: 0..3000 mA, 200 mA steps\r\n");
            return;
        }
        report(s, "set current", mks_set_current_ma(g_servo, (uint16_t)n));
    } else if (strcasecmp(what, "mstep") == 0) {
        if (!parse_long(val, &n) || n < 1 || n > 256) {
            cmd_printf(s, "ERR set mstep: 1..256\r\n");
            return;
        }
        esp_err_t err = mks_set_microsteps(g_servo, (uint16_t)n);
        report(s, "set mstep", err);
        if (err == ESP_OK) {
            cmd_printf(s, ".. now %" PRIu32 " pulses/rev\r\n",
                       mks_pulses_per_rev(g_servo));
        }
    } else if (strcasecmp(what, "mode") == 0) {
        mks_mode_t mode;
        if (strcasecmp(val, "open") == 0)      mode = MKS_MODE_CR_OPEN;
        else if (strcasecmp(val, "vfoc") == 0) mode = MKS_MODE_CR_VFOC;
        else if (strcasecmp(val, "uart") == 0) mode = MKS_MODE_CR_UART;
        else { cmd_printf(s, "ERR set mode: open|vfoc|uart\r\n"); return; }
        if (mode != MKS_MODE_CR_UART) {
            cmd_printf(s, ".. leaving CR_UART disables serial motion commands\r\n");
        }
        report(s, "set mode", mks_set_mode(g_servo, mode));
    } else if (strcasecmp(what, "dir") == 0) {
        mks_dir_t dir;
        if (!parse_dir(val, &dir)) { cmd_printf(s, "ERR set dir: cw|ccw\r\n"); return; }
        report(s, "set dir", mks_set_direction(g_servo, dir));
    } else if (strcasecmp(what, "mottype") == 0) {
        if (strcmp(val, "1.8") == 0 || strcmp(val, "18") == 0) {
            report(s, "set mottype", mks_set_motor_type(g_servo, MKS_MOTOR_1_8_DEG));
        } else if (strcmp(val, "0.9") == 0 || strcmp(val, "09") == 0) {
            report(s, "set mottype", mks_set_motor_type(g_servo, MKS_MOTOR_0_9_DEG));
        } else {
            cmd_printf(s, "ERR set mottype: 1.8|0.9\r\n");
        }
    } else if (strcasecmp(what, "protect") == 0) {
        bool on;
        if (!parse_bool(val, &on)) { cmd_printf(s, "ERR set protect: 0|1\r\n"); return; }
        report(s, "set protect", mks_set_stall_protection(g_servo, on));
    } else if (strcasecmp(what, "mplyer") == 0) {
        bool on;
        if (!parse_bool(val, &on)) { cmd_printf(s, "ERR set mplyer: 0|1\r\n"); return; }
        report(s, "set mplyer", mks_set_interpolation(g_servo, on));
    } else if (strcasecmp(what, "screenoff") == 0) {
        bool on;
        if (!parse_bool(val, &on)) { cmd_printf(s, "ERR set screenoff: 0|1\r\n"); return; }
        report(s, "set screenoff", mks_set_auto_screen_off(g_servo, on));
    } else if (strcasecmp(what, "kp") == 0 || strcasecmp(what, "ki") == 0 ||
               strcasecmp(what, "kd") == 0 || strcasecmp(what, "acc") == 0 ||
               strcasecmp(what, "maxt") == 0) {
        if (!parse_long(val, &n) || n < 0 || n > 0xFFFF) {
            cmd_printf(s, "ERR set %s: 0..65535\r\n", what);
            return;
        }
        esp_err_t err;
        if (strcasecmp(what, "kp") == 0)        err = mks_set_kp(g_servo, (uint16_t)n);
        else if (strcasecmp(what, "ki") == 0)   err = mks_set_ki(g_servo, (uint16_t)n);
        else if (strcasecmp(what, "kd") == 0)   err = mks_set_kd(g_servo, (uint16_t)n);
        else if (strcasecmp(what, "acc") == 0) {
            cmd_printf(s, ".. the manual warns an excessive ACC can damage the board\r\n");
            err = mks_set_acceleration(g_servo, (uint16_t)n);
        } else {
            if (n > 0x4B0) {
                cmd_printf(s, "ERR set maxt: 0..1200 (0x4B0)\r\n");
                return;
            }
            err = mks_set_max_torque(g_servo, (uint16_t)n);
        }
        report(s, "set", err);
    } else if (strcasecmp(what, "addr") == 0) {
        if (!parse_long(val, &n) || n < 0xE0 || n > 0xE9) {
            cmd_printf(s, "ERR set addr: 0xE0..0xE9\r\n");
            return;
        }
        esp_err_t err = mks_set_address(g_servo, (uint8_t)n);
        report(s, "set addr", err);
        if (err == ESP_OK) {
            mks_reconfigure_link(g_servo, 0, (uint8_t)n);
            cmd_printf(s, ".. this side now uses 0x%02lX; "
                          "update SERVO_ADDRESS to keep it after a reboot\r\n",
                       (unsigned long)n);
        }
    } else if (strcasecmp(what, "baud") == 0) {
        static const struct { int rate; mks_baud_code_t code; } kBauds[] = {
            {   9600, MKS_BAUD_9600   }, {  19200, MKS_BAUD_19200  },
            {  25000, MKS_BAUD_25000  }, {  38400, MKS_BAUD_38400  },
            {  57600, MKS_BAUD_57600  }, { 115200, MKS_BAUD_115200 },
        };
        if (!parse_long(val, &n)) {
            cmd_printf(s, "ERR set baud: 9600|19200|25000|38400|57600|115200\r\n");
            return;
        }
        for (size_t i = 0; i < sizeof(kBauds) / sizeof(kBauds[0]); i++) {
            if (kBauds[i].rate == (int)n) {
                esp_err_t err = mks_set_baud_code(g_servo, kBauds[i].code);
                report(s, "set baud", err);
                if (err == ESP_OK) {
                    /* The servo answered at the old rate, then switched. */
                    vTaskDelay(pdMS_TO_TICKS(100));
                    mks_reconfigure_link(g_servo, (int)n, 0);
                    cmd_printf(s, ".. this side now at %ld baud; update "
                                  "SERVO_BAUD_RATE to keep it after a reboot\r\n", n);
                }
                return;
            }
        }
        cmd_printf(s, "ERR set baud: 9600|19200|25000|38400|57600|115200\r\n");
    } else {
        cmd_printf(s, "ERR set: unknown parameter '%s'\r\n", what);
    }
}

static void cmd_cal(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    cmd_printf(s, ".. calibrating, the motor must be unloaded (up to 20 s)\r\n");
    report(s, "cal", mks_calibrate_encoder(g_servo, 20000));
}

static void cmd_restore(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    cmd_printf(s, ".. this also resets baud rate and address to the defaults\r\n");
    report(s, "restore", mks_restore_defaults(g_servo));
}

static void cmd_protect(int argc, char **argv, const cmd_sink_t *s)
{
    if (argc > 1 && strcasecmp(argv[1], "clear") == 0) {
        report(s, "protect clear", mks_release_protect(g_servo));
        return;
    }
    mks_protect_state_t st = MKS_PROTECT_ERROR;
    esp_err_t err = mks_read_protect_state(g_servo, &st);
    if (err == ESP_OK) {
        cmd_printf(s, "OK protect %s\r\n", protect_state_str(st));
    } else {
        report(s, "protect", err);
    }
}

/* ------------------------------------------------------------------ */
/* Local info and demo                                                 */
/* ------------------------------------------------------------------ */

static void cmd_info(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    cmd_printf(s, "link      UART%d TX=GPIO%d RX=GPIO%d %d baud addr 0x%02X\r\n",
               SERVO_UART_PORT, SERVO_TX_GPIO, SERVO_RX_GPIO,
               g_servo->cfg.baud_rate, g_servo->cfg.address);
    cmd_printf(s, "motor     %s deg/step, MStep %u\r\n",
               g_servo->cfg.step_angle_1_8 ? "1.8" : "0.9", g_servo->cfg.microsteps);
    cmd_printf(s, "geometry  %" PRIu32 " pulses/rev\r\n", mks_pulses_per_rev(g_servo));
    cmd_printf(s, "speed     code 1 = %.3f rpm, code 127 = %.1f rpm\r\n",
               mks_speed_code_to_rpm(g_servo, 1), mks_speed_code_to_rpm(g_servo, 127));
    cmd_printf(s, "OK info\r\n");
}

static void cmd_net(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;

    if (SERVO_WIFI_SSID[0] == '\0') {
        cmd_printf(s, "wifi      off\r\n");
        cmd_printf(s, ".. build with WIFI_CLAITEC_SSID and WIFI_CLAITEC_PASS set "
                      "to enable it\r\n");
        cmd_printf(s, "OK net\r\n");
        return;
    }

    cmd_printf(s, "wifi      \"%s\", %s\r\n", SERVO_WIFI_SSID,
               wifi_link_is_connected() ? "connected" : "not connected");
    cmd_printf(s, "address   %s (hostname %s)\r\n",
               wifi_link_ip(), SERVO_WIFI_HOSTNAME);
    cmd_printf(s, "tcp       port %d, %u client(s)\r\n",
               SERVO_TCP_PORT, console_tcp_clients());
    cmd_printf(s, "OK net\r\n");
}

static void cmd_demo(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    const uint32_t timeout = move_timeout_ms(DEMO_REVOLUTIONS, DEMO_RPM);
    const uint8_t code = mks_rpm_to_speed_code(g_servo, DEMO_RPM);
    const uint32_t pulses = mks_degrees_to_pulses(g_servo, DEMO_REVOLUTIONS * 360.0f);

    cmd_printf(s, ".. %+.2f rev cw\r\n", DEMO_REVOLUTIONS);
    start_and_await(s, "demo cw", MKS_DIR_CW, code, pulses, timeout);
    if (abort_requested(s)) { cmd_printf(s, "OK demo aborted\r\n"); return; }

    cmd_printf(s, ".. %+.2f rev ccw\r\n", -DEMO_REVOLUTIONS);
    start_and_await(s, "demo ccw", MKS_DIR_CCW, code, pulses, timeout);
    if (abort_requested(s)) { cmd_printf(s, "OK demo aborted\r\n"); return; }

    cmd_printf(s, ".. constant speed for 3 s\r\n");
    esp_err_t err = mks_run_rpm(g_servo, MKS_DIR_CW, DEMO_RPM);
    if (err != ESP_OK) { report(s, "demo", err); return; }
    if (sleep_or_abort(s, 3000)) {
        mks_stop(g_servo);
        cmd_printf(s, "OK demo aborted\r\n");
        return;
    }
    report(s, "demo", mks_stop(g_servo));
}

/* ------------------------------------------------------------------ */
/* Raw escape hatch                                                    */
/* ------------------------------------------------------------------ */

/* raw <cmd-hex> [data-hex ...] [rx=<n>]
 * Sends an arbitrary command frame; the driver adds address and checksum. */
static void cmd_raw(int argc, char **argv, const cmd_sink_t *s)
{
    uint8_t tx[4];
    size_t tx_len = 0;
    long rx_len = 1;
    long cmd = 0;

    if (!parse_long(argv[1], &cmd) || cmd < 0 || cmd > 0xFF) {
        cmd_printf(s, "ERR raw: bad command byte '%s'\r\n", argv[1]);
        return;
    }

    for (int i = 2; i < argc; i++) {
        if (strncasecmp(argv[i], "rx=", 3) == 0) {
            if (!parse_long(argv[i] + 3, &rx_len) || rx_len < 0 || rx_len > 6) {
                cmd_printf(s, "ERR raw: rx= must be 0..6\r\n");
                return;
            }
            continue;
        }
        long b = 0;
        if (!parse_long(argv[i], &b) || b < 0 || b > 0xFF) {
            cmd_printf(s, "ERR raw: bad data byte '%s'\r\n", argv[i]);
            return;
        }
        if (tx_len >= sizeof(tx)) {
            cmd_printf(s, "ERR raw: at most %u data bytes\r\n", (unsigned)sizeof(tx));
            return;
        }
        tx[tx_len++] = (uint8_t)b;
    }

    uint8_t rx[6] = {0};
    esp_err_t err = mks_transfer(g_servo, (uint8_t)cmd, tx, tx_len,
                                 rx, (size_t)rx_len, 2000);
    if (err != ESP_OK) {
        report(s, "raw", err);
        return;
    }
    char hex[6 * 3 + 1];
    size_t off = 0;
    for (long i = 0; i < rx_len; i++) {
        off += (size_t)snprintf(hex + off, sizeof(hex) - off, "%02x ", rx[i]);
    }
    hex[off] = '\0';
    cmd_printf(s, "OK raw %s\r\n", hex);
}

/* ------------------------------------------------------------------ */
/* Dispatch table                                                      */
/* ------------------------------------------------------------------ */

typedef void (*cmd_fn_t)(int argc, char **argv, const cmd_sink_t *s);

typedef struct {
    const char *name;
    int         min_argc;   /* including the command word itself */
    cmd_fn_t    fn;
    const char *usage;
} cmd_entry_t;

static void cmd_help(int argc, char **argv, const cmd_sink_t *s);

static const cmd_entry_t kCommands[] = {
    { "help",      1, cmd_help,      "help                       this list" },
    { "info",      1, cmd_info,      "info                       link and geometry" },
    { "net",       1, cmd_net,       "net                        wifi and tcp status" },
    { "status",    1, cmd_status,    "status                     read everything" },
    { "read",      2, cmd_read,      "read <encoder|angle|error|pulses|en|protect>" },
    { "enable",    1, cmd_enable,    "enable [0|1]               energise the motor" },
    { "disable",   1, cmd_disable,   "disable                    de-energise" },
    { "stop",      1, cmd_stop,      "stop                       stop motion" },
    { "move",      2, cmd_move,      "move <deg> [rpm]           relative move, blocks" },
    { "rev",       2, cmd_rev,       "rev <revs> [rpm]           relative move, blocks" },
    { "pulses",    4, cmd_pulses,    "pulses <cw|ccw> <code> <n> raw pulse move" },
    { "run",       3, cmd_run,       "run <cw|ccw> <rpm>         constant speed" },
    { "speedcode", 3, cmd_speedcode, "speedcode <cw|ccw> <0-127> constant speed, raw" },
    { "save",      1, cmd_save,      "save [on|off]              store speed as power-on" },
    { "zero",      1, cmd_zero,      "zero [go|here|mode|speed|dir] [arg]  bare 'zero' shows the setup order" },
    { "protect",   1, cmd_protect,   "protect [clear]            stall protection" },
    { "set",       2, cmd_set,       "set <param> <value>        see below" },
    { "cal",       1, cmd_cal,       "cal                        calibrate, unloaded" },
    { "restore",   1, cmd_restore,   "restore                    factory defaults" },
    { "demo",      1, cmd_demo,      "demo                       one demo cycle" },
    { "raw",       2, cmd_raw,       "raw <cmd> [data..] [rx=n]  arbitrary frame" },
};

#define N_COMMANDS (sizeof(kCommands) / sizeof(kCommands[0]))

static void cmd_help(int argc, char **argv, const cmd_sink_t *s)
{
    (void)argc; (void)argv;
    cmd_printf(s, "commands:\r\n");
    for (size_t i = 0; i < N_COMMANDS; i++) {
        cmd_printf(s, "  %s\r\n", kCommands[i].usage);
    }
    cmd_printf(s, "set parameters (these write the servo's EEPROM):\r\n");
    cmd_printf(s, "  current <mA>   mstep <1-256>   mode <open|vfoc|uart>\r\n");
    cmd_printf(s, "  dir <cw|ccw>   mottype <1.8|0.9>   protect <0|1>\r\n");
    cmd_printf(s, "  mplyer <0|1>   screenoff <0|1>\r\n");
    cmd_printf(s, "  kp|ki|kd|acc|maxt <n>   addr <0xE0-0xE9>   baud <rate>\r\n");
    cmd_printf(s, "examples:\r\n");
    cmd_printf(s, "  enable 1 / move 90 / move -90 60 / run ccw 120 / stop\r\n");
    cmd_printf(s, "OK help\r\n");
}

void servo_cmd_init(mks_t *servo)
{
    g_servo = servo;
}

/* True if the first word of `line` is exactly `word`, ignoring case. */
static bool leading_word_is(const char *line, const char *word)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    size_t n = strlen(word);
    if (strncasecmp(line, word, n) != 0) {
        return false;
    }
    char after = line[n];
    return after == '\0' || after == ' ' || after == '\t' ||
           after == '\r' || after == '\n' || after == '#';
}

bool servo_cmd_is_urgent(const char *line)
{
    return leading_word_is(line, "stop") || leading_word_is(line, "disable");
}

void servo_cmd_dispatch(int argc, char **argv, const cmd_sink_t *sink)
{
    if (g_servo == NULL) {
        cmd_printf(sink, "ERR servo not initialised\r\n");
        return;
    }
    if (argc < 1) {
        return;
    }
    for (size_t i = 0; i < N_COMMANDS; i++) {
        if (strcasecmp(argv[0], kCommands[i].name) != 0) {
            continue;
        }
        if (argc < kCommands[i].min_argc) {
            cmd_printf(sink, "ERR usage: %s\r\n", kCommands[i].usage);
            return;
        }
        kCommands[i].fn(argc, argv, sink);
        return;
    }
    cmd_printf(sink, "ERR unknown command '%s', try 'help'\r\n", argv[0]);
}

void servo_cmd_execute_line(char *line, const cmd_sink_t *sink)
{
    char *argv[CMD_MAX_ARGS];
    int argc = 0;

    char *save = NULL;
    for (char *tok = strtok_r(line, " \t", &save);
         tok != NULL && argc < CMD_MAX_ARGS;
         tok = strtok_r(NULL, " \t", &save)) {
        if (tok[0] == '#') {         /* rest of the line is a comment */
            break;
        }
        argv[argc++] = tok;
    }
    if (argc == 0) {
        return;
    }
    servo_cmd_dispatch(argc, argv, sink);
}
