# mks-servo42c-esp32

Control an **MKS SERVO42C** closed-loop stepper driver from an **ESP32** over UART,
using PlatformIO + ESP-IDF.

The driver implements the complete serial command set from the
[MKS-SERVO42C wiki](https://github.com/makerbase-mks/MKS-SERVO42C/wiki) /
*MKS SERVO42C V1.1.2 User Manual*: reads, parameter writes, homing, PID/ACC/torque,
and the CR_UART motion commands.

## Wiring

UART0 is left alone — it carries the boot loader, flashing and the log console over
USB — so the servo sits on **UART2**:

| ESP32           | MKS SERVO42C |
| --------------- | ------------ |
| GPIO17 (TX2)    | RX           |
| GPIO16 (RX2)    | TX           |
| GND             | GND          |

A **common ground is required**. The servo's motor supply (12–24 V) is separate;
the ESP32 is powered over USB. Do not feed 5 V into the servo's logic pins — both
sides are 3.3 V.

Pins, baud rate and address live in [`include/servo_config.h`](include/servo_config.h).

## Servo setup (on-board screen)

The firmware talks to a servo configured as:

| Menu option | Value    | Why |
| ----------- | -------- | --- |
| `Mode`      | `CR_UART` | Required for the `F3`/`F6`/`F7`/`FD` motion commands |
| `UartBaud`  | `19200`   | Must match `SERVO_BAUD_RATE` |
| `UartAddr`  | `0xE0`    | Must match `SERVO_ADDRESS` |
| `MStep`     | `16`      | Must match `SERVO_MICROSTEPS` for correct angle/RPM math |
| `MotType`   | `1.8`     | Must match `SERVO_STEP_ANGLE_IS_1_8` |

Run `Cal` (calibration) once with the motor unloaded before using closed-loop modes.

`MStep` and `MotType` only affect this side's unit conversion — get them wrong and
the motor still turns, just not by the distance you asked for.

## Build, flash, monitor

```sh
pio run                 # build
pio run -t upload        # flash over USB (UART0)
pio device monitor       # log console, 115200 baud
```

The monitor's 115200 baud is the ESP-IDF console on UART0 and is unrelated to the
19200 baud servo link on UART2.

## Control console

`src/main.c` brings the link up, then hands over to an interactive console on the
USB serial port. Open it with `pio device monitor` and type `help`.

```
servo> info
link      UART2 TX=GPIO19 RX=GPIO18 38400 baud addr 0xE0
motor     1.8 deg/step, MStep 16
geometry  3200 pulses/rev
speed     code 1 = 9.375 rpm, code 127 = 1190.6 rpm
OK info

servo> enable 1
OK enable
servo> move 90
.. moving 90.00 deg at 60.0 rpm
OK move
servo> run ccw 120
.. running ccw at 121.9 rpm (code 13)
OK run
servo> stop
OK stop
```

Every reply starts with `OK`, `ERR` or `..` (progress), so a remote client can
parse them.

| Command | Purpose |
| ------- | ------- |
| `help` / `info` / `status` | command list, local geometry, full readback |
| `read <encoder\|angle\|error\|pulses\|en\|protect>` | one field |
| `enable [0\|1]` / `disable` / `stop` | motor power and stopping |
| `move <deg> [rpm]` / `rev <revs> [rpm]` | relative positioning, blocks until done |
| `pulses <cw\|ccw> <code> <n>` | raw pulse move |
| `run <cw\|ccw> <rpm>` / `speedcode <cw\|ccw> <0-127>` | constant speed |
| `save [on\|off]` | store the current speed as power-on behaviour |
| `zero <go\|here\|mode\|speed\|dir> [arg]` | homing |
| `protect [clear]` | stall protection |
| `set <param> <value>` | `current mstep mode dir mottype protect mplyer screenoff kp ki kd acc maxt addr baud` |
| `cal` / `restore` | calibrate (unloaded), factory reset |
| `demo` | one automated cycle: 1 rev CW, 1 rev CCW, 3 s constant speed |
| `raw <cmd> [data..] [rx=n]` | arbitrary frame, for commands not wrapped |

Numbers accept hex (`set kp 0x120`) as well as decimal. Negative degrees or
revolutions run counter-clockwise. `#` starts a comment, so command scripts can
be pasted in.

The `set` commands write the servo's EEPROM — use them for commissioning, not in
a loop. `set addr` and `set baud` also re-point this side of the link so the
session keeps working; edit `servo_config.h` to make the change survive a reboot.

If the servo does not answer at boot, the firmware prints a wiring/config
checklist and still starts the console, so you can probe by hand with `raw`.

## Architecture

```
  transport                    servo_ctl                    servo_cmd + driver
  ---------                    ---------                    ------------------
  console_serial.c  submit()   [ urgent queue ]  dequeue     servo_cmd_execute_line()
  (or a socket)     ------->   [ normal queue ]  ------->    mks_* on UART2
                                                   |
                       replies via cmd_sink_t <-----+
```

A single **servo task** owns the `mks_t` handle and is the only code that talks to
the UART. Transports never call the driver; they queue a command line with
`servo_ctl_submit()` and receive replies later through their `cmd_sink_t`.

That solves two problems at once:

- **Serialisation.** `mks_t` is not thread-safe — one request/reply exchange has to
  finish before the next begins, or two callers read each other's replies. With
  every transport funnelling through one task, several can be connected at once.
- **Pre-emptible moves.** `stop` and `disable` go to a separate urgent queue that
  is always served first. Long-running commands (`move`, `rev`, `pulses`, `demo`)
  poll that queue instead of blocking on the servo's "run complete" reply, so a
  stop typed mid-move takes effect immediately:

```
servo> rev 20 10
.. moving 20.000 rev at 10.0 rpm
stop
OK move aborted
OK stop
servo>
```

On pre-emption the move handler halts the motor itself and then discards whatever
the interrupted move reports, so the next command cannot mistake a stale frame
for its own reply. A stop that arrives before the move has started suppresses the
motion command entirely rather than letting the motor twitch.

The serial console never blocks on command completion — that is what lets it read
a `stop` while a move is running. The prompt is written by the servo task once a
command finishes, so replies land before it.

`cal` and `zero go` are not pre-emptible: the servo itself is busy for the
duration and there is nothing useful to interrupt.

### Adding a WiFi transport

A socket transport reuses all of the above unchanged — implement `write`, then
submit:

```c
static void tcp_write(void *ctx, const char *text)
{
    send(*(int *)ctx, text, strlen(text), 0);
}

int client_fd = /* accepted socket */;
cmd_sink_t sink = { .write = tcp_write, .ctx = &client_fd };   /* prompt = NULL */
servo_ctl_submit(line, &sink);
```

`servo_ctl` installs the abort hook itself, so a WiFi client's `stop` pre-empts a
move started from the serial console and vice versa. Keep `ctx` alive until the
command has run: replies are delivered from the servo task, after
`servo_ctl_submit()` has already returned.

## Using the driver

```c
#include "mks_servo42c.h"

mks_t servo;
const mks_config_t cfg = {
    .uart_num = 2, .tx_gpio = 17, .rx_gpio = 16,
    .baud_rate = 19200, .address = 0xE0,
    .microsteps = 16, .step_angle_1_8 = true,
    .reply_timeout_ms = 300,
};
ESP_ERROR_CHECK(mks_init(&servo, &cfg));

mks_enable(&servo, true);

/* absolute-ish positioning: 90 degrees CW at 60 RPM, block until finished */
mks_move_degrees(&servo, 90.0f, 60.0f, true, 5000);

/* negative degrees run counter-clockwise */
mks_move_degrees(&servo, -90.0f, 60.0f, true, 5000);

/* free running */
mks_run_rpm(&servo, MKS_DIR_CW, 120.0f);
vTaskDelay(pdMS_TO_TICKS(2000));
mks_stop(&servo);

float angle;
mks_read_angle_deg(&servo, &angle);
```

Every function returns `esp_err_t`. `ESP_ERR_TIMEOUT` means no reply (wiring, baud
rate or address), `ESP_ERR_INVALID_CRC` means a corrupt reply, and `ESP_FAIL` means
the servo answered but rejected the command — usually because it is not in
`CR_UART` mode.

### Notable functions

| Area | Functions |
| ---- | --------- |
| Motion | `mks_enable`, `mks_move_pulses`, `mks_move_degrees`, `mks_move_revolutions`, `mks_run_constant_speed`, `mks_run_rpm`, `mks_stop`, `mks_save_speed_state` |
| Feedback | `mks_read_encoder`, `mks_read_angle_deg`, `mks_read_angle_error_deg`, `mks_read_pulses`, `mks_read_en_state`, `mks_read_protect_state`, `mks_release_protect` |
| Homing | `mks_set_zero_mode`, `mks_set_zero_here`, `mks_set_zero_speed`, `mks_set_zero_direction`, `mks_goto_zero` |
| Setup | `mks_set_mode`, `mks_set_current_ma`, `mks_set_microsteps`, `mks_set_motor_type`, `mks_set_direction`, `mks_set_stall_protection`, `mks_set_interpolation`, `mks_calibrate_encoder`, `mks_restore_defaults` |
| Tuning | `mks_set_kp`, `mks_set_ki`, `mks_set_kd`, `mks_set_acceleration`, `mks_set_max_torque` |
| Units | `mks_pulses_per_rev`, `mks_speed_code_to_rpm`, `mks_rpm_to_speed_code` |
| Raw | `mks_transfer` for anything not wrapped above |

The `mks_set_*` parameter calls write the servo's EEPROM, so call them during
commissioning rather than on every boot. `SERVO_APPLY_SETTINGS_ON_BOOT` in
`servo_config.h` gates the example that does this and defaults to off.

`mks_set_baud_code()` and `mks_set_address()` change the link itself: the servo
answers at the old setting, then switches. Follow them with
`mks_reconfigure_link()` to move this side to match.

## Protocol notes

```
host  -> servo : [addr][cmd][data ...][chk]
servo -> host  : [addr][data ...][chk]

chk = (sum of all preceding bytes in the frame) & 0xFF     "CHECKSUM-8"
```

- Addresses are `0xE0`–`0xE9`, so up to 10 servos share one bus.
- Multi-byte values are big-endian.
- Speed is a 7-bit code with the direction in bit 7 (`0x80` = CCW), not an RPM:
  `Vrpm = code x 30000 / (MStep x 200)` for a 1.8°/step motor (`x 400` for 0.9°).
  At `MStep = 16` code 16 is 150 RPM and the maximum code 127 is ~1190 RPM.
  Keep it under 2000 RPM.
- `FD` (move by pulses) replies **twice**: status `1` when the move starts and
  status `2` when it completes. `mks_move_pulses()` handles both; pass
  `wait_complete = false` to return early and call `mks_wait_move_complete()`
  yourself later.
- One revolution is `200 x MStep` pulses (`400 x MStep` for a 0.9° motor).
- The manual's example `e0 f6 81 d7` has a typo — the correct checksum for
  reverse at speed 1 is `0x57`. This driver computes checksums, so it is unaffected.

Every frame this driver emits was checked byte-for-byte against the examples in
the manual (parts 5.1–5.5 and 6.4).

## Diagnostics

When the link does not come up, flash the diagnostics firmware instead of the demo:

```sh
pio run -e diag -t upload && pio device monitor -e diag
```

It runs four phases and never aborts on an error:

1. **passive listen** — dumps anything arriving on the RX pin
2. **baud sweep** — sends read-encoder at 9600/19200/25000/38400/57600/115200,
   both with and without the trailing checksum byte (V1.0 firmware expects none),
   and prints every raw byte that comes back
3. **address sweep** — tries slave addresses 0xE0–0xE9
4. **loopback** — with the servo unplugged and a jumper from GPIO17 to GPIO16,
   proves whether the ESP32 half of the link works at all

A reply in phase 2 or 3 gives you the working baud rate and address; put them in
`include/servo_config.h` and rebuild the demo.

## Troubleshooting

| Symptom | Likely cause |
| ------- | ------------ |
| `ESP_ERR_TIMEOUT` on every command | TX/RX swapped, no common ground, or `UartBaud`/`UartAddr` mismatch |
| Commands acknowledged but motor does not move | `Mode` is not `CR_UART`, or the motor is not enabled (`mks_enable`) |
| Motor moves the wrong distance | `SERVO_MICROSTEPS`/`SERVO_STEP_ANGLE_IS_1_8` do not match the servo's `MStep`/`MotType` |
| Motor stalls or skips | current too low (`mks_set_current_ma`), acceleration too aggressive (`mks_set_acceleration`), or supply voltage too low |
| Stall protection keeps tripping | clear with `mks_release_protect()`; run `Cal` and check the mechanics |
| `ESP_ERR_INVALID_CRC` occasionally | long or unshielded UART wiring, or a ground loop |
| Nothing at all, servo `Tx` sits at 0 V | wrong header — the UART pins are the 4-pin `3V3 / G / Tx / Rx` group, not `Com / En / Stp / Dir` |
| Nothing at all on GPIO16/17 | ESP32-WROVER modules wire GPIO16/17 to the PSRAM chip; use different pins (phase 4 of the diagnostics confirms this) |

The servo's UART header is `3V3` / `G` / `Tx` / `Rx`, where `3V3` is left floating
and the labels are from the *servo's* point of view — its `Tx` is an output, so it
goes to the ESP32's RX pin. Use the header's own `G` as the ground reference.

## Layout

```
platformio.ini            build config: env:esp32dev and env:diag
sdkconfig.defaults        ESP-IDF options (4 MB flash, full printf, console on UART0)
CMakeLists.txt            IDF project entry point
include/servo_config.h    wiring, baud, address, motor and demo settings
include/mks_servo42c.h    driver API
src/mks_servo42c.c        protocol implementation
src/servo_cmd.c           text command layer, transport-agnostic
src/servo_ctl.c           servo task: owns the handle, queues, pre-emption
src/console_serial.c      UART0 transport for the command layer
src/diag.c                link diagnostics (env:diag only)
src/main.c                startup: bring up the link, start the console
```

## References

- [MKS-SERVO42C wiki](https://github.com/makerbase-mks/MKS-SERVO42C/wiki)
- [Serial communication description](https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description)
- [MKS SERVO42C V1.1.2 User Manual (PDF)](https://vallder.com/wp-content/uploads/2024/06/MKS-SERVO42C-User-Manual-V1.1.2.pdf)
