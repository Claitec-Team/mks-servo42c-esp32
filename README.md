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
| GPIO32 (TX)     | `Rx`         |
| GPIO33 (RX)     | `Tx`         |
| GND             | `G`          |

Any GPIO works — the ESP32 routes UART signals through its GPIO matrix — so
moving the link is just the two defines in `servo_config.h`.

A **common ground is required**. The servo's motor supply (12–24 V) is separate;
the ESP32 is powered over USB. Do not feed 5 V into the servo's logic pins — both
sides are 3.3 V.

Pins, baud rate and address live in [`include/servo_config.h`](include/servo_config.h).

## Servo setup (on-board screen)

The firmware talks to a servo configured as:

| Menu option | Value    | Why |
| ----------- | -------- | --- |
| `Mode`      | `CR_UART` | Required for the `F3`/`F6`/`F7`/`FD` motion commands |
| `UartBaud`  | `38400`   | Must match `SERVO_BAUD_RATE` |
| `UartAddr`  | `0xE0`    | Must match `SERVO_ADDRESS` |
| `MStep`     | `128`     | Set by the firmware at boot; see [Microstepping](#microstepping-and-speed) |
| `MotType`   | `1.8`     | Must match `SERVO_STEP_ANGLE_IS_1_8` |

Run `Cal` (calibration) once with the motor unloaded before using closed-loop modes.

`MotType` only affects this side's unit conversion — get it wrong and the motor
still turns, just not by the distance you asked for. `MStep` matters the same
way, but the firmware sets it at every boot so it cannot drift out of step.

## Build, flash, monitor

```sh
pio run                 # build
pio run -t upload        # flash over USB (UART0)
pio device monitor       # log console, 115200 baud
```

The monitor's 115200 baud is the ESP-IDF console on UART0 and is unrelated to the
38400 baud servo link on UART2.

## Control console

`src/main.c` brings the link up, then hands over to an interactive console on the
USB serial port. Open it with `pio device monitor` and type `help`.

```
servo> info
link      UART2 TX=GPIO32 RX=GPIO33 38400 baud addr 0xE0
motor     1.8 deg/step, MStep 128
geometry  25600 pulses/rev
speed     code 1 = 1.172 rpm, code 127 = 148.8 rpm
OK info

servo> enable 1
OK enable
servo> move 90
.. moving 90.00 deg at 30.0 rpm
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
| `help` / `info` / `status` / `net` | command list, local geometry, full readback, network state |
| `read <encoder\|angle\|error\|pulses\|en\|protect>` | one field |
| `enable [0\|1]` / `disable` / `stop` | motor power and stopping |
| `move <deg> [rpm]` / `rev <revs> [rpm]` | relative positioning, blocks until done; speed defaults to `SERVO_DEFAULT_RPM` |
| `goto <angle> [rpm]` | absolute positioning, shortest way round — see below |
| `pulses <cw\|ccw> <code> <n>` | raw pulse move |
| `run <cw\|ccw> <rpm>` / `speedcode <cw\|ccw> <0-127>` | constant speed |
| `save [on\|off]` | store the current speed as power-on behaviour |
| `zero [go\|here\|mode\|speed\|dir] [arg]` | homing; bare `zero` prints the setup order |
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

### Microstepping and speed

Speed is a 7-bit **code**, not an RPM figure:

```
Vrpm = code x 30000 / (MStep x 200)          # 1.8 deg/step motor
```

RPM is therefore *inversely* proportional to `MStep`: **raising it makes the motor
slower and finer, lowering it makes it faster and coarser.** Since the code is an
integer, `MStep` also sets the slowest speed achievable at all — code 1.

| MStep | pulses/rev | slowest (code 1) | fastest (code 127) | deg/pulse |
| ----- | ---------- | ---------------- | ------------------ | --------- |
| 8     | 1600       | 18.75 rpm        | 2381 rpm           | 0.2250    |
| 16    | 3200       | 9.375 rpm        | 1191 rpm           | 0.1125    |
| 32    | 6400       | 4.688 rpm        | 595 rpm            | 0.0563    |
| 64    | 12800      | 2.344 rpm        | 298 rpm            | 0.0281    |
| **128** | **25600** | **1.172 rpm**   | **148.8 rpm**      | **0.0141** |
| 256   | 51200      | 0.586 rpm        | 74.4 rpm           | 0.0070    |

The trade-off is top speed, which falls by the same factor. The manual also caps
`Vrpm` at 2000 regardless, so the low `MStep` rows are largely unusable. Higher
microstepping additionally reduces vibration and noise.

The screen's `MStep` menu only offers powers of two, 1 to 256. The serial command
(`0x84`) accepts any value in that range — the manual's own example uses 26 — but
sticking to what the screen can display keeps the two in agreement.

**The firmware sets `MStep` at every boot**, from `SERVO_MICROSTEPS`, and reports
it on the log and in every console banner:

```
I (612) app: MStep set to 128: 25600 pulses/rev, 0.01 deg/pulse
I (612) app:   speed range 1.17 rpm (code 1) to 148.8 rpm (code 127)
I (612) app:   the servo's MStep menu now shows 128 too
```

```
MKS SERVO42C control console. Type 'help'.
MStep 128: 25600 pulses/rev, 0.014 deg/pulse, 1.17-148.8 rpm
```

This exists because there is no command to *read* `MStep` back, so the firmware
cannot check the servo and adapt — it can only assert a value. Writing it every
boot means the two can never disagree, whichever way the screen buttons were last
used. The cost is one flash write per boot on the servo; set
`SERVO_APPLY_MICROSTEPS_ON_BOOT` to 0 to skip it, in which case
`SERVO_MICROSTEPS` must be edited to match the servo by hand or every distance
and speed will be scaled wrongly.

Changing it at runtime with `set mstep <n>` works too and keeps the conversions
correct for that session, but does **not** survive a reboot: `SERVO_MICROSTEPS`
is reasserted. Edit that constant to make a change permanent.

### Absolute positioning

The servo only moves relative distances, so `goto` reads where the shaft is,
works out the difference, and issues one bounded relative move:

```
servo> read angle
OK angle 657.746
servo> goto 90
.. at 657.72 deg, target 90.00 deg (mod 360): shortest path +152.28 deg at 30.0 rpm
.. arrived at 810.00 deg, -0.000 deg from target
OK goto
```

Two things worth knowing:

**The target is a position modulo 360, reached the shortest way round.** No `goto`
ever turns more than half a revolution, so it cannot unwind several turns
unexpectedly. The servo's own angle keeps accumulating across turns, though, so
after a few turns the reading will not equal the target numerically even though
the shaft is in the right place — `goto 0` from `720.0` is already there and does
not move. Every error reported is therefore a modular one.

**The direction depends on your wiring.** Whether a clockwise move makes the
reported angle rise or fall depends on the servo's `Dir` setting and the motor's
coil order, so it cannot be known in advance. `SERVO_ANGLE_INCREASES_CW` in
`servo_config.h` says which it is. On the hardware this was developed against a
clockwise move *decreases* the angle, so it is set to 0 — check yours, because if
it is wrong `goto` moves away from the target and says so:

```
ERR goto: ended further from the target than it started
.. the angle counts the other way round: set SERVO_ANGLE_INCREASES_CW to 0 in servo_config.h
```

Because each `goto` is a single bounded move, a wrong setting cannot run away —
it overshoots once, reports it, and stops.

Measured on real hardware at `MStep = 128`: 13 consecutive `goto` commands in both
directions all landed within **0.5°** of target, most within 0.2°. The floor is
the servo's own closed-loop error rather than pulse rounding, which is 0.014°.

### Homing

`zero go` on a fresh servo fails with status 0. Per the manual, "Goto 0" needs
`0_Mode` set to something other than `Disable` *and* a zero point already stored,
and `0_Mode` defaults to `Disable`. Set it up once:

```
servo> zero mode dir        # or 'near' for the shortest path
.. the servo will now home on every power-on
servo> zero here            # store the current position as zero
servo> zero go
```

Once `0_Mode` is not `Disable` the servo also homes by itself at every power-on —
`zero mode off` turns that back off. `zero go` returns as soon as the servo
accepts it and homes on its own, so poll `read angle` to see it arrive.

### Completion reporting

An `FD` move is supposed to be acknowledged twice: status 1 when it starts and
status 2 when it finishes. On the hardware this was developed against **the second
frame goes missing on roughly half of all moves** — measured 25/50 back-to-back,
and 5/8 and 4/8 in single-direction batches, so it is not tied to direction —
while the shaft reaches the commanded position every time.

Worse, the acknowledgement can arrive when nothing moved at all: with the motor
disabled the servo still answered "run complete" for every move while the shaft
sat still. So the frame is evidence that the command was accepted, not that the
shaft went anywhere.

So a move that gets no completion frame is settled by the encoder rather than
assumed failed. `move`/`rev`/`pulses`/`goto` read the shaft angle before starting,
and if the frame never comes they check whether it turned as far as it was told:

```
servo> move 20
.. moving 20.00 deg at 30.0 rpm
OK move (position reached; the servo sent no completion frame)
```

If the shaft did *not* arrive, that is still a failure and still reported:

```
ERR move: no completion reply within 928 ms, and the shaft is not where it was told to go
.. check 'read en' and 'read protect'
```

The comparison is by magnitude, so it does not depend on
`SERVO_ANGLE_INCREASES_CW`. The timeout margin is deliberately small (15% of the
expected travel plus 800 ms) because it is reached so often, and every millisecond
of it is dead time before the encoder is consulted.

Following error (`read error`) is *not* usable as a witness here, in case it looks
tempting: the closed loop keeps it under about half a degree even mid-move, so it
cannot tell "finished" from "still going".

Measured after the change: 50 back-to-back moves gave 25 completion frames, 25
encoder confirmations and **0 failures**, with 0.022° of drift over the 50 —
against 48 failures out of 50 for the same test beforehand. A move that really
cannot complete, such as one issued while the motor is disabled, is still
reported as a failure.

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

## Remote control over WiFi

Credentials come from the environment at build time, so they never enter the
repository:

```sh
export WIFI_CLAITEC_SSID='your-ssid'
export WIFI_CLAITEC_PASS='your-password'
pio run -t upload && pio device monitor
```

`scripts/wifi_credentials.py` writes them into `include/wifi_credentials.h`
(gitignored) before each build. With the variables unset the SSID is empty, the
radio is never started, and the firmware behaves exactly as it does without
WiFi — so an unconfigured checkout still builds and runs.

Find the address from the serial console, then connect:

```
servo> net
wifi      "your-ssid", connected
address   192.168.1.57 (hostname mks-servo42c)
tcp       port 3333, 0 client(s)
OK net
```

```sh
$ nc 192.168.1.57 3333          # or: nc mks-servo42c 3333
MKS SERVO42C. One command per line, 'help' for a list.
enable 1
OK enable
rev 20 10
.. moving 20.000 rev at 10.0 rpm
stop
OK move aborted
OK stop
```

The full command set is available, identical to the serial console. There is no
prompt over TCP, which keeps the stream easy to parse: every line begins with
`OK`, `ERR` or `..`.

Up to four clients may connect at once and the serial console keeps working
alongside them. Because every transport funnels through `servo_ctl`, commands
from different clients are serialised, and a `stop` from any of them pre-empts a
move started by another.

Since the line protocol is plain text, scripting it needs no library:

```sh
printf 'enable 1\nmove 90\nread angle\n' | nc -q1 192.168.1.57 3333
```

### tools/servoctl

`nc` has no history and no guard rails, so `tools/servoctl` is a friendlier
client for interactive use. Python 3 only, no dependencies:

```sh
tools/servoctl 192.168.1.57          # port defaults to 3333
tools/servoctl mks-servo42c 3333     # or spell it out, like nc
```

```
servoctl -> 192.168.1.57:3333
  speed: default 30 rpm, capped at 100 rpm; '!' prefix bypasses the cap
  up arrow for history, tab to complete, Ctrl-C sends stop, Ctrl-D quits
servo> move 90
-- speed defaulted to 30 rpm
.. moving 90.00 deg at 30.0 rpm
OK move
servo> run cw 400
-- 400 rpm capped to 100 rpm
.. running cw at 100.0 rpm (code 10)
OK run
```

- **History** on the up arrow, persisted to `$XDG_STATE_HOME/servoctl/history`
  so it survives across sessions.
- **Tab completion** of commands, and of the arguments to `read`, `zero` and
  `set`.
- **Ctrl-C sends `stop`** rather than quitting. With a motor turning, the reflex
  key should be a brake — quitting the client would leave it spinning. Use
  Ctrl-D or `quit` to exit.
- **Default speed** of 30 rpm filled in whenever `move`/`rev`/`goto` omit one,
  matching `SERVO_DEFAULT_RPM` in the firmware so both consoles behave alike.
- **Speed cap** of 100 rpm applied to `move`, `rev`, `goto`, `run`, and — via
  the geometry read from `info` at connect — to the raw speed codes in
  `speedcode` and `pulses`.

Both limits are adjustable: `--default-rpm`, `--max-rpm`. Every rewrite is
announced with a `--` line, so nothing changes silently.

> The cap is a convenience, not a safety interlock: it lives in the client, so
> `nc` or the serial console bypass it entirely, and so does a `!`-prefixed line
> (`!run cw 400`). If you need a real limit, it belongs in the firmware —
> `mks_set_max_torque()` and `mks_rpm_to_speed_code()` are where it would go.

> **Security.** The port is unauthenticated and unencrypted: anyone who can
> reach it can drive the motor. Use it only on a network you trust, and do not
> forward the port through a router. The credentials are also compiled into the
> firmware image, so treat `firmware.bin` as a secret too.

### If it reboots as soon as WiFi is enabled

Powering up the radio is the largest current spike in the whole boot, and a board
fed through a thin USB cable can sag enough to brown out. The signature is a
reboot during PHY calibration, before WiFi ever associates, with no panic output:

```
W (702) phy_init: failed to load RF calibration data (0xffffffff), falling back to full calibration
ets Jun  8 2016 00:22:57
rst:0x3 (SW_RESET),boot:0x33 (SPI_FAST_FLASH_BOOT)
```

There is no backtrace because a brownout is not a panic: it is handled by an
interrupt that resets the chip in software, hence `SW_RESET`. The firmware names
it on the next boot:

```
E (562) app: reset: BROWNOUT - the 3.3 V rail sagged, this is a power problem...
```

In order of effectiveness: a better supply (powered hub, or 5 V on VIN), a
shorter or thicker USB cable, a 470–1000 µF bulk capacitor across 3V3–GND at the
module, and lowering `SERVO_WIFI_MAX_TX_POWER` in `servo_config.h` to shrink the
spike. Do **not** disable `CONFIG_ESP_BROWNOUT_DET` to silence it — that lets the
chip keep running below its safe voltage, which corrupts flash.

### Adding another transport

Implement one `write` callback and submit; everything else is inherited:

```c
static void my_write(void *ctx, const char *text) { /* send `text` somewhere */ }

cmd_sink_t sink = { .write = my_write, .ctx = whatever, .prompt = NULL };
servo_ctl_submit(line, &sink);
```

`servo_ctl` installs the abort hook itself, so pre-emption works from any
transport. One caveat that `console_tcp.c` shows how to handle: replies are
delivered later, from the servo task, so `ctx` must still be valid — or at least
safe to interpret — after `servo_ctl_submit()` returns. The TCP transport packs a
slot index and a generation counter into `ctx` instead of a pointer, and drops
the reply if that slot no longer holds the same connection. A raw `int *` to a
socket would risk writing one client's reply into another client's connection
after a disconnect reused the descriptor.

## Using the driver

```c
#include "mks_servo42c.h"

mks_t servo;
const mks_config_t cfg = {
    .uart_num = 2, .tx_gpio = 32, .rx_gpio = 33,
    .baud_rate = 38400, .address = 0xE0,
    .microsteps = 128, .step_angle_1_8 = true,
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
  So RPM falls as `MStep` rises — see [Microstepping](#microstepping-and-speed).
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
| Reboots during PHY calibration once WiFi is enabled | 3.3 V brownout — see below |
| Nothing at all, servo `Tx` sits at 0 V | wrong header — the UART pins are the 4-pin `3V3 / G / Tx / Rx` group, not `Com / En / Stp / Dir` |
| Nothing at all on GPIO16/17 | ESP32-WROVER modules wire GPIO16/17 to the PSRAM chip; use different pins (phase 4 of the diagnostics confirms this) |

The servo's UART header is `3V3` / `G` / `Tx` / `Rx`, where `3V3` is left floating
and the labels are from the *servo's* point of view — its `Tx` is an output, so it
goes to the ESP32's RX pin. Use the header's own `G` as the ground reference.

## Layout

```
platformio.ini              build config: env:esp32dev and env:diag
sdkconfig.defaults          ESP-IDF options (4 MB flash, full printf, console on UART0)
CMakeLists.txt              IDF project entry point
scripts/wifi_credentials.py generates wifi_credentials.h from the environment
tools/servoctl              host-side client: history, completion, speed cap
include/servo_config.h      wiring, baud, address, motor, wifi and demo settings
include/mks_servo42c.h      driver API
src/mks_servo42c.c          protocol implementation
src/servo_cmd.c             text command layer, transport-agnostic
src/servo_ctl.c             servo task: owns the handle, queues, pre-emption
src/console_serial.c        UART0 transport for the command layer
src/console_tcp.c           TCP transport, up to four concurrent clients
src/wifi_link.c             station setup and reconnection
src/diag.c                  link diagnostics (env:diag only)
src/main.c                  startup: bring up the link, start the transports
```

## References

- [MKS-SERVO42C wiki](https://github.com/makerbase-mks/MKS-SERVO42C/wiki)
- [Serial communication description](https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description)
- [MKS SERVO42C V1.1.2 User Manual (PDF)](https://vallder.com/wp-content/uploads/2024/06/MKS-SERVO42C-User-Manual-V1.1.2.pdf)
