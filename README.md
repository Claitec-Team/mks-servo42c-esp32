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

## What the demo does

`src/main.c` probes the link, prints the servo state, enables the motor, then loops:

1. one revolution clockwise, waiting for the "run complete" reply
2. one revolution counter-clockwise
3. three seconds of constant-speed rotation, then stop

after each step it reads back encoder, shaft angle, angle error, pulse count,
En-pin state and stall-protection state.

Speed and distance come from `DEMO_RPM` / `DEMO_REVOLUTIONS` in `servo_config.h`.

If the servo does not answer, the demo prints a wiring/config checklist and stops
rather than driving blind.

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

## Troubleshooting

| Symptom | Likely cause |
| ------- | ------------ |
| `ESP_ERR_TIMEOUT` on every command | TX/RX swapped, no common ground, or `UartBaud`/`UartAddr` mismatch |
| Commands acknowledged but motor does not move | `Mode` is not `CR_UART`, or the motor is not enabled (`mks_enable`) |
| Motor moves the wrong distance | `SERVO_MICROSTEPS`/`SERVO_STEP_ANGLE_IS_1_8` do not match the servo's `MStep`/`MotType` |
| Motor stalls or skips | current too low (`mks_set_current_ma`), acceleration too aggressive (`mks_set_acceleration`), or supply voltage too low |
| Stall protection keeps tripping | clear with `mks_release_protect()`; run `Cal` and check the mechanics |
| `ESP_ERR_INVALID_CRC` occasionally | long or unshielded UART wiring, or a ground loop |

## Layout

```
platformio.ini            build config, env:esp32dev
sdkconfig.defaults        ESP-IDF options (4 MB flash, full printf, console on UART0)
CMakeLists.txt            IDF project entry point
include/servo_config.h    wiring, baud, address, motor and demo settings
include/mks_servo42c.h    driver API
src/mks_servo42c.c        protocol implementation
src/main.c                demo application
```

## References

- [MKS-SERVO42C wiki](https://github.com/makerbase-mks/MKS-SERVO42C/wiki)
- [Serial communication description](https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description)
- [MKS SERVO42C V1.1.2 User Manual (PDF)](https://vallder.com/wp-content/uploads/2024/06/MKS-SERVO42C-User-Manual-V1.1.2.pdf)
