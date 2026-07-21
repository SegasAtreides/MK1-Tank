# MK1-Tank

Firmware for a custom LEGO Technic tank, built around an ESP32-S3 touchscreen
board that reads an Xbox Wireless Controller (XWC) over Bluetooth and drives
up to three Mould King Bluetooth hubs over the Mould King 6.0 (MK6) RC
protocol. The board's touchscreen doubles as a live dashboard and an
on-device input-mapping editor - no companion app, no PC required.


## Acknowledgments

MK1-Tank stands on the work of others, gratefully acknowledged:

- **[Bluepad32](https://github.com/ricardoquesada/bluepad32)** by Ricardo Quesada — the
  Bluetooth gamepad library that makes the Xbox Wireless Controller link possible, and the
  [project template](https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template)
  this firmware was built from.
- **[BTstack](https://github.com/bluekitchen/btstack)** by BlueKitchen — the underlying
  Bluetooth stack.
- **Waveshare** — for the [ESP32-S3-Touch-LCD-2](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2)
  board and its documentation, schematics, and demo code.
- The **Mould King BLE protocol** reverse-engineering work shared by the LEGO/MK hobbyist
  community, without which no third-party controller could talk to these hubs.

Full license details for third-party components are in [NOTICE](NOTICE) and [LICENSE](LICENSE).

## How this project was built

MK1-Tank is a human–AI collaboration: a human project manager specified, bench-tested, and
accepted every feature, working with Claude (Anthropic) as research and implementation
manager and Claude Code as the coding agent. Every release was verified on physical
hardware before being tagged. The design decisions, the test rig, and the tank are human;
the keystrokes were shared.

## What it does

- Reads an Xbox Wireless Controller via [Bluepad32](https://github.com/ricardoquesada/bluepad32)/BTstack.
- Broadcasts the Mould King 6.0 telegram (`main/mkh_protocol.*`) to drive up
  to 3 Mould King hubs (6 output ports each) over non-connectable BLE
  advertising - no pairing needed on the hub side.
- Full on-device mapping editor: assign any of the 18 XWC inputs (both
  sticks, both triggers, all four face buttons, both bumpers, the D-pad, and
  both stick clicks) to any hub port - each port can carry several
  alternative bindings - with per-binding invert, output-range cap, and
  proportional/momentary/latched mode, all from the touchscreen. A
  confirm-gated reset-all wipes every binding back to a blank slate.
- Config is saved to internal flash (LittleFS) and survives reboot/power loss.
- Dashboard shows live per-hub broadcast state, XWC connection state, and
  onboard battery voltage (good/low/critical); an idle screen kicks in
  after 30s of no touch input with nothing connected.

## Hardware

| Component | Part |
|---|---|
| Main board | [Waveshare ESP32-S3-Touch-LCD-2](https://www.waveshare.com/esp32-s3-touch-lcd-2.htm) - ESP32-S3, 2" 320x240 ST7789 SPI LCD, CST816D capacitive touch |
| Hubs | Mould King Bluetooth hubs (MKH1/2/3), addressed by the blue-flash LED count set on each hub, 6 output ports (A-F) per hub |
| Controller | Any Xbox Wireless Controller (Bluetooth LE, tested against Xbox One-generation controllers) |

Only 2 of the 3 possible hubs can be actively driven at once - the MK6
protocol only allows one telegram on air at a time, so the firmware
time-slices between two active hubs; a third hub, if present, stays parked
(a deliberate product decision, not a bug).

## Building and flashing

This is a [PlatformIO](https://platformio.org/) + ESP-IDF (Arduino-as-a-
component) project.

```sh
# Build
pio run

# Build and flash (board connected over USB)
pio run --target upload

# Watch the serial log after flashing
pio device monitor --baud 115200
```

The default/only meaningful environment is `esp32-s3-devkitc-1` (see
`platformio.ini`); the other declared environments (`esp32dev`,
`esp32-c3-devkitc-02`, etc.) are inherited from the upstream template this
project was forked from and are not used or tested by MK1-Tank.

You'll see a `Flash memory size mismatch detected. Expected 8MB, found 4MB!`
warning on every build - harmless. The board's flash is 8MB, but
`sdkconfig.defaults` deliberately targets a 4MB `CONFIG_ESPTOOLPY_FLASHSIZE`
ceiling to match `partitions.csv`; PlatformIO just reports the board's
physical size against that target and moves on.

No config file needs to be flashed separately - on first boot (or after a
full chip erase), the firmware seeds `/mk1config.txt` on its internal
LittleFS partition with a compiled-in default mapping (left stick -> hub 1
port A, right stick -> hub 2 port A). From there, use the on-device editor
(tap the settings icon on the dashboard) to remap ports - no reflashing
needed for config changes.

## Config file format

`/mk1config.txt` lives on the board's internal LittleFS "storage" partition
(see `partitions.csv`) - not an SD card; an earlier microSD-based design was
retired (see `main/mkh_sdcard.h` for why) in favor of internal flash, which
is what every device in the field actually uses. The file is read once at
boot; there's no hot-reload. The on-device editor's Save writes it out for
you, but the format is documented here for anyone who wants to hand-edit or
inspect it (see `mk1config.sample.txt` for a runnable annotated example).

One line per **binding** - a port can carry up to 8 bindings (soft cap,
raisable), each an alternative way to reach that port ("bind several
controls, use them one at a time" - if you touch two at once their
outputs sum and clamp to +/-100%, but an idle binding always contributes
zero so the common case just works). Repeated lines for the same port
accumulate rather than replace:

```
HUB<n>_PORT_<letter> = <input> [invert=yes|no] [max=0-100] [curve=linear] [mode=proportional|momentary|latched]
```

- `HUB<n>` - app-facing hub number, 1-3, matching the hub's own blue-flash
  LED blink count (not the internal protocol device slot 0/1/2).
- `PORT_<letter>` - physical output port on the hub, A-F, matching the
  letters silkscreened on the hub.
- `invert` - `yes` flips the input's sign before mapping. Default `no`.
- `max` - caps the output range as a percentage of full scale, applied
  symmetrically around neutral. Default `100`.
- `curve` - reshapes the input-to-output response. Only `linear` (the
  default) is implemented; any other value is accepted but logged as
  unsupported and treated as linear.
- `mode` - only meaningful for non-axis inputs (triggers/buttons/D-pad/stick
  clicks); ignored for the two stick axes. `proportional` (triggers only),
  `momentary` (default for buttons - port follows the press), or `latched`
  (each press toggles the port's output and holds it, runtime-only state,
  always boots off).

Lines starting with `#` are comments (the file's own first line records a
schema version this way) and blank lines are ignored. Unknown keys or
malformed lines are logged and skipped; parsing continues with the rest of
the file, and any port without a valid line keeps its compiled-in default.
A missing partition, missing file, or malformed line never prevents boot -
only the affected port(s) fall back to defaults.

### Input tokens

The full set of tokens accepted for `<input>`, taken verbatim from the
parser's token table (`main/mkh_config.c`):

| Token | Input |
|---|---|
| `LSNS` | Left stick, Y axis (forward/back) |
| `LSEW` | Left stick, X axis (left/right) |
| `RSNS` | Right stick, Y axis (forward/back) |
| `RSEW` | Right stick, X axis (left/right) |
| `LT` | Left trigger |
| `RT` | Right trigger |
| `A` | A button |
| `B` | B button |
| `X` | X button |
| `Y` | Y button |
| `LB` | Left bumper |
| `RB` | Right bumper |
| `DUP` | D-pad up |
| `DDN` | D-pad down |
| `DLT` | D-pad left |
| `DRT` | D-pad right |
| `L3` | Left stick click |
| `R3` | Right stick click |

### Example

```
HUB1_PORT_A = LSNS invert=no max=100 curve=linear
HUB2_PORT_A = RSNS invert=no max=100 curve=linear
HUB1_PORT_B = RT max=75 mode=momentary
HUB1_PORT_C = A mode=latched
```

## Project layout

- `main/sketch.cpp` - Arduino-style entry point: dashboard, on-device
  editor UI, touch dispatch, idle screen.
- `main/mkh_protocol.*` - Mould King 6.0 telegram encoder (frozen; not
  modified by this project beyond the initial port).
- `main/mkh_broadcast.*` - BLE advertising/time-slicing layer that drives
  `mkh_protocol.*` against up to 3 hubs.
- `main/mkh_config.*` - `/mk1config.txt` parser/serializer and the live
  per-port mapping table.
- `main/mkh_storage.*` - LittleFS mount, boot-time config read, atomic save.
- `main/mkh_touch.*` - CST816D touch controller driver.

## License

Apache License 2.0 - see [LICENSE](LICENSE), consistent with the
`SPDX-License-Identifier: Apache-2.0` header already on every file under
`main/`. Third-party components (Bluepad32, BTstack, the Arduino core,
the display driver, etc.) keep their own original licenses - see
[NOTICE](NOTICE).

