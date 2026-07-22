# MK1-Tank

Firmware for a custom LEGO Technic tank, built around an ESP32-S3 touchscreen
board that reads an Xbox Wireless Controller (XWC) over Bluetooth and drives
up to three Mould King Bluetooth hubs over the Mould King 6.0 (MK6) RC
protocol. The board's touchscreen doubles as a live dashboard and an
on-device input-mapping editor - no companion app, no PC required.

<img width="1744" height="1308" alt="3" src="https://github.com/user-attachments/assets/3c911158-6c11-400b-8fd7-f664f6581103" />
<img width="2119" height="1589" alt="7" src="https://github.com/user-attachments/assets/f4ea0132-50ef-4059-b91d-7a724ab5dfa2" />

## Acknowledgments

MK1-Tank stands on the work of others, gratefully acknowledged:

- **The Mould King protocol reverse-engineering lineage** — this firmware's hub protocol
  (`mkh_protocol`) was ported from and cross-checked byte-for-byte against three
  independent references, all of which agree on the telegram format:
  - **[J0EK3R](https://github.com/J0EK3R)**'s
    [mkconnect-python](https://github.com/J0EK3R/mkconnect-python)
  - **[vicocz](https://github.com/vicocz)**'s Mould King implementation for
    [BrickController2](https://github.com/imurvai/brickcontroller2)
    (original app by **[imurvai](https://github.com/imurvai)**)
  - **[Espruino](https://github.com/espruino)**'s
    [mouldking.js](https://github.com/espruino/EspruinoDocs/blob/master/devices/mouldking.js)
    module
- **[Bluepad32](https://github.com/ricardoquesada/bluepad32)** by Ricardo Quesada — the
  Bluetooth gamepad library that makes the Xbox Wireless Controller link possible, and the
  [project template](https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template)
  this firmware was built from.
- **[BTstack](https://github.com/bluekitchen/btstack)** by BlueKitchen — the underlying
  Bluetooth stack.
- **Waveshare** — for the [ESP32-S3-Touch-LCD-2](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2)
  board and its documentation, schematics, and demo code.

Full license details for third-party components are in [NOTICE](NOTICE) and [LICENSE](LICENSE).

## Why this project exists

I built MK1-Tank because phone-based control of Mould King 6.0 hubs has practical
limits — not because the apps are bad, but because they run on phones.

BrickController2 did the pioneering work and proved these hubs can be driven from a
gamepad via BLE advertising telegrams. But a phone app has to live with the phone's
rules:

- **The screen must stay on.** The hubs need a continuous telegram stream. When the
  screen turns off or the app goes to the background, the OS can throttle or suspend
  Bluetooth broadcasting and the model stops mid-run.
- **Portability.** In the field you're managing a gamepad plus a phone that has to
  stay awake, unlocked, and in range. One more battery, one more screen.

MK1-Tank removes the phone. A dedicated ESP32-S3 board with its own touchscreen
pairs directly with an Xbox Wireless Controller and broadcasts the hub telegrams
itself. Configuration happens on the touchscreen: no phone, no PC, no cables.
<img width="2165" height="1624" alt="12" src="https://github.com/user-attachments/assets/0465fbc4-c844-4f62-b30d-54845e8178d1" />
<img width="2229" height="1672" alt="13" src="https://github.com/user-attachments/assets/bdc216de-f6ed-4093-969e-31689ba0cb9a" />

### Design philosophy

I'm a former Program Implementation Manager at Amazon, and this build follows the
Invent and Simplify principle: don't work around a limitation, remove the component
that causes it.

- **Remove the phone** and the screen-off problem doesn't need solving.
- **Right-size the power:** a larger LiPo for the motor hubs, a small dedicated cell
  for the ESP32. Each system has its own energy budget.
- **Buy headroom, not features:** the board's QMI8658 IMU (accelerometer + gyroscope)
  is unused for now. It's there for future turret stabilization and motion-aware
  upgrades.

Every release was specified, bench-tested, and accepted on physical hardware before
it was tagged.
<img width="1974" height="1481" alt="5" src="https://github.com/user-attachments/assets/f987ada8-c4c5-4ed0-bad7-aaa75daa9b8d" />

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
  proportional/momentary/latched/pulse mode, all from the touchscreen. A
  confirm-gated reset-all wipes every binding back to a blank slate.
- Config is saved to internal flash (LittleFS) and survives reboot/power loss.
- Dashboard shows live per-hub broadcast state, XWC connection state, and
  onboard battery voltage (good/low/critical); an idle screen kicks in
  after 30s of no touch input with nothing connected.
  <img width="2151" height="1613" alt="11" src="https://github.com/user-attachments/assets/40f2a92c-8b3d-4e05-9959-4a50f029c761" />

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
HUB<n>_PORT_<letter> = <input> [invert=yes|no] [max=0-100] [curve=linear] [mode=proportional|momentary|latched|pulse]
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
  `momentary` (default for buttons - port follows the press), `latched`
  (each press toggles the port's output and holds it, runtime-only state,
  always boots off), or `pulse` (each press-edge emits one fixed-duration
  full-level burst, then the port returns to neutral on its own - holding
  or releasing the input doesn't matter, and taps during an active pulse
  are ignored rather than queued/retriggering/extending it; duration is a
  compile-time constant, `MKH_PULSE_FRAMES` in `main/sketch.cpp`, 4 frames
  of 100ms = 400ms by default, meant to be bench-tuned). Editor: `pulse` is
  offered for button-class inputs only, not stick axes.

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
<img width="2175" height="1631" alt="10" src="https://github.com/user-attachments/assets/467b67f2-1099-4a17-98f2-0b538aad81a1" />
<img width="2188" height="1641" alt="9" src="https://github.com/user-attachments/assets/f75519ae-60ac-4eb0-b415-7b787012c544" />
<img width="2064" height="1548" alt="8" src="https://github.com/user-attachments/assets/733f1f18-d6d1-49e2-a2a5-7c42a12bca3e" />

<img width="2011" height="1508" alt="6" src="https://github.com/user-attachments/assets/7bc48f76-6da9-4750-a8e3-940e5cb800be" />

<img width="1569" height="2092" alt="4" src="https://github.com/user-attachments/assets/9dae4bde-7768-432f-a2a7-b1a5a4e51666" />

<img width="2326" height="1744" alt="2" src="https://github.com/user-attachments/assets/e993930e-7e35-4c13-975d-623cd1af487a" />
<img width="1732" height="1299" alt="1" src="https://github.com/user-attachments/assets/7c274f2b-8c17-44a8-b2ad-aea0b6b140c2" />

<img width="2174" height="1630" alt="14" src="https://github.com/user-attachments/assets/536dc834-4938-4620-8263-250b42fb2437" />
<img width="2109" height="1582" alt="15" src="https://github.com/user-attachments/assets/55d0a214-b018-4dc4-bb4e-c30567423125" />
