# Endeavour Tank Receiver

Receiver / vehicle firmware for the **MARTYX ENDEAVOUR** tracked tank.
Listens for joystick + light commands from the **Prometheus Controller**
over an nRF24L01 radio link and drives:

- **Two tracked motors** through H-bridges (PWM + direction pins)
- **Two MOSFET-switched light groups** (front headlights, rear lights)
- **16×4 I²C LCD** for live status

This firmware is the matching half of the
[`prometheus-controller-v3`](https://github.com/MartinBugar/prometheus-controller-v3)
transmitter. It is intentionally a "dumb executor" — all the user-facing
logic (mode selection, navbar, light blink patterns) lives on the
controller. The tank just executes whatever the packet says.

---

## Quick reference

| Behavior                              | When                                                |
|---------------------------------------|-----------------------------------------------------|
| Failsafe (motors ramp to 0, lights off)| No packet for 1 s (`SIGNAL_TIMEOUT_MS`)            |
| Watchdog reset                        | `loop()` stalls for more than `WDT_TIMEOUT` (4 s)   |
| Front lights on                       | Controller's S1 active                              |
| Rear lights on                        | Controller's S2 active                              |
| Front blink                           | Controller's S1 + S3 active                         |
| Rear blink                            | Controller's S2 + S4 active                         |
| LCD shows `*NO SIGNAL **`             | Failsafe is engaged                                 |

---

## Features

- **Failsafe** — motors smoothly ramp to 0 (over `FAILSAFE_RAMP_MS` =
  200 ms) and lights go off if no packet arrives for 1 s. Less stress
  on the gearbox than a hard cut.
- **Watchdog timer** — AVR auto-resets if `loop()` ever stalls for more
  than 4 seconds. Keeps the tank from sitting at full throttle if the
  firmware deadlocks.
- **Slew-rate-limited motor PWM** — actual PWM is ramped instead of
  jumped (`SLEW_STEP / SLEW_INTERVAL_MS = 1000 PWM/s`). On a direction
  reversal the slew passes through zero, doubling as an
  anti-shoot-through guard for the H-bridge.
- **Quadratic response curve** — finer control near center, full power
  at the extremes (toggleable via `USE_QUADRATIC`).
- **Per-side mirror compensation** — `INVERT_LEFT` / `INVERT_RIGHT`
  flags fix mirrored motor mounts in firmware so you don't need to
  rewire.
- **Two-way ACK telemetry** — every received packet is acknowledged
  with a `VehicleTelemetry` ACK payload carrying uptime, boot count,
  failsafe count, and the vehicle's unique UUID. The controller can
  display these live and detect "I'm talking to a different tank".
- **Persistent stats in EEPROM** — boot counter, total uptime, failsafe
  counter, and the vehicle UUID. Auto-flushed every 5 minutes.
- **Per-vehicle UUID** — generated once at first boot from ADC noise on
  a floating analog pin, persisted in EEPROM. Used by the controller's
  pairing system as a unique vehicle identifier.

---

## Hardware

| Component                | Notes                                          |
|--------------------------|------------------------------------------------|
| Arduino Mega 2560        | ENDEAVOUR_TITANBOARD_V3 PCB                    |
| nRF24L01 radio           | Same model as the controller                   |
| 2 × H-bridge motor driver| Each takes IN1 / IN2 + PWM (e.g. L298, BTS7960)|
| 2 × MOSFET load switch   | One per light group, driven by a digital pin   |
| 16×4 I²C LCD             | PCF8574 backpack, address `0x27`               |
| 3 × front white LED      | Switched together via the front MOSFET (D38)   |
| 2 × rear red LED         | Switched together via the rear MOSFET (D39)   |
| Tracked-vehicle chassis  | Two independently driven tracks                |
| Battery                  | Sized for the motors                           |

### Pin assignments (Arduino Mega 2560)

```
Motor PWM (must be PWM-capable pins)
  RIGHT track PWM  -> D5
  LEFT  track PWM  -> D6

Motor direction flags  ->  H-bridge IN1 / IN2 inputs
  Right forward (vpred) -> D22
  Right reverse (vzad)  -> D23
  Left  forward (vpred) -> D24
  Left  reverse (vzad)  -> D25

Light groups (MOSFET load switches)
  Front lights (3x white LED) -> D38   (AD4184_module2 on the PCB)
  Rear  lights (2x red   LED) -> D39   (AD4184_module1)

LCD (I²C)
  SDA -> pin 20  (Mega — NOT A4)
  SCL -> pin 21

nRF24L01
  CE  -> D7
  CSN -> D8
  SCK / MOSI / MISO -> Mega SPI bus (D52 / D51 / D50)
  VCC -> 3.3 V (NOT 5 V!)
  GND -> GND

UUID seed
  A0 (left floating; sampled once at first boot for the random seed
     used to generate the vehicle UUID)
```

---

## Required libraries

| Library             | Version tested | Purpose                              |
|---------------------|----------------|--------------------------------------|
| `RF24`              | 1.6.0          | nRF24L01 driver, ACK payloads        |
| `LiquidCrystal_I2C` | 1.1.2          | 16×4 I²C LCD driver                  |
| `SPI`               | bundled        | nRF24L01 SPI                         |
| `EEPROM`            | bundled        | Persistent stats and UUID            |
| `avr/wdt`           | bundled        | Watchdog timer                       |

Compile target: `Arduino Mega or Mega 2560` (`arduino:avr:mega`),
processor `ATmega2560`.

---

## Build & flash

```bash
arduino-cli compile --fqbn arduino:avr:mega Endeavour.ino
arduino-cli upload  --fqbn arduino:avr:mega -p COM5 Endeavour.ino
```

Or just open `Endeavour.ino` in the Arduino IDE.

---

## How it works

### Packet format (must match the controller)

```c
// Inbound (controller -> tank)
struct Signal {
  byte rightTracks;   // 0..255, 127 = center
  byte leftTracks;    // 0..255, 127 = center
  byte frontLights;   // 0/1
  byte rearLights;    // 0/1
};

// ACK payload (tank -> controller)
struct VehicleTelemetry {
  uint32_t uptimeSec;       // tank uptime since power-on
  uint16_t bootCount;       // total boots (EEPROM-persisted)
  uint16_t failsafeCount;   // total link-loss events (EEPROM-persisted)
  uint32_t vehicleUuid;     // unique-per-tank ID (EEPROM-persisted)
};
```

The controller sends a `Signal` ~50 times per second. The tank ACKs each
packet with a `VehicleTelemetry` payload, queued via
`radio.writeAckPayload()` after every successful receive.

### Joystick byte → motor PWM pipeline

For each track, the byte from the controller (0..255, 127=neutral) goes
through:

1. **Center subtraction** — `centered = byte - 127` (range −128..+128)
2. **Dead-band cut** — values within `±DEAD_BAND` (5) become 0
3. **Optional quadratic curve** — `y = x² / xmax` if `USE_QUADRATIC=1`
   so the same stick travel feels gentler near center
4. **Floor + scale** — mapped onto `[MIN_MOTOR_PWM, MAX_MOTOR_PWM]`
5. **Sign + INVERT** — gives a signed PWM value (`+` = "natural
   forward", `−` = "natural reverse") with mirror compensation per side
6. **Slew-rate limit** — actual PWM walks toward target at most
   `SLEW_STEP` per `SLEW_INTERVAL_MS` (1000 PWM/s normal, ~1300 PWM/s
   in failsafe ramp-down)
7. **`applyTrack()`** — drives the H-bridge: signed PWM > 0 ⇒ FWD pin
   HIGH; < 0 ⇒ REV pin HIGH; = 0 ⇒ both LOW + PWM 0

### Failsafe behavior

If no packet has arrived for `SIGNAL_TIMEOUT_MS` (1 s):

1. `failsafed = true` — counter is bumped (and persisted) on the
   `false → true` edge
2. `data` is reset to neutral (127, 127, 0, 0)
3. The slew rate switches to `FAILSAFE_SLEW_STEP` (faster), so motors
   wind down to 0 in ~`FAILSAFE_RAMP_MS` (200 ms) instead of the normal
   ~250 ms — a smooth ramp, not a hard stop
4. Lights go off
5. The LCD switches to:
   ```
   *** NO SIGNAL **
   Tank STOP
   Lights OFF
   ```

When packets resume, `failsafed = false`, the slew rate returns to
normal, and the joystick takes over again.

### LCD layout (normal operation)

16×4 characters. Every row is fully rewritten on each refresh
(`LCD_UPDATE_MS` = 200 ms) so stale chars from the previous frame are
overwritten — no `lcd.clear()` flicker.

```
P=xxx d=xxx Sp=x       row 0  P = right joystick byte, d = right fwd PWM,
                              Sp = front lights
      z=xxx            row 1  z = right reverse PWM
L=xxx d=xxx Sz=x       row 2  L = left  joystick byte, d = left  fwd PWM,
                              Sz = rear  lights
      z=xxx            row 3  z = left  reverse PWM
```

The `INVERT_*` flags are *undone* before display so "fwd" / "rev" reflects
the tank-frame direction the operator perceives, not the motor frame.

---

## Pairing (vehicle side)

The tank generates a random 32-bit UUID at its first boot if EEPROM is
fresh (all `0xFF`). The seed comes from a single `analogRead(A0)` (left
floating) XORed with `micros()` so two freshly-flashed tanks won't
collide.

The UUID is then:

- Persisted in EEPROM at addr 8..11
- Sent in **every** ACK telemetry packet as `VehicleTelemetry.vehicleUuid`
- Never changes again, even across firmware reflashes (unless EEPROM is
  also wiped — see below)

The controller stores its own per-mode "paired UUID" and uses the
on-air UUID to detect "different vehicle". See the controller README for
the user-facing pairing flow.

### Forcing a new UUID on the tank side

Useful if you want to "pretend to be a fresh tank" to a controller that
has already paired with you.

1. Flash a sketch that calls
   `EEPROM.update(EEPROM_ADDR_VEHICLE_UUID + i, 0xFF)` for `i = 0..3`,
   then reset.
2. Or wipe the entire EEPROM via Arduino IDE → Tools → Burn Bootloader.
3. On the next boot, the tank firmware will detect the empty cell,
   roll a new random UUID, and persist it.

---

## EEPROM layout

```
addr   size   purpose
-----  -----  -----------------------------------------------------------
   0     2    Boot counter            (uint16_t LE)
   2     4    Total uptime in seconds (uint32_t LE)
   6     2    Failsafe counter        (uint16_t LE)
   8     4    Vehicle UUID            (uint32_t LE) -- generated once
```

A fresh chip (all `0xFF`) is detected and treated as zero / defaults.
Uptime is flushed every 5 minutes (`UPTIME_SAVE_INTERVAL_MS`) to limit
EEPROM wear. The failsafe counter is flushed on every link drop.

---

## Radio link

| Parameter        | Value                                |
|------------------|--------------------------------------|
| Channel          | 108                                  |
| Power level      | `RF24_PA_HIGH`                       |
| Data rate        | 250 kbps                             |
| Pipe             | `0xE9E8F0F0E1` (matches controller)  |
| Auto-ACK         | enabled, with payload                |
| Dynamic payloads | enabled (required for ACK payloads)  |
| Inbound packet   | `Signal` (4 bytes)                   |
| ACK payload      | `VehicleTelemetry` (12 bytes)        |

These constants **must match the controller firmware byte-for-byte**.

---

## Tweakable parameters (top of `Endeavour.ino`)

| `#define`                | Default | What it does                                       |
|--------------------------|---------|----------------------------------------------------|
| `FW_VERSION`             | string  | Firmware version tag (logged at compile time)      |
| `JOY_CENTER`             | 127     | Joystick byte at rest (must match controller)      |
| `DEAD_BAND`              | 5       | Joystick units treated as "stick released"         |
| `MIN_MOTOR_PWM`          | 50      | Minimum PWM at which the motor actually spins      |
| `MAX_MOTOR_PWM`          | 255     | Hard upper PWM limit                               |
| `USE_QUADRATIC`          | 1       | 1 = quadratic curve, 0 = linear                    |
| `INVERT_LEFT`            | 0       | Flip 0↔1 if the left motor spins the wrong way     |
| `INVERT_RIGHT`           | 1       | Same, for the right motor (this PCB needs it)      |
| `SLEW_STEP`              | 20      | PWM units the slew can move per interval           |
| `SLEW_INTERVAL_MS`       | 20      | How often the slew runs                            |
| `FAILSAFE_RAMP_MS`       | 200     | Target time for motors to ramp to 0 in failsafe    |
| `FAILSAFE_SLEW_STEP`     | 26      | PWM units per interval during failsafe ramp        |
| `SIGNAL_TIMEOUT_MS`      | 1000    | No-packet threshold before failsafe engages        |
| `LCD_UPDATE_MS`          | 200     | LCD refresh period                                 |
| `WDT_TIMEOUT`            | WDTO_4S | Watchdog reset window                              |
| `UPTIME_SAVE_INTERVAL_MS`| 300000  | EEPROM uptime flush interval (5 min)               |

---

## Light combinations

The tank only ever sees `frontLights` and `rearLights` as binary 0/1
bytes — all the smarts (S1+S3 = blink, etc.) live in the controller. So
the tank's behavior is just:

| Packet says    | Front MOSFET pin | Rear MOSFET pin |
|----------------|------------------|-----------------|
| `frontLights=0`| LOW (off)        | (unchanged)     |
| `frontLights=1`| HIGH (on)        | (unchanged)     |
| `rearLights=0` | (unchanged)      | LOW (off)       |
| `rearLights=1` | (unchanged)      | HIGH (on)       |

When the controller is blinking a light, it just toggles the byte at
1 Hz between 0 and 1 in the outgoing packet — the tank faithfully
mirrors that on the LED. No tank firmware update needed if the
controller adds new blink patterns later.

See the [controller README](../../Ovladanie/README.md) for the full
S1/S2/S3/S4 behavior table.

---

## Troubleshooting

| Symptom                                      | Likely cause / fix                                                     |
|----------------------------------------------|------------------------------------------------------------------------|
| Both motors spin one direction is reversed   | Toggle `INVERT_LEFT` / `INVERT_RIGHT` and reflash                      |
| Tank moves opposite of every joystick action | Verify the controller's `mapJoystickValues(..., true)` `reverse` flag  |
| LCD blank or garbled                         | Wiring (SDA→20, SCL→21 on Mega), or contrast pot on the I²C backpack   |
| Front lights don't come on                   | MOSFET wiring, check D38 actually toggles with a multimeter            |
| Rear lights don't come on                    | Same, check D39                                                        |
| Continuous boot loop                         | Watchdog tripping — something in `loop()` is taking > 4 s              |
| `*NO SIGNAL **` while controller is on       | Channel / pipe / data rate mismatch, low battery, or out of range      |
| Failsafe count climbs even at close range    | Radio interference; check the controller's DEBUG → Tank stats          |
| Tank works but UUID can't be paired          | Reflash the controller — its EEPROM may be corrupt (use FORGET in INFO)|

---

## File layout

```
.
├── Endeavour.ino    # Firmware (single sketch)
├── README.md   # This file
└── .gitignore
```
