/*
 * TANK Receiver -- ENDEAVOUR_TITANBOARD_V3 (Arduino Mega 2560)
 *
 * Receives joystick + light commands from the handheld transmitter
 * (Ovladanie) over an nRF24L01 link and drives:
 *   - Two tracked motors via H-bridges (PWM + direction pins)
 *   - Two MOSFET-switched light groups (front / rear)
 *   - 16x4 I2C LCD for live status
 *
 * Safety / robustness features:
 *   - Failsafe: motors stop and lights go off if no packet arrives for
 *     SIGNAL_TIMEOUT_MS.
 *   - Watchdog timer: the AVR auto-resets if loop() ever stalls.
 *   - Slew-rate limiter: motor PWM is ramped instead of jumped, which also
 *     creates a natural anti-shoot-through brake on direction reversal
 *     because the current PWM passes through zero.
 */

#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <avr/wdt.h>


// =============================================================================
// FIRMWARE VERSION
// =============================================================================
#define FW_VERSION  "PRMTS2026FW1A1"


// =============================================================================
// PIN ASSIGNMENTS  (matched to the ENDEAVOUR_TITANBOARD_V3 PCB)
// =============================================================================

// Motor PWM (must be PWM-capable pins on the Mega)
#define LEFT_TRACK_PWM_PIN    6
#define RIGHT_TRACK_PWM_PIN   5

// Motor direction flags -> H-bridge IN1/IN2 inputs
#define R_FLAG_VPRED_PIN     22   // right track forward
#define R_FLAG_VZAD_PIN      23   // right track reverse
#define L_FLAG_VPRED_PIN     24   // left  track forward
#define L_FLAG_VZAD_PIN      25   // left  track reverse

// Light groups -- each pin drives one MOSFET load switch on the PCB.
//   D38 -> AD4184_module2 -> 3x front LED (LED-F-1, LED-F-2, LED-F-3)
//   D39 -> AD4184_module1 -> 2x rear  LED (LED_B_1, LED_B_2)
#define FRONT_LIGHTS_PIN     38
#define REAR_LIGHTS_PIN      39


// =============================================================================
// CONTROL FEEL TUNING
// =============================================================================

// Joystick value at rest (must match the transmitter's center).
#define JOY_CENTER          127

// Dead band around center, in joystick units. Anything inside +/- DEAD_BAND
// counts as "stick released" and the motor stays off.
#define DEAD_BAND             5

// Minimum PWM at which the motor actually starts spinning. Below this it
// only hums but does not move. Tune to your motors / battery voltage.
#define MIN_MOTOR_PWM        50
#define MAX_MOTOR_PWM       255

// 1 = quadratic response (gentle near center, full at the extremes).
// 0 = plain linear response.
#define USE_QUADRATIC         1

// Mirror compensation. If a motor spins the wrong way relative to the tank
// (typical for one mirrored side), flip 0 <-> 1 instead of rewiring.
#define INVERT_RIGHT          1
#define INVERT_LEFT           0

// Slew rate: how fast the actual PWM is allowed to chase the target PWM.
//   SLEW_STEP / SLEW_INTERVAL_MS = max PWM units per second
//   20 / 20 ms = 1000 PWM/s, i.e. ~250 ms from full-stop to full-throttle.
// On a direction reversal the slew passes through zero, so this also
// doubles as an anti-shoot-through delay for the H-bridge.
#define SLEW_INTERVAL_MS     20
#define SLEW_STEP            20

// Failsafe ramp: when the link is lost we ramp motors to 0 in this many
// milliseconds (instead of an abrupt cut). Less stress on gearbox + motor.
//   FAILSAFE_SLEW_STEP = (MAX_MOTOR_PWM * SLEW_INTERVAL_MS) / FAILSAFE_RAMP_MS
//   ~ 200 ms from full-throttle to stop.
#define FAILSAFE_RAMP_MS    200
#define FAILSAFE_SLEW_STEP   26   // 255 PWM / (200 ms / 20 ms) = 25.5 -> 26


// =============================================================================
// TIMING / FAILSAFE
// =============================================================================

// If no packet has arrived for this long, enter failsafe (motors stop,
// lights off, LCD shows NO SIGNAL).
#define SIGNAL_TIMEOUT_MS  1000

// LCD refresh interval. The I2C transactions are slow, so refreshing less
// often leaves more headroom for the radio.
#define LCD_UPDATE_MS       200

// Watchdog. If loop() never reaches wdt_reset() within this window, the
// AVR resets itself. 4 s is comfortably longer than bootloader + setup().
#define WDT_TIMEOUT       WDTO_4S


// =============================================================================
// RADIO  (these MUST match the transmitter byte-for-byte)
// =============================================================================

// Channel 108 -> ~2.508 GHz, just above the standard 2.4 GHz WiFi range.
#define RADIO_CHANNEL       108

// Output power: MIN / LOW / HIGH / MAX. Higher = more range, more current.
#define RADIO_PA_LEVEL      RF24_PA_HIGH

// Lower bitrate = better range and noise immunity, at the cost of latency.
#define RADIO_DATA_RATE     RF24_250KBPS


// =============================================================================
// EEPROM LAYOUT  --  persistent stats reported back as ACK telemetry.
// =============================================================================
//
//   addr   size   purpose
//   -----  -----  -------------------------------------------------------------
//      0     2    boot counter (uint16_t LE)
//      2     4    total uptime in seconds (uint32_t LE)
//      6     2    failsafe count (uint16_t LE)
//      8     4    vehicle UUID (uint32_t LE) -- generated once at first boot,
//                 sent in every telemetry packet so the controller can warn
//                 if it accidentally talks to a different vehicle.
//
#define EEPROM_ADDR_BOOT_COUNT     0
#define EEPROM_ADDR_UPTIME         2
#define EEPROM_ADDR_FAILSAFE_CNT   6
#define EEPROM_ADDR_VEHICLE_UUID   8

#define UPTIME_SAVE_INTERVAL_MS  300000UL  // 5 min between EEPROM uptime saves

// Pin used as a noise source to seed random() at first boot. Must NOT be
// driven by anything (left floating).
#define UUID_SEED_PIN              A0


// =============================================================================
// HARDWARE OBJECTS & STATE
// =============================================================================

LiquidCrystal_I2C lcd(0x27, 16, 4);
RF24              radio(7, 8);                  // CE, CSN
const uint64_t    pipeIn = 0xE9E8F0F0E1LL;      // shared with the transmitter

// Wire packet from the transmitter (must match the transmitter struct).
struct Signal {
  byte rightTracks;   // 0..255, 127 = center
  byte leftTracks;    // 0..255, 127 = center
  byte frontLights;   // 0/1
  byte rearLights;    // 0/1
};

// Telemetry packet pushed back to the transmitter as the auto-ACK payload.
// MUST match the transmitter's VehicleTelemetry struct byte-for-byte.
// Generic name -- this firmware happens to run the ENDEAVOUR tank, but
// future receivers (drone, boat, ...) can reuse the same packet shape.
struct VehicleTelemetry {
  uint32_t uptimeSec;       // vehicle uptime since power-on
  uint16_t bootCount;       // total boots (EEPROM-persisted)
  uint16_t failsafeCount;   // total times the link dropped (EEPROM-persisted)
  uint32_t vehicleUuid;     // unique-per-vehicle ID (EEPROM-persisted)
};

Signal           data;
VehicleTelemetry telem;

// Time bookkeeping (millis-based, non-blocking).
unsigned long lastRecvTime    = 0;
unsigned long lastLcdUpdate   = 0;
unsigned long lastSlewTime    = 0;
unsigned long lastUptimeTick  = 0;
unsigned long lastUptimeSave  = 0;
bool          failsafed       = false;
bool          prevFailsafed   = false;     // edge detect for failsafe counter

// Persistent stats (loaded from EEPROM at boot, refreshed in telem).
uint16_t      bootCount       = 0;
uint32_t      totalUptimeSec  = 0;
uint16_t      failsafeCount   = 0;
uint32_t      vehicleUuid     = 0;

// Signed PWM values:
//   target  -- where the joystick wants the motor to be
//   current -- where the motor actually is, after slew-rate limiting
// Sign convention (after INVERT is applied):
//   > 0  = "natural forward" of the motor  -> VPRED pin HIGH
//   < 0  = "natural reverse"                -> VZAD  pin HIGH
//     0  = stop (both direction pins LOW, PWM=0)
int rTargetPwm  = 0;
int rCurrentPwm = 0;
int lTargetPwm  = 0;
int lCurrentPwm = 0;


// =============================================================================
// EEPROM HELPERS
// =============================================================================

uint16_t eepromReadU16(int addr) {
  return (uint16_t)EEPROM.read(addr) |
         ((uint16_t)EEPROM.read(addr + 1) << 8);
}
void eepromWriteU16(int addr, uint16_t v) {
  EEPROM.update(addr,     v & 0xFF);
  EEPROM.update(addr + 1, (v >> 8) & 0xFF);
}

uint32_t eepromReadU32(int addr) {
  return  (uint32_t)EEPROM.read(addr) |
         ((uint32_t)EEPROM.read(addr + 1) << 8)  |
         ((uint32_t)EEPROM.read(addr + 2) << 16) |
         ((uint32_t)EEPROM.read(addr + 3) << 24);
}
void eepromWriteU32(int addr, uint32_t v) {
  EEPROM.update(addr,     v & 0xFF);
  EEPROM.update(addr + 1, (v >> 8)  & 0xFF);
  EEPROM.update(addr + 2, (v >> 16) & 0xFF);
  EEPROM.update(addr + 3, (v >> 24) & 0xFF);
}


// =============================================================================
// PERSISTENT STATS
// =============================================================================

void incrementBootCount() {
  bootCount = eepromReadU16(EEPROM_ADDR_BOOT_COUNT);
  if (bootCount == 0xFFFF) bootCount = 0;
  bootCount++;
  eepromWriteU16(EEPROM_ADDR_BOOT_COUNT, bootCount);
}

void loadStats() {
  totalUptimeSec = eepromReadU32(EEPROM_ADDR_UPTIME);
  if (totalUptimeSec == 0xFFFFFFFFUL) totalUptimeSec = 0;
  failsafeCount = eepromReadU16(EEPROM_ADDR_FAILSAFE_CNT);
  if (failsafeCount == 0xFFFF) failsafeCount = 0;

  // Load vehicle UUID. If the cell is fresh (all 0xFF) or all 0x00,
  // generate a new random one and persist it. Use ADC noise from a
  // floating pin as the seed so two vehicles built from the same blank
  // chip won't collide.
  vehicleUuid = eepromReadU32(EEPROM_ADDR_VEHICLE_UUID);
  if (vehicleUuid == 0xFFFFFFFFUL || vehicleUuid == 0UL) {
    randomSeed((unsigned long)analogRead(UUID_SEED_PIN) ^ micros());
    vehicleUuid = ((uint32_t)random(0, 65536) << 16) |
                  ((uint32_t)random(0, 65536));
    if (vehicleUuid == 0UL) vehicleUuid = 1UL;     // never 0 -- 0 means "unset"
    eepromWriteU32(EEPROM_ADDR_VEHICLE_UUID, vehicleUuid);
  }
}

void saveUptime() {
  eepromWriteU32(EEPROM_ADDR_UPTIME, totalUptimeSec);
}

void saveFailsafeCount() {
  eepromWriteU16(EEPROM_ADDR_FAILSAFE_CNT, failsafeCount);
}

// Once-per-second uptime accumulator + periodic EEPROM flush.
void updateUptime() {
  unsigned long now = millis();
  if (now - lastUptimeTick >= 1000UL) {
    uint32_t delta = (now - lastUptimeTick) / 1000UL;
    totalUptimeSec += delta;
    lastUptimeTick += delta * 1000UL;
  }
  if (now - lastUptimeSave >= UPTIME_SAVE_INTERVAL_MS) {
    saveUptime();
    lastUptimeSave = now;
  }
}

// Refresh the telemetry struct and queue it for the next ACK so the
// transmitter sees fresh values on every successful packet.
void updateTelem() {
  telem.uptimeSec     = totalUptimeSec;
  telem.bootCount     = bootCount;
  telem.failsafeCount = failsafeCount;
  telem.vehicleUuid   = vehicleUuid;
  radio.writeAckPayload(1, &telem, sizeof(telem));
}


// =============================================================================
// HELPERS
// =============================================================================

// Reset 'data' to a safe neutral state. Used at startup and on signal loss.
void ResetData() {
  data.rightTracks = JOY_CENTER;
  data.leftTracks  = JOY_CENTER;
  data.frontLights = 0;
  data.rearLights  = 0;
}

// Drain the radio RX FIFO into 'data', refresh the failsafe timestamp,
// and queue an updated telemetry packet for the next auto-ACK reply.
void recvData() {
  while (radio.available()) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();
    updateTelem();   // load fresh telem for the next ACK back to the controller
  }
}

// Map a raw joystick byte to a signed target PWM.
//
// Pipeline:
//   1. Center around JOY_CENTER  -> 'centered' in [-128 .. +128]
//   2. Apply DEAD_BAND           -> ignore tiny stick deflections
//   3. Optional quadratic curve  -> finer control near center
//   4. Lift floor to MIN_MOTOR_PWM and stretch to MAX_MOTOR_PWM
//   5. Apply sign + INVERT flag  -> signed PWM
//
// Sign convention is determined empirically together with the transmitter
// settings (reverse=true on the joystick map): stick pushed forward produces
// the byte interpreted as "forward" after these flips.
int computeSignedPwm(byte joyVal, bool invert) {
  int centered = (int)joyVal - JOY_CENTER;
  int absC     = (centered < 0) ? -centered : centered;

  if (absC <= DEAD_BAND) {
    return 0;
  }

  int x    = absC - DEAD_BAND;        // 1 .. (128 - DEAD_BAND)
  int xmax = 128  - DEAD_BAND;

#if USE_QUADRATIC
  // y = x^2 / xmax. Endpoints (0 and xmax) are preserved, mid-range values
  // are pushed downward, giving a softer response near center.
  int y = ((long)x * x) / xmax;
#else
  int y = x;
#endif

  // Stretch [0..xmax] onto [MIN_MOTOR_PWM..MAX_MOTOR_PWM].
  int pwm = (int)(MIN_MOTOR_PWM
                  + ((long)(MAX_MOTOR_PWM - MIN_MOTOR_PWM) * y) / xmax);
  if (pwm > MAX_MOTOR_PWM) pwm = MAX_MOTOR_PWM;
  if (pwm < MIN_MOTOR_PWM) pwm = MIN_MOTOR_PWM;

  if (centered > 0) pwm = -pwm;   // stick pulled back -> negative
  if (invert)       pwm = -pwm;   // mirror compensation
  return pwm;
}

// Move 'current' toward 'target' by at most `step`. Used to ramp the
// PWM smoothly instead of letting it jump. The caller chooses the step
// (normal vs. accelerated failsafe).
int slewToward(int current, int target, int step) {
  if (current < target) {
    int next = current + step;
    return (next > target) ? target : next;
  }
  if (current > target) {
    int next = current - step;
    return (next < target) ? target : next;
  }
  return current;
}

// Drive one track from a signed PWM value:
//   > 0 : forward direction pin HIGH, PWM = magnitude
//   < 0 : reverse direction pin HIGH, PWM = magnitude
//     0 : both direction pins LOW, PWM = 0  (coast/stop)
void applyTrack(int pwmPin, int fwdPin, int bckPin, int signedPwm) {
  if (signedPwm > 0) {
    digitalWrite(fwdPin, HIGH);
    digitalWrite(bckPin, LOW);
    analogWrite (pwmPin, signedPwm);
  } else if (signedPwm < 0) {
    digitalWrite(fwdPin, LOW);
    digitalWrite(bckPin, HIGH);
    analogWrite (pwmPin, -signedPwm);
  } else {
    digitalWrite(fwdPin, LOW);
    digitalWrite(bckPin, LOW);
    analogWrite (pwmPin, 0);
  }
}


// =============================================================================
// LCD DISPLAY
// =============================================================================

// Layout (16 columns x 4 rows). Every row is rewritten as exactly 16 chars
// so stale characters from a previous frame are fully overwritten -- no
// lcd.clear() needed and no flicker.
//
// Normal:
//   row 0:  P=xxx d=xxx Sp=x       (right joy / right fwd PWM / front lights)
//   row 1:        z=xxx            (right reverse PWM)
//   row 2:  L=xxx d=xxx Sz=x       (left  joy / left  fwd PWM / rear lights)
//   row 3:        z=xxx            (left reverse PWM)
//
// Failsafe (no signal):
//   *** NO SIGNAL **
//   Tank STOP
//   Lights OFF
//
void updateLcd() {
  char buf[17];

  if (failsafed) {
    lcd.setCursor(0, 0); lcd.print("*** NO SIGNAL **");
    lcd.setCursor(0, 1); lcd.print("Tank STOP       ");
    lcd.setCursor(0, 2); lcd.print("Lights OFF      ");
    lcd.setCursor(0, 3); lcd.print("                ");
    return;
  }

  // For the display, undo the INVERT flag so "fwd" / "rev" reflects the
  // tank-frame direction the operator perceives, not the motor frame.
  int rTank = INVERT_RIGHT ? -rCurrentPwm : rCurrentPwm;
  int lTank = INVERT_LEFT  ? -lCurrentPwm : lCurrentPwm;

  int rVpred = (rTank > 0) ?  rTank : 0;
  int rVzad  = (rTank < 0) ? -rTank : 0;
  int lVpred = (lTank > 0) ?  lTank : 0;
  int lVzad  = (lTank < 0) ? -lTank : 0;

  snprintf(buf, sizeof(buf), "P=%-3u d=%-3d Sp=%u",
           (unsigned)data.rightTracks, rVpred, (unsigned)data.frontLights);
  lcd.setCursor(0, 0); lcd.print(buf);

  snprintf(buf, sizeof(buf), "      z=%-3d     ", rVzad);
  lcd.setCursor(0, 1); lcd.print(buf);

  snprintf(buf, sizeof(buf), "L=%-3u d=%-3d Sz=%u",
           (unsigned)data.leftTracks, lVpred, (unsigned)data.rearLights);
  lcd.setCursor(0, 2); lcd.print(buf);

  snprintf(buf, sizeof(buf), "      z=%-3d     ", lVzad);
  lcd.setCursor(0, 3); lcd.print(buf);
}


// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // If we got here from a watchdog reset, the WDT may still be running with
  // a very short timeout. Clear MCUSR and disable WDT immediately, then
  // re-arm at the very end of setup() with our chosen timeout.
  MCUSR = 0;
  wdt_disable();

  lcd.init();
  lcd.backlight();

  pinMode(LEFT_TRACK_PWM_PIN,  OUTPUT);
  pinMode(RIGHT_TRACK_PWM_PIN, OUTPUT);
  pinMode(R_FLAG_VPRED_PIN,    OUTPUT);
  pinMode(R_FLAG_VZAD_PIN,     OUTPUT);
  pinMode(L_FLAG_VPRED_PIN,    OUTPUT);
  pinMode(L_FLAG_VZAD_PIN,     OUTPUT);
  pinMode(FRONT_LIGHTS_PIN,    OUTPUT);
  pinMode(REAR_LIGHTS_PIN,     OUTPUT);

  // Force outputs to a known stopped/off state before the first loop tick.
  analogWrite (LEFT_TRACK_PWM_PIN,  0);
  analogWrite (RIGHT_TRACK_PWM_PIN, 0);
  digitalWrite(R_FLAG_VPRED_PIN, LOW);
  digitalWrite(R_FLAG_VZAD_PIN,  LOW);
  digitalWrite(L_FLAG_VPRED_PIN, LOW);
  digitalWrite(L_FLAG_VZAD_PIN,  LOW);
  digitalWrite(FRONT_LIGHTS_PIN, LOW);
  digitalWrite(REAR_LIGHTS_PIN,  LOW);

  ResetData();

  // Persistent stats (must be ready before the first telem packet).
  incrementBootCount();
  loadStats();
  unsigned long now = millis();
  lastUptimeTick = now;
  lastUptimeSave = now;

  radio.begin();
  radio.setPALevel (RADIO_PA_LEVEL);
  radio.setDataRate(RADIO_DATA_RATE);
  radio.setChannel (RADIO_CHANNEL);

  // Two-way telemetry: the controller will receive a TankTelemetry struct
  // riding on every auto-ACK from us. enableDynamicPayloads() is required
  // for ACK payloads to work reliably.
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  radio.openReadingPipe(1, pipeIn);
  radio.startListening();

  // Pre-load the first ACK payload so the very first ACK already carries
  // valid telemetry instead of zeros.
  updateTelem();

  wdt_enable(WDT_TIMEOUT);
}


// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  wdt_reset();
  recvData();

  unsigned long now = millis();

  // ---- Failsafe check + edge-triggered counter ----
  failsafed = (now - lastRecvTime > SIGNAL_TIMEOUT_MS);
  if (failsafed && !prevFailsafed) {
    // Link just dropped -- bump the EEPROM-backed failsafe counter.
    failsafeCount++;
    saveFailsafeCount();
  }
  prevFailsafed = failsafed;
  if (failsafed) {
    ResetData();   // forces stick-to-center and lights off
  }

  // ---- Joystick -> target PWM ----
  rTargetPwm = computeSignedPwm(data.rightTracks, INVERT_RIGHT);
  lTargetPwm = computeSignedPwm(data.leftTracks,  INVERT_LEFT);

  // ---- Slew current PWM toward target ----
  // During failsafe we ramp 2-3x faster (FAILSAFE_SLEW_STEP) so the motors
  // wind down within ~FAILSAFE_RAMP_MS instead of the normal 250 ms.
  if (now - lastSlewTime >= SLEW_INTERVAL_MS) {
    lastSlewTime = now;
    int step     = failsafed ? FAILSAFE_SLEW_STEP : SLEW_STEP;
    rCurrentPwm  = slewToward(rCurrentPwm, rTargetPwm, step);
    lCurrentPwm  = slewToward(lCurrentPwm, lTargetPwm, step);
  }

  // ---- Drive motors ----
  applyTrack(RIGHT_TRACK_PWM_PIN, R_FLAG_VPRED_PIN, R_FLAG_VZAD_PIN, rCurrentPwm);
  applyTrack(LEFT_TRACK_PWM_PIN,  L_FLAG_VPRED_PIN, L_FLAG_VZAD_PIN, lCurrentPwm);

  // ---- Lights (binary on/off) ----
  digitalWrite(FRONT_LIGHTS_PIN, data.frontLights ? HIGH : LOW);
  digitalWrite(REAR_LIGHTS_PIN,  data.rearLights  ? HIGH : LOW);

  // ---- LCD (rate-limited so I2C doesn't starve the radio) ----
  if (now - lastLcdUpdate >= LCD_UPDATE_MS) {
    lastLcdUpdate = now;
    updateLcd();
  }

  // ---- Uptime accounting (also drives EEPROM flush every 5 min) ----
  updateUptime();
}
