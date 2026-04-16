/*
 * ============================================================
 * PROJECT:  SimplePilot — ESP32 Outboard Tiller Control
 * VERSION:  2.12
 * PURPOSE:  Small-boat autopilot with joystick manual control,
 *           cruise control, and auto-pilot heading hold using
 *           ICM-20948 9-axis DMP compass for heading.
 * HARDWARE: ESP32 + ICM-20948 (Pimoroni) + joystick + linear
 *           actuator + throttle DAC + LEDs
 * LIBRARY:  SparkFun ICM-20948  (Arduino Library Manager)
 *           — includes portable-C backbone & DMP firmware
 * ============================================================
 *
 * CHANGELOG v2.12:
 *  - BUG FIX: Manual tiller control broken in v2.11 —
 *    StopActuator() was called unconditionally in the
 *    pilot-OFF else block, killing the motor every loop
 *    iteration even when the joystick was deflected.
 *    Now only calls StopActuator() on the TRANSITION from
 *    auto→manual (when wasPilotActive is true), so manual
 *    joystick control works normally in manual mode.
 *
 * CHANGELOG v2.11:
 *  - BUG FIX: pilot.ino used JOYCRUISE_CENTER (1900) for the
 *    pilot joystick deadband instead of JOYPILOT_CENTER (1860).
 *    This offset the deadband by 40 counts, causing asymmetric
 *    manual override detection. Fixed all 3 occurrences.
 *  - BUG FIX: Missing StopActuator() when exiting auto mode —
 *    motor could run indefinitely after switching pilot OFF.
 *    Now calls StopActuator() before clearing motorMoving.
 *  - BUG FIX: lastPrintMs = millis() was commented out in
 *    cruise.ino, causing serial spam on every loop iteration.
 *    Added separate lastCruisePrintMs timer so cruise and
 *    pilot outputs don't starve each other.
 *  - CLEANUP: Removed stale commented-out PWM_A/PWM_B defs;
 *    fixed comments on active PWM pin definitions.
 *
 * CHANGELOG v2.10:
 *  - Added PI (Proportional + Integral) controller to auto-pilot
 *    mode only. Manual mode and cruise control are UNCHANGED.
 *  - P-term: proportional to heading error (gain KP).
 *  - I-term: accumulates persistent error (gain KI) to eliminate
 *    steady-state offset from wind/current.
 *  - Anti-windup: integral clamped to ±INTEGRAL_LIMIT.
 *  - Integral resets on: entering auto mode, manual override,
 *    and exiting auto mode.
 *  - MIN_AUTO_SPEED ensures actuator actually moves when auto
 *    steering is active (below ~80 PWM the actuator stalls).
 *  - PULSE_GAIN multiplies pulse duration for stronger response.
 *  - Serial debug shows P/I components for on-water tuning.
 *
 * CHANGELOG v2.0 (new — integrated from v1.12 compass + autopilot):
 *  - Replaced LSM303 compass with ICM-20948 DMP 9-axis fusion.
 *    The DMP provides stable, tilt-compensated heading with
 *    auto-calibration and EEPROM bias persistence.
 *  - Removed all LSM303 / Pololu library dependencies.
 *  - Fixed heading wrap-around bug: shortest-path error calc
 *    now works correctly across the 0°/360° boundary.
 *  - Non-blocking motor control: replaced delay()-based motor
 *    pulses with timestamp-based timing. Loop never blocks.
 *  - Pilot auto-mode blocked during compass auto-calibration.
 *  - Heading cached globally: getHeading() called once per
 *    loop cycle at 25 Hz, PilotControl() reads the cache.
 *  - Serial commands merged: compass cmds (D/B/S/R) alongside
 *    autopilot status output.
 *  - Tab-file architecture:
 *      sensor_init.ino  — ICM-20948 + DMP initialization
 *      heading.ino      — DMP quaternion→heading, FIFO drain
 *      eeprom_bias.ino  — EEPROM bias save/restore/clear
 *      autocal.ino      — auto-calibration mode handler
 *      cruise.ino       — cruise control with DAC throttle
 *      pilot.ino        — auto-pilot heading hold + manual steer
 *      serial_cmds.ino  — serial command handler + diagnostics
 *
 * CHANGELOG v2.01:
 *  - Added throttle (DAC output) to pilot serial status line.
 *  - Fixed serial output freeze when autopilot ON (missing newline).
 *  - Added steering error + status label ([STEERING]/[HOLDING]) when auto ON.
 *
 * ORIGINAL AUTOPILOT CREDITS:
 *  V0.1 TE Feb 2026 — Basic code with Pilot/Cruise joystick inputs
 *  V0.2 TBE Feb 2026 — Two functions, joystick + IMU compass input
 *  V2   Apr 2026     — Improved hysteresis auto-steering
 *
 * SERIAL COMMANDS
 * ───────────────
 *   D   — Toggle DMP debug output (shows raw quaternion)
 *   B   — Show current DMP compass biases
 *   S   — Save current DMP compass biases to EEPROM
 *   R   — Reset (clear) EEPROM biases — forces fresh calibration
 * ============================================================
 */

// ═════════════════════════════════════════════════════════════
//  INCLUDES
// ═════════════════════════════════════════════════════════════

#include <math.h>
#include <Wire.h>
#include <EEPROM.h>

// ─── DMP ENABLE ─────────────────────────────────────────────
// IMPORTANT: You MUST uncomment #define ICM_20948_USE_DMP in
// the library's src/util/ICM_20948_C.h for DMP support.
// A #define here in the sketch alone does NOT work — the library's
// C files are compiled separately and never see it.
#define ICM_20948_USE_DMP   // kept for .ino compilation (enums, etc.)

#include "ICM_20948.h"

// ═════════════════════════════════════════════════════════════
//  CONFIGURATION — COMPASS
// ═════════════════════════════════════════════════════════════

#define DECLINATION_DEG  14.9f  // Magnetic declination of Possession Point

// ICM-20948 I2C address select:
//   Pimoroni breakout: AD0 pin tied to GND → address 0x68 → AD0_VAL = 0
//   SparkFun breakout: AD0 pin pulled high → address 0x69 → AD0_VAL = 1
#define AD0_VAL          0      // Pimoroni board (0x68). Change to 1 for SparkFun.

#define SAMPLE_HZ        25     // Compass loop update rate (Hz)

// ─────────────── AUTO CAL MODE ──────────────────────────────
#define AUTO_CAL_SEC     300    // 5 minutes auto-cal window after boot
#define CAL_MIN_SEC      45     // Minimum seconds before EEPROM save allowed

// ─────────────── EEPROM BIAS STORAGE ─────────────────────────
// Match the official Example11 pattern: save ALL 9 biases
// (gyro + accel + compass) with a checksum, not just compass.

struct BiasStore {
  uint32_t  magic;
  int32_t   biasGyroX;
  int32_t   biasGyroY;
  int32_t   biasGyroZ;
  int32_t   biasAccelX;
  int32_t   biasAccelY;
  int32_t   biasAccelZ;
  int32_t   biasCPassX;
  int32_t   biasCPassY;
  int32_t   biasCPassZ;
  int32_t   sum;         // Checksum: sum of magic + all 9 bias values
};

#define BIAS_MAGIC       0x42        // Match Sparkfun Example11 header value
#define BIAS_EEPROM_SIZE 128

// ─────────────── HEADING RESULT STRUCT ───────────────────────

struct HeadingResult {
  float    heading;          // 0 – 360 degrees (true, declination applied)
  bool     valid;            // false if sensor had no new data this cycle
  uint16_t magAccuracy;      // DMP compass accuracy (lower 2 bits: 0–3)
  uint16_t magAccuracyRaw;   // Raw FIFO value for debugging
};

// ═════════════════════════════════════════════════════════════
//  CONFIGURATION — AUTOPILOT
// ═════════════════════════════════════════════════════════════

// Pin Definitions
const int PILOT_JOY_PIN  = 32;   // Joystick pot Y-axis (pilot/rudder)
const int CRUISE_JOY_PIN = 34;   // Joystick pot X-axis (cruise/throttle)
const int PILOT_SW_PIN   = 33;   // Pilot toggle switch (SPST, LOW=ON)
const int CRUISE_SW_PIN  = 4;    // Cruise momentary switch
const int DACPIN         = 25;   // DAC output for throttle (simulated hall)
const int PWM_A          = 26;   // Motor driver PWM A (right/starboard)
const int PWM_B          = 27;   // Motor driver PWM B (left/port)
const int PilotLEDpin    = 18;   // Pilot active LED
const int CruiseLEDpin   = 19;   // Cruise active LED

// --- SAFETY LIMITS (Calibrate these using Serial Monitor) ---
const int LIMIT_LEFT     = 500;
const int LIMIT_RIGHT    = 3500;

// Joystick Center Calibration
const int JOYCRUISE_CENTER = 1860;  // Rest position — find via serial monitor
const int JOYPILOT_CENTER  = 1900;  // Rest position — find via serial monitor

const int NUM_SAMPLES     = 50;     // Analog samples with ~1ms dwell for accuracy

// Cruise Control Settings
const int CRUISE_DEADBAND   = 110;   // Joystick deadband for throttle
const int MINIMUM_THROTTLE  = 200;   // Minimum throttle to engage cruise (~10%)
const int DACLOW            = 41;    // DAC idle: ~0.8V post-Op-Amp. NOT zero because
                                          // the hall-effect throttle sensor needs a minimum
                                          // voltage to register idle/tickover. 0V would
                                          // look like a sensor fault to the outboard.
const int DACHIGH           = 215;   // DAC full throttle: ~4.2V post-Op-Amp

// Pilot (Autopilot) Settings
const int PILOT_DEADBAND    = 500;   // Joystick deadband for manual rudder
const int HEADING_HYSTERESIS = 3;     // Degrees — don't steer if error < this
const int MAX_MOTOR_SPEED   = 255;   // PWM max for actuator
const int MAX_ERROR_DEG     = 200;   // Cap error for motor speed calc
const int HEADING_TIMEOUT   = 5000;  // ms — if no heading update, stop motor

// ─────────────── PI CONTROLLER (auto mode only) ──────────────
// Proportional gain: motor response per degree of heading error.
// Higher = more aggressive steering. Start at 1.5 and tune.
const float KP              = 1.5f;
// Integral gain: slowly accumulates persistent error from wind/current.
// Too high causes oscillation. Start low (0.1–0.3) and increase slowly.
const float KI              = 0.2f;
// Anti-windup: maximum integral contribution in degrees-equivalent.
// Prevents integral from growing unbounded when error can't be corrected.
const float INTEGRAL_LIMIT  = 50.0f;
// Minimum PWM for auto-mode actuator movement. Below this the actuator
// stalls and doesn't move. Adjust for your actuator (typically 60–100).
const int   MIN_AUTO_SPEED  = 80;
// Pulse duration multiplier. Original was 1ms per degree of error;
// this was too short for most actuators. Increase for longer pulses.
const int   PULSE_GAIN      = 10;    // ms of motor runtime per unit of PI output

// ═════════════════════════════════════════════════════════════
//  GLOBALS — COMPASS
// ═════════════════════════════════════════════════════════════

ICM_20948_I2C myICM;

// Mode flags
static bool sensorOK        = false;  // True after successful sensor init
static bool dmpOK           = false;  // True after successful DMP init
static bool debugMode       = false;  // DMP debug output toggle

// Auto-cal state
static bool         autoCalActive   = true;   // True during auto-cal window
static bool         accuracy3Saved  = false;  // True once biases saved at accuracy=3
static unsigned long dmpStartMs     = 0;      // When DMP became active
static bool         eepromReady     = false;  // True after EEPROM.begin() succeeded
static unsigned long lastCompassMs  = 0;      // Last compass update timestamp

// Cached heading (updated by getHeading() once per loop)
static float  currentHeading    = 0.0f;  // 0–360°, updated at SAMPLE_HZ
static bool   headingValid      = false; // True when we have a fresh heading

// ═════════════════════════════════════════════════════════════
//  GLOBALS — AUTOPILOT
// ═════════════════════════════════════════════════════════════

// Cruise state
static bool cruiseActive      = false;
static int  cruiseLockedValue = 0;
static bool lastButtonState   = HIGH;

// Pilot state
static bool         pilotActive   = false;  // True when pilot switch is ON
static bool         warmup        = false;  // True after first manual-mode cycle
static float        targetHeading = 0.0f;   // Heading to maintain (auto mode)
static int          motorSpeed    = 200;    // Current actuator PWM speed (200 default from original)
static int          currentDacOutput = DACLOW; // Current DAC throttle output (set by cruise.ino)
static bool         motorMoving   = false;  // True while actuator is active
static unsigned long motorStopMs  = 0;      // When to stop actuator (non-blocking)
static unsigned long lastHeadingMs = 0;     // When we last got a valid heading

// PI controller state (auto mode only — declared here, used in pilot.ino)
static float         integralSum     = 0.0f;  // Accumulated integral error
static unsigned long lastSteerTimeMs = 0;     // Timestamp of last steering calculation

// Debug timers (separate so cruise and pilot don't starve each other)
static unsigned long lastPrintMs = 0;       // Used by pilot.ino
static unsigned long lastCruisePrintMs = 0; // Used by cruise.ino

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════

void setup() {
  // ── Autopilot pin setup ───────────────────────────────────
  pinMode(PILOT_SW_PIN, INPUT_PULLUP);    // SPST toggle switch
  pinMode(CRUISE_SW_PIN, INPUT_PULLUP);   // Momentary switch
  pinMode(PWM_A, OUTPUT);                 // Motor driver PWM
  pinMode(PWM_B, OUTPUT);
  pinMode(PilotLEDpin, OUTPUT);           // LEDs
  pinMode(CruiseLEDpin, OUTPUT);
  analogReadResolution(12);               // ESP32 0–4095 range

  // ── Compass + DMP init ───────────────────────────────────
  initSensor();            // Serial, I2C, ICM-20948, DMP — in sensor_init.ino

  if (dmpOK) {
    bool biasesRestored = restoreBiasesFromEEPROM();   // in eeprom_bias.ino
    dmpStartMs = millis();
    if (biasesRestored) {
      // Biases restored — flush stale FIFO data then go to heading
      myICM.resetFIFO();
      autoCalActive  = false;
      accuracy3Saved = true;
      Serial.println(F("Calibrated — heading mode active."));
      Serial.println(F("'D'=DMP debug, 'B'=biases, 'S'=save, 'R'=reset biases."));
    } else {
      announceAutoCal();   // in autocal.ino
    }
  }

  lastCompassMs  = millis();
  lastHeadingMs  = millis();
}

// ═════════════════════════════════════════════════════════════
//  LOOP — non-blocking, time-sliced
// ═════════════════════════════════════════════════════════════

void loop() {
  handleSerialCommands();        // in serial_cmds.ino

  // ── Compass update at SAMPLE_HZ ──────────────────────────
  unsigned long now = millis();
  if (now - lastCompassMs >= (1000UL / SAMPLE_HZ)) {
    lastCompassMs = now;

    HeadingResult hr = getHeading();   // in heading.ino

    if (hr.valid) {
      currentHeading = hr.heading;
      headingValid   = true;
      lastHeadingMs  = now;
      markDataReceived();              // in serial_cmds.ino

      if (autoCalActive) {
        handleAutoCal(hr);             // in autocal.ino
      }
    } else {
      handleNoDataDiagnostic();        // in serial_cmds.ino
    }
  }

  // ── Motor safety: stop if no heading for too long ─────────
  if (motorMoving && (millis() - lastHeadingMs > HEADING_TIMEOUT)) {
    StopActuator();
    motorMoving = false;
  }

  // ── Non-blocking motor timer ──────────────────────────────
  if (motorMoving && millis() >= motorStopMs) {
    StopActuator();
    motorMoving = false;
  }

  // ── Cruise control (runs every loop — analog reads throttle it) ──
  CruiseControl();               // in cruise.ino

  // ── Pilot / autopilot (runs every loop) ───────────────────
  PilotControl();                // in pilot.ino
}
