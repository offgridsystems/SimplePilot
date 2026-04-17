
  SIMPLEPILOT v2.12 — Integrated Autopilot + DMP Compass
  ESP32 + ICM-20948 (9-axis DMP) + Joystick + Linear Actuator

OVERVIEW
────────
SimplePilot v2.12 combines small-boat autopilot heading hold with
the ICM-20948's onboard DMP for stable 9-axis compass headings.

Previous versions used an LSM303 magnetometer which required manual
calibration and was not tilt-compensated. The ICM-20948 DMP provides:
  • 9-axis sensor fusion (gyro + accel + compass)
  • Automatic calibration with EEPROM persistence
  • Tilt-compensated heading — stable on a rocking boat

v2.12 bug fix:
  • Fixed manual tiller control broken in v2.11 — StopActuator()
    was called unconditionally in the pilot-OFF else block, killing
    the motor every loop even when the joystick was deflected.
    Now only calls StopActuator() on the auto→manual TRANSITION.

v2.11 bug fixes:
  • Fixed pilot joystick deadband using JOYCRUISE_CENTER instead
    of JOYPILOT_CENTER — deadband was offset by 40 counts
  • Fixed missing StopActuator() when exiting auto mode — motor
    could run indefinitely after switching pilot OFF mid-pulse
  • Fixed lastPrintMs = millis() commented out in cruise.ino —
    was causing serial spam every loop iteration

v2.10 features:
  • PI controller — P-term for immediate response, I-term for
    wind/current steady-state offset elimination
  • Anti-windup: integral clamped to ±INTEGRAL_LIMIT
  • Integral resets on: entering auto mode, manual joystick
    override, and exiting auto mode
  • MIN_AUTO_SPEED ensures actuator moves (stall prevention)
  • PULSE_GAIN multiplies pulse duration for stronger response
  • Serial debug shows P/I components for on-water tuning

v2.0 features:
  • ICM-20948 DMP 9-axis fusion (replaced LSM303)
  • Fixed heading wrap-around bug (shortest-path error)
  • Non-blocking motor control (replaced delay())

v2.01 fixes:
  • Serial output freeze when autopilot ON (missing newline)
  • Added throttle (DAC output) to pilot serial status line
  • Added steering error + status label when auto ON


HARDWARE
────────
  • ESP32 dev board
  • Pimoroni ICM-20948 Breakout on I2C (pins 21=SDA, 22=SCL)
  • 2-axis joystick (pilot + cruise)
  • Linear actuator with motor driver (PWM on pins 26, 27)
  • DAC output for throttle (pin 25) via Op-Amp to hall sensor
  • Pilot toggle switch (pin 33)
  • Cruise momentary switch (pin 4)
  • Pilot LED (pin 18), Cruise LED (pin 19)

LIBRARY REQUIREMENT
───────────────────
  • SparkFun ICM-20948 library (install via Arduino Library Manager)
  • IMPORTANT: You MUST uncomment this line in the library's
    src/util/ICM_20948_C.h before compiling:
        #define ICM_20948_USE_DMP
    Without it, the DMP code is not compiled and the sketch
    will fail with link errors.



  PI CONTROLLER — AUTO MODE ONLY


  The auto-pilot uses a PI controller instead of the old P-only
  (proportional) approach. This produces more aggressive steering
  and eliminates steady-state heading offset from wind/current.

  FORMULA:
    Output = (KP × heading_error) + integral_sum

  P-TERM (Proportional):
    Responds immediately to heading error. Higher KP = more
    aggressive steering. Default: 1.5 (was effectively 1.0 in
    v2.0/v2.01). Increase if the boat wanders too much before
    correcting; decrease if the rudder overshoots.

  I-TERM (Integral):
    Accumulates persistent error over time. If wind pushes the
    bow 5° off course constantly, the integral slowly increases
    the correction until the offset is eliminated. Default KI:
    0.2 (very gentle). Increase slowly — too much causes
    oscillation (rudder swinging back and forth).

  ANTI-WINDUP:
    The integral is clamped to ±INTEGRAL_LIMIT (default: 50).
    This prevents the integral from growing without bound when
    the error can't be corrected (e.g. motor stalled, rudder
    at travel limit). Without anti-windup, the integral would
    build up a huge value and cause a massive overshoot when
    conditions change.

  INTEGRAL RESET:
    The integral is reset to zero when:
    1. Entering auto mode (switch turned ON)
    2. Manual joystick override (user steers manually)
    3. Exiting auto mode (switch turned OFF)
    This prevents stale integral from the previous heading from
    causing a sudden rudder slam.

  MIN_AUTO_SPEED:
    Below ~80 PWM, most linear actuators stall and don't move.
    MIN_AUTO_SPEED (default: 80) ensures the actuator actually
    moves when the auto-pilot commands a correction. In v2.0/
    v2.01, a 5° error produced PWM 5 — the actuator did nothing.

  PULSE_GAIN:
    Multiplies the pulse duration. Old code: 1ms per degree of
    error (e.g. 5° → 5ms pulse, far too short). New default: 10
    (5° error with KP=1.5 → output ~7.5 → 75ms pulse). Increase
    for longer pulses; decrease for shorter, more frequent pulses.



  PI TUNING GUIDE


  The PI controller has 5 tunable constants. Start with defaults
  and adjust one at a time on the water.

  PARAMETER        DEFAULT  EFFECT OF INCREASING
  ──────────────  ────────  ────────────────────────
  KP               1.5      Stronger immediate response.
                              Too high → overshoot/oscillation.
  KI               0.2      Faster steady-state correction.
                              Too high → oscillation/hunting.
  INTEGRAL_LIMIT   50.0     Allows larger integral build-up.
                              Too high → overshoot after wind shift.
  MIN_AUTO_SPEED   80       Faster actuator movement.
                              Too high → jerky small corrections.
  PULSE_GAIN       10       Longer motor pulses per correction.
                              Too high → overshoot/oscillation.

  TUNING STEPS:
  1. Start with defaults. Turn auto-pilot ON in calm conditions.
  2. If the boat wanders too much before correcting → increase KP.
  3. If the boat holds heading but is consistently a few degrees
     off (especially in wind) → increase KI slightly (0.05 steps).
  4. If the rudder overshoots and swings back and forth → decrease
     KP or KI.
  5. If small corrections don't move the actuator → increase
     MIN_AUTO_SPEED.
  6. If corrections are too weak / too short → increase PULSE_GAIN.

  The serial output shows P and I components for tuning:
    PilotActive: ON  | ... | ERR:+5 P:+7.5 I:+1.2 [STEERING]
    PilotActive: ON  | ... | ERR:+0 P:+0.0 I:+0.0 [HOLDING]

  P = proportional term (KP × error)
  I = integral sum (accumulated correction)



  DAC THROTTLE CALIBRATION GUIDE


  The ESP32's DAC (GPIO 25) outputs a voltage that passes through
  an Op-Amp boost circuit to drive the outboard's hall-effect
  throttle sensor. The DAC values in the code map throttle
  position (0–4095 from joystick) to DAC output (0–255):

    dacOutput = map(finalThrottle, 0, 4095, DACLOW, DACHIGH)

  DACLOW = 41  → ~0.8V post-Op-Amp (idle / tickover)
  DACHIGH = 215 → ~4.2V post-Op-Amp (full throttle)

  WHY DACLOW IS 41 AND NOT 0:
    The hall-effect throttle sensor needs a minimum voltage
    (~0.8V) to register idle/tickover. Sending 0V (DAC=0)
    looks like a disconnected sensor to the outboard, which
    may trigger a fault condition. DACLOW=41 provides the
    "zero throttle but engine still running" baseline — the
    equivalent of the throttle handle at its rest position.

  WHEN TO RECALIBRATE:
    • After changing the Op-Amp circuit (different gain resistor)
    • After moving to a different outboard with a different
      hall-effect sensor voltage range
    • If the engine doesn't idle smoothly (DACLOW too low)
    • If you can't reach full throttle (DACHIGH too low)
    • If the engine races at idle (DACLOW too high)

  WHAT YOU NEED:
    • Multimeter to measure voltage at the Op-Amp output
      (the wire going to the outboard's hall-effect sensor)
    • Serial Monitor connected at 115200 baud

  STEP-BY-STEP RECALIBRATION:

  METHOD 1: Using a temporary calibration sketch

  1. Create a new Arduino sketch with this code:

       const int DACPIN = 25;
       void setup() {
         Serial.begin(115200);
         Serial.println("DAC Calibrator — enter value 0-255, or 'sweep'");
       }
       void loop() {
         if (Serial.available()) {
           String input = Serial.readStringUntil('\n');
           input.trim();
           if (input == "sweep") {
             for (int v = 0; v <= 255; v += 5) {
               dacWrite(DACPIN, v);
               delay(500);
               Serial.printf("DAC=%d  (measure voltage now)\n", v);
             }
           } else {
             int val = input.toInt();
             val = constrain(val, 0, 255);
             dacWrite(DACPIN, val);
             Serial.printf("DAC set to %d\n", val);
           }
         }
       }

  2. Upload the calibration sketch to the ESP32.

  3. FIND DACLOW:
     • Start with DAC=0, measure Op-Amp output voltage.
     • Type increasing values: 10, 20, 30, 35, 40, 45, 50.
     • The lowest DAC value where the outboard idles
       smoothly = your new DACLOW.
     • ⚠️ Don't go below the voltage the outboard expects
       at idle — it may see 0V as a sensor fault.

  4. FIND DACHIGH:
     • Keep increasing: 180, 200, 210, 220, 230, 240, 255.
     • The highest DAC value before the Op-Amp output clips
       or exceeds the outboard's WOT (wide-open throttle)
       voltage = your new DACHIGH.
     • ⚠️ Don't exceed the outboard's max throttle voltage —
       you could over-rev the engine.

  5. Or use the "sweep" shortcut:
     • Type sweep — it steps through 0–255 in increments of 5.
     • Measure voltage at each step with your multimeter.
     • Note the DAC values at idle and at WOT.

  METHOD 2: Using the serial output from SimplePilot itself

  1. Upload SimplePilot v2.12.
  2. Watch the cruise serial line (printed once per second):
       JoyCruise:0, Out:41, Cruise:0
  3. The "Out" value is the current DAC output.
     • Throttle centered → Out should equal DACLOW (~41)
     • Throttle full forward → Out should reach DACHIGH (~215)
  4. If the values don't match your outboard's needs, edit
     DACLOW and DACHIGH in simplepilot_v2.12.ino and re-upload.

  AFTER RECALIBRATION:
  1. Update the constants in simplepilot_v2.12.ino:
       const int DACLOW  = XX;   // Your measured idle DAC value
       const int DACHIGH = YY;   // Your measured WOT DAC value
  2. Re-upload SimplePilot.
  3. Verify with serial monitor: at idle, DAC-OUT should equal
     your new DACLOW; at full throttle, it should reach DACHIGH.
  4. Test on the water: idle should be smooth, WOT should be
     reachable without over-revving.



  WIRING DIAGRAM — Pimoroni ICM-20948 Breakout


  The Pimoroni ICM-20948 Breakout has 6 through-hole pins.
  Only 4 are needed for I2C operation (VCC, GND, SDA, SCL).
  AD0 and INT are optional.

  Pimoroni Breakout Pin    ESP32 GPIO    Notes
  ──────────────────────  ────────────  ───────────────
  VCC                     3V3           3.3V supply
  GND                     GND           Common ground
  SDA                     GPIO 21       I2C data
  SCL                     GPIO 22       I2C clock
  AD0                     GND           Tied to GND for
                                          address 0x68
  INT                     (unused)      Interrupt out,
                                          not needed for
                                          DMP FIFO mode

  The breakout has onboard pull-ups on SDA/SCL — no external
  pull-ups needed. AD0 tied to GND sets I2C address to 0x68
  (AD0_VAL=0 in code). For SparkFun breakout (AD0 pulled high,
  address 0x69): change AD0_VAL to 1.



  WIRING DIAGRAM — Full SimplePilot v2.12 System


  Component           ESP32 GPIO    Function
  ─────────────────  ────────────  ──────────────────────
  Joystick Y-axis    GPIO 32       Pilot/rudder input
  Joystick X-axis    GPIO 34       Cruise/throttle input
  Pilot switch       GPIO 33       Toggle (SPST, LOW=ON)
  Cruise switch      GPIO 4        Momentary button
  DAC output         GPIO 25       Throttle (0.8–4.2V via Op-Amp)
  Motor driver PWM_A GPIO 26       Actuator right/starboard
  Motor driver PWM_B GPIO 27       Actuator left/port
  Pilot LED          GPIO 18       Auto-pilot active
  Cruise LED         GPIO 19       Cruise control active
  ICM-20948 SDA      GPIO 21       I2C data
  ICM-20948 SCL      GPIO 22       I2C clock



  BOOT SEQUENCE


  When powered on, the ESP32 goes through this sequence:

  1.  Serial monitor starts at 115200 baud.
      "Press any key within 5s to pause, or auto-start..."
      → Send any character to pause; otherwise auto-starts.

  2.  ICM-20948 sensor initialised.
      "ICM-20948 sensor OK."

  3.  DMP firmware loaded and ORIENTATION sensor enabled.
      "DMP firmware loaded OK."
      "DMP: ORIENTATION enabled."

  4.  EEPROM check — one of two paths:

      ┌─────────────────────────────────────────────────┐
      │  PATH A: EEPROM biases found (normal boot)      │
      │                                                  │
      │  "EEPROM biases restored to DMP."               │
      │  "Calibrated — heading mode active."            │
      │                                                  │
      │  → Goes straight to heading mode.               │
      │  → Auto-pilot available immediately.            │
      └─────────────────────────────────────────────────┘

      ┌─────────────────────────────────────────────────┐
      │  PATH B: No saved biases (first boot or 'R')    │
      │                                                  │
      │  "No valid biases in EEPROM"                    │
      │  "AUTO-CAL for 300 seconds..."                  │
      │                                                  │
      │  → Do figure-8s with the sensor until           │
      │    accuracy reaches 3/3.                        │
      │  → Biases auto-save to EEPROM.                  │
      │  → Next boot uses PATH A.                       │
      └─────────────────────────────────────────────────┘


  AUTO-CALIBRATION PROCEDURE


  First boot (or after pressing 'R' and rebooting):

  1.  The DMP starts outputting headings immediately but
      accuracy is 0/3 — heading may be inaccurate.

  2.  Hold the ICM-20948 sensor and slowly rotate it in
      figure-8 patterns for 1–2 minutes. The DMP uses
      the motion to compute compass calibration biases.

  3.  Watch the serial output:
      "Acc:0(0)/3 (do figure-8s, 280s left)"
      "Acc:1(1)/3 (do figure-8s, 200s left)"
      "Acc:2(2)/3 (do figure-8s, 120s left)"
      "Acc:3(3)/3  → CALIBRATED!"

  4.  Once accuracy=3/3 AND 45 seconds have elapsed,
      biases are automatically saved to EEPROM:
      "BIAS SAVED to EEPROM (acc=3/3): Compass X:-40.2µT..."

  5.  From now on, every boot uses PATH A — instant heading.

  NOTE: Auto-pilot is BLOCKED during calibration (heading
  unreliable). Manual joystick steering still works.



  SERIAL OUTPUT FORMAT


  With autopilot OFF (manual mode):
    JoyCruise:0, Out:41, Cruise:0
    PilotActive: OFF | JOY-PILOT:1914 | TGT:1914 | CUR HDNG:0 | TARGET HDNG:0 | DAC-OUT:41 | [STANDBY]

  With autopilot ON:
    JoyCruise:0, Out:41, Cruise:0
    PilotActive: ON  | JOY-PILOT:1860 | TGT:1860 | CUR HDNG:126 | TARGET HDNG:126 | DAC-OUT:41 | ERR:+0 P:+0.0 I:+0.0 [HOLDING]
    PilotActive: ON  | JOY-PILOT:1860 | TGT:1860 | CUR HDNG:121 | TARGET HDNG:126 | DAC-OUT:41 | ERR:+5 P:+7.5 I:+0.4 [STEERING]

  Fields:
    DAC-OUT   — Current DAC throttle output (0.8V baseline = 41)
    ERR       — Heading error in degrees (+ = steer right, - = steer left)
    P         — Proportional term (KP × error)
    I         — Integral sum (accumulated correction for wind/current)
    [STANDBY] — Autopilot off, manual mode
    [HOLDING] — Autopilot on, heading within hysteresis
    [STEERING]— Autopilot on, actuator actively correcting



  SERIAL COMMANDS


  Key  Action
  ───  ─────────────────────────────────────────────────
  D    Toggle DMP debug output (raw quaternion data)
  B    Show current DMP biases (gyro, accel, compass)
  S    Force-save current biases to EEPROM
  R    Clear EEPROM biases — forces recalibration on reboot

  NOTE: 'S' only saves if accuracy=3 has been reached this
  session. Press 'S' to force-save even if already saved.



  AUTOPILOT OPERATION


  MANUAL MODE (default):
  • Joystick left/right steers the rudder directly
  • Joystick up/down controls throttle (cruise control)
  • Target heading tracks current heading

  AUTO-PILOT MODE (toggle switch ON):
  • Current heading is locked as target when switch turned ON
  • PI controller steers to maintain target heading
  • P-term: immediate proportional response to heading error
  • I-term: slow correction for persistent wind/current offset
  • Short non-blocking pulses — loop never stalls
  • Joystick override: moving joystick steers manually and
    resets integral + updates target heading when released

  CRUISE CONTROL:
  • Press momentary switch with throttle applied → locks speed
  • Press again to disengage
  • Pull joystick back to cancel
  • Joystick forward overrides cruise with higher speed

  WARMUP: Auto-pilot switch is ignored for the first loop
  cycle after boot. Turn pilot switch ON after boot to
  engage auto mode.



  RECALIBRATION FOR IMU MAG SENSOR


  Recalibrate when:
  • You install the sensor in a new location
  • Magnetic environment changes (new electronics nearby)
  • Heading seems consistently wrong
  • After moving the boat to a different region

  Steps:
  1. Press 'R' and return in serial monitor
  2. Reboot the ESP32
  3. Do figure-8s until accuracy=3/3
  4. Biases auto-save — done!



  TROUBLESHOOTING


  SYMPTOM                          LIKELY CAUSE / FIX
  ───────────────────────────────  ──────────────────────────
  "ICM-20948 not found"            Check wiring, AD0_VAL.
                                   Try I2C scanner sketch.
  "DMP init failed"                ICM_20948_USE_DMP not
                                   uncommented in ICM_20948_C.h
  Heading jumps wildly on reboot   Biases corrupted — press
                                   'R', reboot, recalibrate.
  "No heading for 30s"             FIFO not producing data.
                                   Check DMP firmware loaded.
  Heading drifts slowly            Compass near ferrous metal
                                   or current-carrying wire.
                                   Relocate sensor or recalibrate.
  Auto-pilot won't engage          Still in auto-cal mode.
                                   Wait for calibration, or
                                   auto_cal timed out.
  Actuator runs continuously       Motor pulse timing issue.
                                   Check heading timeout.
  Heading wrong at 0°/360°         (Fixed in v2.0 — shortest-
                                   path error calculation.)
  Serial freezes when auto ON      (Fixed in v2.01 — missing
                                   newline added.)
  Manual tiller doesn't work       (Fixed in v2.12 —
                                   StopActuator() was called
                                   every loop in pilot-OFF
                                   else block.)
  Rudder overshoots / oscillates   KP or KI too high.
                                   Reduce KP by 0.5 steps.
                                   Reduce KI by 0.05 steps.
  Boat holds heading but offset    KI too low — increase by
  a few degrees in wind            0.05. Or PULSE_GAIN too low.
  Small corrections don't move     MIN_AUTO_SPEED too low.
  the actuator                     Increase to 90 or 100.
  Corrections too gentle           Increase KP or PULSE_GAIN.
  Engine doesn't idle smoothly     DACLOW too low — see DAC
                                   Calibration Guide above.
  Can't reach full throttle        DACHIGH too low — see DAC
                                   Calibration Guide above.
  Engine races at idle             DACLOW too high — see DAC
                                   Calibration Guide above.



  FILE STRUCTURE


  simplepilot_v2.12/
  ├── simplepilot_v2.12.ino     Main: config, globals, setup, loop
  ├── sensor_init.ino           ICM-20948 + DMP initialization
  ├── heading.ino               DMP quaternion→heading, FIFO drain
  ├── eeprom_bias.ino           EEPROM bias save/restore/clear
  ├── autocal.ino               Auto-calibration mode handler
  ├── cruise.ino                Cruise control + DAC throttle
  ├── pilot.ino                 Auto-pilot PI heading hold + manual
  ├── serial_cmds.ino           Serial commands + diagnostics
  └── README_simplepilot_v2.12.txt  This file



  VERSION HISTORY


  v2.12 (2026) — Manual tiller fix
    • BUG FIX: Manual tiller control broken in v2.11 —
      StopActuator() was called unconditionally in the
      pilot-OFF else block, killing the motor every loop
      iteration even when the joystick was deflected.
      Now only calls StopActuator() on the TRANSITION from
      auto→manual (when wasPilotActive was true).

  v2.11 (2026) — Bug fixes + cleanup
    • BUG FIX: pilot joystick deadband used JOYCRUISE_CENTER (1900)
      instead of JOYPILOT_CENTER (1860). Fixed all 3 occurrences.
    • BUG FIX: Missing StopActuator() when exiting auto mode —
      motor could run indefinitely after switching pilot OFF.
    • BUG FIX: lastPrintMs = millis() commented out in cruise.ino
      — was causing serial spam every loop iteration.
    • Added separate lastCruisePrintMs timer for cruise output.
    • CLEANUP: Removed stale commented-out PWM_A/PWM_B definitions.
    • CLEANUP: Fixed PWM_A/PWM_B comments to match variable names.

  v2.10 (2026) — PI controller for auto mode
    • Added PI (Proportional + Integral) controller
    • P-term: proportional to heading error (KP = 1.5)
    • I-term: accumulates persistent error (KI = 0.2)
    • Anti-windup: integral clamped to ±INTEGRAL_LIMIT (50)
    • Integral resets on mode transitions and manual override
    • MIN_AUTO_SPEED = 80 (actuator stall prevention)
    • PULSE_GAIN = 10 (stronger pulse duration)
    • Serial debug shows P/I components for tuning
    • Manual mode and cruise control UNCHANGED

  v2.01 (2026) — Bug fixes + throttle display
    • Fixed serial output freeze when autopilot ON (missing newline)
    • Added throttle (DAC output) to pilot serial status line
    • Added steering error + status label ([STEERING]/[HOLDING])
    • Folder renamed to simplepilot_v2.01

  v2.0  (2026) — Integrated autopilot + ICM-20948 DMP compass
    • Replaced LSM303 with ICM-20948 DMP 9-axis fusion
    • Fixed heading wrap-around bug (shortest-path error)
    • Non-blocking motor control (replaced delay())
    • Auto-pilot blocked during compass calibration
    • Heading cached globally at 25 Hz
    • Tab-file architecture for maintainability

  v1.12 — Compass-only: all-9-bias EEPROM, FIFO flush, no
    verification mode (DMP accuracy doesn't reflect external
    biases)

  v1.11 — Compass-only: refactored into tabs, removed
    GEOMAGNETIC_FIELD enable (garbage data)

  v1.10 — Compass-only: accuracy masking fix (& 0x03),
    45s min wait, 'R' command

  v0.1–v2 (autopilot only) — Basic joystick + LSM303,
    cruise control, hysteresis auto-steering

