// ═════════════════════════════════════════════════════════════
//  PILOT — auto-pilot heading hold + manual rudder override
//  (included as tab in simplepilot_v2.12)
//
//  Manual mode logic preserved exactly from original SimplePilot
//  code — do not modify the manual override section.
//  Only the auto-pilot section has been updated for:
//    - ICM-20948 DMP heading source (was LSM303)
//    - Shortest-path heading error (was wrap-around broken)
//    - Non-blocking motor pulses (was delay()-based)
//    - PI controller for more aggressive + steady-state correction
//
//  v2.12 FIX: StopActuator() only called on the TRANSITION from
//    auto→manual (when wasPilotActive was true), not every loop
//    iteration. v2.11 called it unconditionally, which killed
//    manual tiller control every loop when pilot was OFF.
//  v2.11 FIX: All 3 joystick deadband checks now use
//    JOYPILOT_CENTER (was incorrectly JOYCRUISE_CENTER).
// ═════════════════════════════════════════════════════════════

// Track whether pilot was active last cycle (for heading lock on entry)
static bool wasPilotActive = false;

// ─── Shortest-path heading error (handles 0°/360° wrap-around) ──
// Returns error in range (-180, +180].
// Positive = target is to starboard (right).
// Negative = target is to port (left).

float headingError(float target, float current) {
  float err = target - current;
  if (err >  180.0f) err -= 360.0f;
  if (err < -180.0f) err += 360.0f;
  return err;
}

// ─── Pilot control ────────────────────────────────────────────

void PilotControl() {
   // Read Inputs from pot on Joystick and switch
   int rawJoyPilot = get_averaged_reading(PILOT_JOY_PIN, NUM_SAMPLES);
   pilotActive = false;
   if (warmup) {    // check switch after warmup and IMU has had a chance to settle
     pilotActive = (digitalRead(PILOT_SW_PIN) == LOW);   // Low = ON
   }

   // Block auto-pilot during compass calibration (heading unreliable)
   if (autoCalActive) {
     pilotActive = false;
   }

   // Apply Safety Constraint to Target
   int JoyPilotPos = constrain(rawJoyPilot, LIMIT_LEFT, LIMIT_RIGHT);

   // Get heading from DMP compass (cached at 25Hz in main loop)
   // (Original code had: compass.read(); Heading = compass.heading(); here)
   if (!warmup && headingValid) {
     targetHeading = currentHeading;   // at power up set both heading and setpoint to same
   }

   // Detect manual joystick override (joystick outside deadband)
   // v2.11 FIX: Use JOYPILOT_CENTER, not JOYCRUISE_CENTER
   bool joyOverride = (JoyPilotPos < (JOYPILOT_CENTER - PILOT_DEADBAND)) ||
                      (JoyPilotPos > (JOYPILOT_CENTER + PILOT_DEADBAND));

   // ── MANUAL OVERRIDE (always active — preserved from original) ──
   // Joystick deflection moves rudder and updates target heading.
   // Direction mapping preserved from original:
   //   low pot value → moveRight()
   //   high pot value → moveLeft()
   // v2.11 FIX: Use JOYPILOT_CENTER, not JOYCRUISE_CENTER
   if (JoyPilotPos < (JOYPILOT_CENTER - PILOT_DEADBAND)) {    // check pot value and move actuator based on pot value
     motorSpeed = 255;
     moveRight();
     if (headingValid) targetHeading = currentHeading;
   }
   else if (JoyPilotPos > (JOYPILOT_CENTER + PILOT_DEADBAND)) {
     motorSpeed = 255;
     moveLeft();
     if (headingValid) targetHeading = currentHeading;
   }
   else {
     // Safety: Stop if no stick input AND no auto-pilot pulse running
     // (Without the motorMoving guard, StopActuator() kills the auto-pilot's
     // timed pulse on the very next loop iteration, making auto mode impossible)
     if (!motorMoving) StopActuator();
   }

   delay(1);

   // ── AUTO-PILOT MODE — PI controller (auto mode only) ────────
   if (pilotActive && headingValid) {
     // On first entry to auto mode, lock current heading as target
     // and reset PI controller state
     if (!wasPilotActive) {
       targetHeading    = currentHeading;
       integralSum      = 0.0f;
       lastSteerTimeMs  = millis();
     }
     wasPilotActive = true;

     // Reset integral on manual joystick override — user is steering,
     // so stale integral from the previous heading is invalid
     if (joyOverride) {
       integralSum     = 0.0f;
       lastSteerTimeMs = millis();
     }

     // Only compute steering if joystick centered and no motor pulse running
     if (!joyOverride && !motorMoving) {

       float err = headingError(targetHeading, currentHeading);
       int absErr = (int)fabsf(err);

       // Cap error at MAX_ERROR_DEG (safety — prevents huge PI output)
       if (absErr > MAX_ERROR_DEG) {
         absErr = MAX_ERROR_DEG;
         err = (err > 0) ? (float)MAX_ERROR_DEG : -(float)MAX_ERROR_DEG;
       }

       if (absErr > HEADING_HYSTERESIS) {
         // ── PI computation ──────────────────────────────────
         unsigned long now = millis();
         float dt = (now - lastSteerTimeMs) / 1000.0f;
         // Clamp dt to reasonable range (0.01s – 2.0s) to prevent
         // integral spike on first calculation or after long pause
         if (dt < 0.01f) dt = 0.01f;
         if (dt > 2.0f)  dt = 2.0f;
         lastSteerTimeMs = now;

         // Accumulate integral (only when outside hysteresis)
         integralSum += (err * KI * dt);
         // Anti-windup: clamp integral to ±INTEGRAL_LIMIT
         integralSum = constrain(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

         // PI output: P-term + I-term
         float piOutput = (KP * err) + integralSum;
         int absOutput = (int)fabsf(piOutput);

         // ── Map PI output to motor speed and pulse duration ──
         // Motor speed: MIN_AUTO_SPEED + output, capped at MAX_MOTOR_SPEED
         motorSpeed = constrain(MIN_AUTO_SPEED + absOutput, MIN_AUTO_SPEED, MAX_MOTOR_SPEED);

         // Pulse duration: output × PULSE_GAIN, minimum 20ms so actuator actually moves
         unsigned long pulseDuration = (unsigned long)(absOutput * PULSE_GAIN);
         if (pulseDuration < 20) pulseDuration = 20;

         // Steer in direction of error
         if (piOutput > 0) {
           moveRight();
         } else {
           moveLeft();
         }

         // Non-blocking: start timed motor pulse
         motorStopMs = millis() + pulseDuration;
         motorMoving = true;
       } else {
         // Within hysteresis — no steering pulse, but keep timer current
         // so next pulse's dt calculation isn't inflated
         lastSteerTimeMs = millis();
       }
     }

     digitalWrite(PilotLEDpin, HIGH); // Turn ON LED

   } else {
     // Manual mode — pilot switch OFF or heading not valid
     // v2.12 FIX: Only clean up on the TRANSITION from auto→manual.
     // If we always call StopActuator(), manual joystick control is
     // killed every loop (motor starts from moveRight/moveLeft above,
     // then immediately stopped here). Only stop on transition.
     if (wasPilotActive) {
       StopActuator();        // Kill any auto-pilot pulse in progress
       motorMoving  = false;  // Cancel stale auto-pilot pulse timer
       integralSum  = 0.0f;   // Reset integral when leaving auto mode
     }
     wasPilotActive = false;
     warmup = true;           // after first manual cycle, allow autopilot
     digitalWrite(PilotLEDpin, LOW); // Turn off LED
   }

   // Debug Output (Every 1sec)
   if (millis() - lastPrintMs > 1000) {
     Serial.print("PilotActive:"); Serial.print(pilotActive ? " ON " : " OFF");
     Serial.print(" | JOY-PILOT:"); Serial.print(rawJoyPilot);
     Serial.print(" | TGT:"); Serial.print(JoyPilotPos);
     Serial.print(" | CUR HDNG:"); Serial.print((int)currentHeading);
     Serial.print(" | TARGET HDNG:"); Serial.print((int)targetHeading);
     Serial.print(" | DAC-OUT:"); Serial.print(currentDacOutput);   // sig from throttle pot
     if (pilotActive) {
       float err = headingError(targetHeading, currentHeading);
       float pTerm = KP * err;
       Serial.printf(" | ERR:%+0.0f P:%+0.1f I:%+0.1f", err, pTerm, integralSum);
       Serial.println(motorMoving ? " [STEERING]" : " [HOLDING]");
     } else {
       Serial.println(" | [STANDBY]");
     }

     lastPrintMs = millis();
   }

   delay(1);
}
