// ═════════════════════════════════════════════════════════════
//  CRUISE — cruise control with DAC throttle output
//  (included as tab in simplepilot_v2.12)
//
//  Cruise control logic preserved exactly from original
//  SimplePilot autopilot code — do not modify.
//
//  v2.11 FIX: Uses separate lastCruisePrintMs timer instead of
//  shared lastPrintMs. The old code had lastPrintMs = millis()
//  commented out, causing serial spam. Uncommenting it would
//  have starved pilot's output (cruise runs first in the loop).
//  Separate timer fixes both problems.
// ═════════════════════════════════════════════════════════════

// ─── Analog sample averaging ─────────────────────────────────

float get_averaged_reading(int pin, int samples) {
  uint32_t sum = 0;                // 32-bit to prevent overflow
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(450);        // ~1ms dwell for noise filtering
  }
  return (float)sum / samples;
}

// ─── Motor primitives ────────────────────────────────────────

void moveRight() { analogWrite(PWM_A, motorSpeed); analogWrite(PWM_B, 0); }
void moveLeft()  { analogWrite(PWM_A, 0); analogWrite(PWM_B, motorSpeed); }
void StopActuator() { analogWrite(PWM_A, 0); analogWrite(PWM_B, 0); }

// ─── Cruise control (preserved from original) ─────────────────

void CruiseControl() {
  // Read Inputs from pot on Joystick
  int rawJoyCruise = get_averaged_reading(CRUISE_JOY_PIN, NUM_SAMPLES);

  // Test for manual mode for cruise. If so run motor speed from joystick control only
  int manualThrottle = 0;     // Handle Forward Throttle Range (Center to Max)
  if (rawJoyCruise > (JOYCRUISE_CENTER + CRUISE_DEADBAND))
  {
    manualThrottle = map(rawJoyCruise, JOYCRUISE_CENTER + CRUISE_DEADBAND, 4095, 0, 4095);
  }
  manualThrottle = constrain(manualThrottle, 0, 4095);

  //  Cruise Button Logic (Toggle momentary switch)
  bool currentButtonState = digitalRead(CRUISE_SW_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
   if (!cruiseActive && manualThrottle > MINIMUM_THROTTLE)
    { // Require ~10% throttle to engage
      cruiseActive = true;
      cruiseLockedValue = manualThrottle;   // lock in current throttle
      Serial.println(">>>> CRUISE ON ");
      digitalWrite(CruiseLEDpin, HIGH); // Turn ON cruise LED
    } else
    {
      cruiseActive = false;
      Serial.println("<<<< CRUISE OFF (Button)");
    }
        delay(50); // Debounce
   }
    lastButtonState = currentButtonState;

    // Safety Cruise Cancel: Only if pulling BACKWARD (below center)
    // But allow the joystick to return to center without killing cruise.
    if (cruiseActive && rawJoyCruise < (JOYCRUISE_CENTER - CRUISE_DEADBAND)) {
    cruiseActive = false;
    Serial.println(">>> CRUISE CANCELLED (Joystick Pull-Back)");
    }

    // Output Selection
    // If manual throttle is higher than cruise, manual takes priority, so save new throttle value
    int finalThrottle = 0;
    if (cruiseActive && manualThrottle < cruiseLockedValue) {
      finalThrottle = cruiseLockedValue;
    } else {
    finalThrottle = manualThrottle;
    cruiseLockedValue = manualThrottle; // lock in higher pot value
  }


  // Map to DAC for Op-Amp
  int dacOutput = map(finalThrottle, 0, 4095, DACLOW, DACHIGH);
  dacOutput = constrain(dacOutput, DACLOW, DACHIGH);
  dacWrite(DACPIN, dacOutput);
  currentDacOutput = dacOutput;   // Store for pilot.ino status display

// Status Serial Monitor output
if (millis() - lastCruisePrintMs > 1000) {
  Serial.printf("JoyCruise:%d, Out:%d, Cruise:%d\n", rawJoyCruise, dacOutput, cruiseActive);
  lastCruisePrintMs = millis();  // Separate timer — doesn't starve pilot output
}

 delay(10);    // TIM might need more delay
 if (cruiseActive == false) digitalWrite(CruiseLEDpin, LOW); // Turn Off blue LED

  delay(1);
}
