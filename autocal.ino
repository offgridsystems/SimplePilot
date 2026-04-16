// ═════════════════════════════════════════════════════════════
//  AUTOCAL — auto-calibration mode handler
//  (included as tab in simplepilot_v2.12)
// ═════════════════════════════════════════════════════════════

void announceAutoCal() {
  Serial.printf("AUTO-CAL for %d seconds — do figure-8s until accuracy=3/3.\n", AUTO_CAL_SEC);
  Serial.printf("  (Min %ds + accuracy 3/3 to save biases)\n", CAL_MIN_SEC);
  Serial.println("'D'=DMP debug, 'B'=biases, 'S'=save, 'R'=reset biases.");
}

void handleAutoCal(HeadingResult &hr) {
  // ── Fresh calibration mode (no saved biases) ───────────────
  // Calculate time elapsed since DMP start
  unsigned long elapsed = (millis() - dmpStartMs) / 1000UL;
  unsigned long remaining = (elapsed < AUTO_CAL_SEC) ? (AUTO_CAL_SEC - elapsed) : 0;

  // Show accuracy (masked/raw) + countdown
  Serial.printf("Acc:%d(%d)/3", hr.magAccuracy, hr.magAccuracyRaw);

  // Check if accuracy reached 3/3 — save and exit auto-cal
  // Also exit if biases already saved (e.g. user pressed 'S')
  if (accuracy3Saved) {
    autoCalActive = false;
    Serial.println(" — Auto-cal complete (biases saved), switching to heading mode.");
  } else if (hr.magAccuracy == 3 && elapsed >= CAL_MIN_SEC) {
    // Both conditions met: accuracy=3/3 AND minimum time elapsed
    Serial.println("  → CALIBRATED!");
    saveBiasesToEEPROM();
    autoCalActive = false;
    Serial.println("Auto-cal complete — switching to heading mode.");
  } else {
    // Show what's preventing save
    if (hr.magAccuracy < 3) {
      Serial.printf(" (do figure-8s, %lus left)\n", remaining);
    } else {
      // Accuracy is 3 but time not yet met
      Serial.printf(" (acc=3! wait %lus more, %lus left)\n",
                    CAL_MIN_SEC - elapsed, remaining);
    }

    // Timeout: exit auto-cal without saving (accuracy never reached 3)
    if (elapsed >= AUTO_CAL_SEC) {
      autoCalActive = false;
      Serial.println("Auto-cal timeout — switching to heading mode.");
      if (!accuracy3Saved) {
        Serial.println("  (Biases NOT saved — accuracy never reached 3/3)");
        Serial.println("  (Use 'S' to force-save current biases if desired.)");
      }
    }
  }
}
