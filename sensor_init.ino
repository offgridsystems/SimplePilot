// ═════════════════════════════════════════════════════════════
//  SENSOR INIT — ICM-20948 + DMP initialization
//  (included as tab in simplepilot_v2.12)
// ═════════════════════════════════════════════════════════════

void initSensor() {
  delay(1000);                        // Let ESP32 I2C settle after power-on
  Serial.begin(115200);
  delay(2000);                        // Give serial monitor time to connect

  Serial.println(F("========================================"));
  Serial.println(F("SimplePilot v2.12 (ICM-20948 DMP)"));
  Serial.println(F("Press any key within 5s to pause, or auto-start..."));
  // Wait up to 5 seconds for a keypress (so you can read setup output).
  unsigned long waitStart = millis();
  while (!Serial.available() && (millis() - waitStart < 5000)) { /* wait with timeout */ }
  if (Serial.available()) {
    int flushCount = Serial.available();
    while (flushCount--) Serial.read();   // bounded flush — can't hang
    Serial.println(F("Key pressed — continuing."));
  } else {
    Serial.println(F("Auto-starting."));
  }

  // ── I2C on ESP32 default pins (400kHz for DMP) ────────────
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // ── Initialise ICM-20948 ──────────────────────────────────
  if (myICM.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok) {
    Serial.println("ERROR: ICM-20948 not found. Check wiring & AD0_VAL.");
    return;
  }
  sensorOK = true;
  Serial.println("ICM-20948 sensor OK.");

  // ── Try to initialise DMP ─────────────────────────────────
  bool dmpSuccess = true;

  // Step 1: Load DMP firmware
  ICM_20948_Status_e dmpStat = myICM.initializeDMP();
  if (dmpStat != ICM_20948_Stat_Ok) {
    Serial.printf("ERROR: DMP init failed: %d (%s)\n",
                  dmpStat, myICM.statusString(dmpStat));
    Serial.println("  Hint: Is ICM_20948_USE_DMP uncommented in ICM_20948_C.h?");
    dmpSuccess = false;
  } else {
    Serial.println("  DMP firmware loaded OK.");
  }

  if (dmpSuccess) {
    // Step 2: Enable ORIENTATION sensor (9-axis Quat9 for heading)
    ICM_20948_Status_e stat = myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);
    if (stat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP enableDMPSensor ORIENT failed: %d (%s)\n",
                    stat, myICM.statusString(stat));
      dmpSuccess = false;
    } else {
      Serial.println("  DMP: ORIENTATION enabled.");
    }
  }

  // NOTE: We do NOT enable INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD.
  // During calibration the DMP outputs garbage Compass_Calibr data.
  // The official SparkFun Example11 only enables ORIENTATION.
  // The library automatically provides Compass_Accuracy via header2
  // when Quat9 is enabled.

  if (dmpSuccess) {
    // Step 3: Set Quat9 ODR to maximum
    ICM_20948_Status_e odrStat = myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0);
    if (odrStat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP Quat9 ODR rate failed: %d (%s)\n",
                    odrStat, myICM.statusString(odrStat));
      dmpSuccess = false;
    } else {
      Serial.println("  DMP: Quat9 ODR set to max.");
    }
  }

  if (dmpSuccess) {
    // Step 4: Enable FIFO, enable DMP, reset both
    ICM_20948_Status_e fifoEnStat = myICM.enableFIFO();
    if (fifoEnStat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP enableFIFO failed: %d (%s)\n",
                    fifoEnStat, myICM.statusString(fifoEnStat));
      dmpSuccess = false;
    }
  }

  if (dmpSuccess) {
    ICM_20948_Status_e dmpEnStat = myICM.enableDMP();
    if (dmpEnStat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP enableDMP failed: %d (%s)\n",
                    dmpEnStat, myICM.statusString(dmpEnStat));
      dmpSuccess = false;
    }
  }

  if (dmpSuccess) {
    ICM_20948_Status_e rstDmpStat = myICM.resetDMP();
    if (rstDmpStat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP resetDMP failed: %d (%s)\n",
                    rstDmpStat, myICM.statusString(rstDmpStat));
      dmpSuccess = false;
    }
  }

  if (dmpSuccess) {
    ICM_20948_Status_e rstFifoStat = myICM.resetFIFO();
    if (rstFifoStat != ICM_20948_Stat_Ok) {
      Serial.printf("ERROR: DMP resetFIFO failed: %d (%s)\n",
                    rstFifoStat, myICM.statusString(rstFifoStat));
      dmpSuccess = false;
    }
  }

  if (dmpSuccess) {
    Serial.println("  Waiting 2s for DMP to start...");
    delay(2000);
  }

  // ── Report DMP status ─────────────────────────────────────
  if (dmpSuccess) {
    dmpOK = true;
    Serial.println("DMP 9-axis fusion ACTIVE.");
  } else {
    dmpOK = false;
    Serial.println("ERROR: DMP not available — no headings.");
  }
}
