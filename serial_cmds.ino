// ═════════════════════════════════════════════════════════════
//  SERIAL CMDS — serial command handler + diagnostics
//  (included as tab in simplepilot_v2.12)
// ═════════════════════════════════════════════════════════════

// ─── Serial command handler ──────────────────────────────────

void handleSerialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();

  // Compass commands
  if (c == 'D' || c == 'd') {
    debugMode = !debugMode;
    if (debugMode) Serial.println("DMP DEBUG ON");
    else           Serial.println("DMP DEBUG OFF");
  }
  if (c == 'B' || c == 'b') {
    showCurrentBiases();              // in eeprom_bias.ino
  }
  if (c == 'S' || c == 's') {
    saveBiasesToEEPROM();             // in eeprom_bias.ino
  }
  if (c == 'R' || c == 'r') {
    clearEEPROMBiases();              // in eeprom_bias.ino
  }
}

// ─── No-data diagnostic ──────────────────────────────────────

static bool everGotData = false;
static unsigned long firstDataWaitMs = 0;

void markDataReceived() {
  everGotData = true;
}

void handleNoDataDiagnostic() {
  // Only warn if we've NEVER had valid data after 30s
  if (!everGotData) {
    if (firstDataWaitMs == 0) firstDataWaitMs = millis();
    if (dmpOK && (millis() - firstDataWaitMs > 30000)) {
      Serial.println("WARN: DMP OK but no heading for 30s — FIFO diagnostic:");
      Serial.printf("  sensorOK=%d  dmpOK=%d\n", sensorOK, dmpOK);
      static icm_20948_DMP_data_t debugData;
      ICM_20948_Status_e dbgStat = myICM.readDMPdataFromFIFO(&debugData);
      Serial.printf("  FIFO read: stat=%d (%s) header=0x%08X\n",
                    dbgStat, myICM.statusString(dbgStat), debugData.header);
      firstDataWaitMs = millis();  // Reset so we don't spam
    }
  }
}
