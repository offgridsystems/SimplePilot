// ═════════════════════════════════════════════════════════════
//  EEPROM BIAS — save/restore DMP calibration biases
//  (included as tab in simplepilot_v2.12)
// ═════════════════════════════════════════════════════════════

bool restoreBiasesFromEEPROM() {
  if (!EEPROM.begin(BIAS_EEPROM_SIZE)) {
    Serial.println(F("  EEPROM init failed — biases not restored."));
    eepromReady = false;
    return false;
  }
  eepromReady = true;
  BiasStore store;
  EEPROM.get(0, store);
  // Validate: correct magic AND checksum matches
  int32_t sum = (int32_t)store.magic
    + store.biasGyroX + store.biasGyroY + store.biasGyroZ
    + store.biasAccelX + store.biasAccelY + store.biasAccelZ
    + store.biasCPassX + store.biasCPassY + store.biasCPassZ;
  if (store.magic == BIAS_MAGIC && store.sum == sum) {
    // Valid biases found — write ALL 9 to DMP (matching Example11)
    bool ok = true;
    ok &= (myICM.setBiasGyroX(store.biasGyroX)   == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasGyroY(store.biasGyroY)   == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasGyroZ(store.biasGyroZ)   == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasAccelX(store.biasAccelX)  == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasAccelY(store.biasAccelY)  == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasAccelZ(store.biasAccelZ)  == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    ok &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);
    if (!ok) {
      Serial.println(F("  EEPROM biases found but DMP write FAILED!"));
      return false;
    }
    Serial.println(F("  EEPROM biases restored to DMP."));
    // Show compass biases in µT for verification
    float bxuT = (float)store.biasCPassX / 65536.0;
    float byuT = (float)store.biasCPassY / 65536.0;
    float bzuT = (float)store.biasCPassZ / 65536.0;
    Serial.printf("  Compass bias X:%.1fµT Y:%.1fµT Z:%.1fµT\n", bxuT, byuT, bzuT);
    return true;
  } else {
    Serial.println(F("  No valid biases in EEPROM — DMP will auto-calibrate."));
    return false;
  }
}

void saveBiasesToEEPROM() {
  if (accuracy3Saved) return;  // Already saved this session
  if (!eepromReady) {
    Serial.println(F("ERROR: EEPROM not available — cannot save biases."));
    return;
  }

  // Read ALL 9 biases from DMP (matching Example11)
  BiasStore store;
  bool ok = true;
  ok &= (myICM.getBiasGyroX(&store.biasGyroX)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasGyroY(&store.biasGyroY)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasGyroZ(&store.biasGyroZ)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);
  if (!ok) {
    Serial.println(F("ERROR: Could not read all DMP biases."));
    return;
  }

  // Compute checksum (matching Example11 pattern)
  store.magic = BIAS_MAGIC;
  store.sum   = (int32_t)store.magic
    + store.biasGyroX + store.biasGyroY + store.biasGyroZ
    + store.biasAccelX + store.biasAccelY + store.biasAccelZ
    + store.biasCPassX + store.biasCPassY + store.biasCPassZ;

  EEPROM.put(0, store);
  EEPROM.commit();
  accuracy3Saved = true;

  float bxuT = (float)store.biasCPassX / 65536.0;
  float byuT = (float)store.biasCPassY / 65536.0;
  float bzuT = (float)store.biasCPassZ / 65536.0;
  Serial.printf("BIAS SAVED to EEPROM (acc=3/3): Compass X:%.1fµT Y:%.1fµT Z:%.1fµT\n",
                bxuT, byuT, bzuT);
}

void showCurrentBiases() {
  // Show all 9 DMP biases (matching Example11)
  int32_t gx, gy, gz, ax, ay, az, cx, cy, cz;
  bool ok = true;
  ok &= (myICM.getBiasGyroX(&gx)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasGyroY(&gy)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasGyroZ(&gz)   == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelX(&ax)  == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelY(&ay)  == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasAccelZ(&az)  == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassX(&cx)  == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassY(&cy)  == ICM_20948_Stat_Ok);
  ok &= (myICM.getBiasCPassZ(&cz)  == ICM_20948_Stat_Ok);
  if (!ok) {
    Serial.println(F("ERROR: Could not read DMP biases."));
    return;
  }
  Serial.printf("Gyro  X:%ld Y:%ld Z:%ld\n", (long)gx, (long)gy, (long)gz);
  Serial.printf("Accel X:%ld Y:%ld Z:%ld\n", (long)ax, (long)ay, (long)az);
  float cxuT = (float)cx / 65536.0;
  float cyuT = (float)cy / 65536.0;
  float czuT = (float)cz / 65536.0;
  Serial.printf("Compass X:%.1fµT Y:%.1fµT Z:%.1fµT (raw: %ld %ld %ld)\n",
                cxuT, cyuT, czuT, (long)cx, (long)cy, (long)cz);
}

void clearEEPROMBiases() {
  if (!eepromReady) {
    Serial.println(F("ERROR: EEPROM not available."));
    return;
  }
  // Zero out the entire store — invalid magic + wrong checksum
  BiasStore store;
  memset(&store, 0, sizeof(store));
  EEPROM.put(0, store);
  EEPROM.commit();
  Serial.println(F("EEPROM biases CLEARED — reboot for fresh calibration."));
}
