// ═════════════════════════════════════════════════════════════
//  HEADING — DMP quaternion→heading, FIFO drain
//  (included as tab in simplepilot_v2.12)
// ═════════════════════════════════════════════════════════════

// ─── Quaternion → heading (0-360°, declination applied) ──────

float dmpQuat9ToHeading(icm_20948_DMP_data_t &dmpData) {
  double q1 = (double)dmpData.Quat9.Data.Q1 / 1073741824.0;
  double q2 = (double)dmpData.Quat9.Data.Q2 / 1073741824.0;
  double q3 = (double)dmpData.Quat9.Data.Q3 / 1073741824.0;

  double q0sq = 1.0 - q1*q1 - q2*q2 - q3*q3;
  double q0;
  if (q0sq > 0.0) q0 = sqrt(q0sq);
  else            q0 = 0.0;

  double heading = atan2(2.0 * (q0 * q3 + q1 * q2),
                         1.0 - 2.0 * (q2 * q2 + q3 * q3));
  heading *= 180.0 / PI;
  heading += DECLINATION_DEG;

  // fmod() instead of while-loops — guarantees exit even on NaN/Infinity
  heading = fmod(heading, 360.0);
  if (heading < 0.0) heading += 360.0;

  // Guard against NaN (e.g. from bad quaternion data)
  if (isnan(heading)) heading = 0.0;

  return (float)heading;
}

// ─── Main heading routine — drain FIFO, extract heading ──────

HeadingResult getHeading() {
  HeadingResult hr;
  hr.heading        = 0.0f;
  hr.valid          = false;
  hr.magAccuracy    = 0;
  hr.magAccuracyRaw = 0;

  if (!sensorOK || !dmpOK) return hr;

  // static: ~200 bytes each, avoids stack overflow on ESP32's 8KB task stack
  static icm_20948_DMP_data_t dmpData;
  static icm_20948_DMP_data_t latestQuat9;
  bool gotQuat9 = false;
  static uint16_t lastMagAccuracy = 0;
  static uint16_t lastMagAccuracyRaw = 0;

  int drainCount = 0;
  ICM_20948_Status_e fifoStat;
  while (((fifoStat = myICM.readDMPdataFromFIFO(&dmpData)) == ICM_20948_Stat_Ok
          || fifoStat == ICM_20948_Stat_FIFOMoreDataAvail)
         && drainCount < 50) {                 // bounded: max 50 packets per cycle
    drainCount++;
    static bool showedFirstPacket = false;
    if (!showedFirstPacket) {
      Serial.printf("DMP first FIFO packet: header=0x%08X drain=%d\n",
                    dmpData.header, drainCount);
      showedFirstPacket = true;
    }
    if ((dmpData.header & DMP_header_bitmap_Quat9) > 0) {
      latestQuat9 = dmpData;
      gotQuat9   = true;
    }
    // Read compass accuracy from header2 (can arrive on any packet)
    // IMPORTANT: The raw uint16_t has undocumented flags in upper bits.
    // The actual 0-3 accuracy is in the lower 2 bits.
    if ((dmpData.header2 & DMP_header2_bitmap_Compass_Accuracy) > 0) {
      lastMagAccuracyRaw = dmpData.Compass_Accuracy;
      lastMagAccuracy = dmpData.Compass_Accuracy & 0x03;  // Mask to lower 2 bits
    }
  }

  // Always use the latest accuracy (updates independently of Compass_Calibr)
  hr.magAccuracy    = lastMagAccuracy;
  hr.magAccuracyRaw = lastMagAccuracyRaw;

  if (gotQuat9) {
    hr.heading = dmpQuat9ToHeading(latestQuat9);
    hr.valid   = true;

    if (debugMode) {
      double dq1 = (double)latestQuat9.Quat9.Data.Q1 / 1073741824.0;
      double dq2 = (double)latestQuat9.Quat9.Data.Q2 / 1073741824.0;
      double dq3 = (double)latestQuat9.Quat9.Data.Q3 / 1073741824.0;
      Serial.printf("DMP Q1:%8.5f Q2:%8.5f Q3:%8.5f Acc:%d\n",
                    dq1, dq2, dq3, latestQuat9.Quat9.Data.Accuracy);
    }
  }

  return hr;
}
