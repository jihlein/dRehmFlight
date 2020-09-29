#define ACCELDIV 6
#define GYRODIV  4

#define CAL_STABLE_TIME 5
#define CAL_TIMEOUT     5

int16_t accADC_raw[NUMBEROFAXIS];
int16_t gyroADC_raw[NUMBEROFAXIS];

const int8_t ACC_RPY_Order[NUMBEROFORIENTS][NUMBEROFAXIS] = 
{
  {ROLL,  PITCH, YAW}, // Up/Back (Normal)
  {PITCH, ROLL,  YAW}, // Up/Left
  {ROLL,  PITCH, YAW}, // Up/Front (Aft)
  {PITCH, ROLL,  YAW}, // Up/Right (Sideways)
};
  
const int8_t Acc_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] =
{
  {-1,-1, 1}, // Up/Back (Normal)
  { 1,-1, 1}, // Up/Left
  { 1, 1, 1}, // Up/Front (Aft)
  {-1, 1, 1}, // Up/Right (Sideways)
};
  
const int8_t Gyro_RPY_Order[NUMBEROFORIENTS][NUMBEROFAXIS] = 
{
  {ROLL,  PITCH, YAW}, // Up/Back (Normal)
  {PITCH, ROLL,  YAW}, // Up/Left
  {ROLL,  PITCH, YAW}, // Up/Front (Aft)
  {PITCH, ROLL,  YAW}, // Up/Right (Sideways)
};

// These are the polarities to return them to the default
const int8_t Gyro_Pol[NUMBEROFORIENTS][NUMBEROFAXIS] = 
{
  //   Model referenced
  //   ROLL, PITCH, YAW
  { 1, 1, 1}, // Up/Back (Normal)
  {-1, 1, 1}, // Up/Left
  {-1,-1, 1}, // Up/Front (Aft)
  { 1,-1, 1}, // Up/Right (Sideways)
};
  
//////////////////////////////////////
//////////////////////////////////////

void initMPU() {
  int status = mpu9250.begin();    

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  mpu9250.setGyroRange(mpu9250.GYRO_RANGE_2000DPS);
  mpu9250.setAccelRange(mpu9250.ACCEL_RANGE_4G);

  mpu9250.setDlpfBandwidth(mpu9250.DLPF_BANDWIDTH_92HZ);
}

//////////////////////////////////////
//////////////////////////////////////

void readRawMPU() {
  mpu9250.getMotion6(&accADC_raw[0], &accADC_raw[1], &accADC_raw[2], &gyroADC_raw[0], &gyroADC_raw[1], &gyroADC_raw[2]);

  // Reorient the data as per the board orientation  
  for (uint8_t i = 0; i < NUMBEROFAXIS; i++)
  {
    // Rearrange the sensors for both orientations
    accADC_P1[i]  = accADC_raw[ACC_RPY_Order[Config.Orientation_P1][i]] >> ACCELDIV;
    accADC_P2[i]  = accADC_raw[ACC_RPY_Order[Config.Orientation_P2][i]] >> ACCELDIV;
    gyroADC_P1[i] = gyroADC_raw[Gyro_RPY_Order[Config.Orientation_P1][i]] >> GYRODIV;
    gyroADC_P2[i] = gyroADC_raw[Gyro_RPY_Order[Config.Orientation_P2][i]] >> GYRODIV;
  }
}

//////////////////////////////////////
//////////////////////////////////////

void readMPU() {
  int16_t temp1, temp2, temp3;
  
  readRawMPU();
  
  // Start "ReadAcc(void)"
  // P1
  // Use default Config.AccZero for Acc-Z if inverse calibration not done yet
  // Actual zero is held in Config.AccZeroNormZ waiting for inverse calibration
  if (!(eepromConfig.Main_flags & (1 << inv_cal_done_P1)))
  {
    eepromConfig.AccZero_P1[YAW] = 0;
  }
  // If inverted cal done, Config.AccZeroNormZ and Config.AccZeroDiff have valid values
  else
  {
    eepromConfig.AccZero_P1[YAW] = eepromConfig.AccZeroNormZ_P1 - eepromConfig.AccZeroDiff_P1;
  }

  // P2
  if (!(eepromConfig.Main_flags & (1 << inv_cal_done_P2)))
  {
    eepromConfig.AccZero_P2[YAW] = 0;
  }
  // If inverted cal done, Config.AccZeroNormZ and Config.AccZeroDiff have valid values
  else
  {
    eepromConfig.AccZero_P2[YAW] = eepromConfig.AccZeroNormZ_P2 - eepromConfig.AccZeroDiff_P2;
  }

  // Roll and Pitch are handled normally
  for (uint8_t i = 0; i < (NUMBEROFAXIS - 1); i++)
  {
    // Only need to do this if the orientations differ
    if (Config.P1_Reference != NO_ORIENT)
    {
      // Change polarity - use the zeros from the appropriate calibrate
      temp1 = ((accADC_P1[i] - eepromConfig.AccZero_P1[i]) * Acc_Pol[Config.Orientation_P1][i]);
      temp2 = ((accADC_P2[i] - eepromConfig.AccZero_P2[i]) * Acc_Pol[Config.Orientation_P2][i]);
      
      // Get P1 value
      temp1 = scale32(temp1, (100 - transition));

      // Get P2 value
      temp2 = scale32(temp2, transition);

      // Sum the two values
      accADC[i] = temp1 + temp2;
    }
    else
    {
      accADC[i] = ((accADC_P2[i] - eepromConfig.AccZero_P2[i]) * Acc_Pol[Config.Orientation_P2][i]);
    }
  }
  
  // Z -axis requires special handling as the zeros are already polarity corrected
  // Only need to do this if the orientations differ
  if (Config.P1_Reference != NO_ORIENT)
  {
    // Change polarity - use the zeros from the appropriate calibrate
    temp1 = ((accADC_P1[YAW] * Acc_Pol[Config.Orientation_P1][YAW]) - eepromConfig.AccZero_P1[YAW]);
    temp2 = ((accADC_P2[YAW] * Acc_Pol[Config.Orientation_P2][YAW]) - eepromConfig.AccZero_P2[YAW]);
      
    // Get P1 value
    temp1 = scale32(temp1, (100 - transition));

    // Get P2 value
    temp2 = scale32(temp2, transition);

    // Sum the two values
    accADC[YAW] = temp1 + temp2;
  }
  else
  {
    accADC[YAW] = ((accADC_P2[YAW] * Acc_Pol[Config.Orientation_P2][YAW]) - eepromConfig.AccZero_P2[YAW]);
  }
    
  // Recalculate current accVertf using filtered acc value
  // Note that AccSmooth[YAW] is already zeroed around 1G so we have to re-add 
  // the zero back here so that Config.AccZeroNormZ subtracts the correct amount
  // accVertf = accSmooth[YAW] + (Config.AccZeroNormZ - Config.AccZero[YAW]);
  
  // Note also that accSmooth[] has already got the correct acc orientations, 
  // so only needs the zeroing value merged from one to the other.

  // Only need to do this if the orientations differ
  if (Config.P1_Reference != NO_ORIENT)
  {
    // Calculate the correct Z-axis data based on the orientation
    temp1 = accSmooth[YAW] + (eepromConfig.AccZeroNormZ_P1 - eepromConfig.AccZero_P1[YAW]); 
    temp2 = accSmooth[YAW] + (eepromConfig.AccZeroNormZ_P2 - eepromConfig.AccZero_P2[YAW]); 
  
    // Merge with transition
    temp1 = scale32(temp1, (100 - transition));
    temp2 = scale32(temp2, transition);
   
    accVertf = (float)(temp1 + temp2);
  }
  // Just use the P2 value
  else
  {
    // Calculate the correct Z-axis data based on the orientation
    accVertf = accSmooth[YAW] + (float)(eepromConfig.AccZeroNormZ_P2 - eepromConfig.AccZero_P2[YAW]);   
  }
  // End "ReadAcc(void)"
  
  // Start "ReadGyros(void)"
  for (uint8_t i = 0; i < NUMBEROFAXIS; i++) {
    if (Config.P1_Reference != NO_ORIENT) {
      // P1 alternate (original) orientation. Swap zeros so that they match.
      temp1 = (gyroADC_P1[i] - eepromConfig.gyroZero_P1[i]) * Gyro_Pol[Config.Orientation_P1][i];
      
      // P2 orientation
      temp2 = (gyroADC_P2[i] - eepromConfig.gyroZero_P2[i]) * Gyro_Pol[Config.Orientation_P2][i];

      // Merge the two gyros per transition percentage
      temp3 = scale32(temp1, (100 - transition)) + scale32(temp2, transition);

      // Gyro alt is always per orientation
      gyroADCalt[i] = temp3;

      // If the P1 reference is MODEL, always use the same gyros as P2
      if (Config.P1_Reference == MODEL)
      {
        // Use P2 orientation
        gyroADC[i] = temp2; 
      }
      // Otherwise use the merged orientation (EARTH reference).
      else {
        // Use merged orientation
        gyroADC[i] = temp3; 
      }
    }
    // Single-orientation models
    else {
      // Change polarity using P2 orientation by default
      gyroADC[i] = (gyroADC_P2[i] - eepromConfig.gyroZero_P2[i]) * Gyro_Pol[Config.Orientation_P2][i];  
        
      // Copy to alternate set of gyro values
      gyroADCalt[i] = gyroADC[i];
    }
  }
  // End "ReadGyros(void)"
}

////////////////////////////////////////
////////////////////////////////////////

void CalibrateAcc(int8_t type)
{
  uint8_t i;
  int16_t accZero[NUMBEROFAXIS] = {0,0,0};  // Used for calibrating Accs on ground

  // Calibrate acc
  // P2
  if (type == NORMAL)
  {
    Serial.println("Normal Accel P2 Calibration Start....");
    
    // Work out which orientation we are calibrating.
    // Only need to do this if the orientations differ.
    // Just do P2 if orientations the same.
    // Will not save new calibration when different and not firmly in P1 or p2.
    if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
    {
      // Get average zero value (over 32 readings)
      for (i = 0; i < 32; i++)
      {
        readRawMPU(); 
        accZero[ROLL]  += accADC_P2[ROLL];
        accZero[PITCH] += accADC_P2[PITCH];
        accZero[YAW]   += accADC_P2[YAW];
        delay(10); 
      }
      
      // Average
      for (i = 0; i < NUMBEROFAXIS; i++)    // For selected axis in RPY order
      {
        // Round and divide by 32
        accZero[i] = ((accZero[i] + 16) >> 5);
      }

      // Reset zeros to normal cal
      eepromConfig.AccZero_P2[ROLL]  = accZero[ROLL];
      eepromConfig.AccZero_P2[PITCH] = accZero[PITCH];
      eepromConfig.AccZeroNormZ_P2   = accZero[YAW];
      
      // Correct polarity of AccZeroNormZ as per orientation
      eepromConfig.AccZeroNormZ_P2 *= Acc_Pol[Config.Orientation_P2][YAW];
      
      // Flag that normal cal done
      eepromConfig.Main_flags |= (1 << normal_cal_done_P2);
      
      // Save new calibration and flash LED for confirmation
      //EEPROM.put(0x00, eepromConfig);
      Serial.println("Normal Accel P2 Calibration Complete....");
    }
    // P1
    else if (transition <= 5)
    {
      Serial.println("Normal Accel P1 Calibration Start....");
      
      // Get average zero value (over 32 readings)
      for (i = 0; i < 32; i++)
      {
        readRawMPU();
        accZero[ROLL]  += accADC_P1[ROLL];
        accZero[PITCH] += accADC_P1[PITCH];
        accZero[YAW]   += accADC_P1[YAW];
        delay(10);
      }
      
      // Average
      for (i = 0; i < NUMBEROFAXIS; i++)    // For selected axis in RPY order
      {
        // Round and divide by 32
        accZero[i] = ((accZero[i] + 16) >> 5);
      }

      // Reset zeros to normal cal
      eepromConfig.AccZero_P1[ROLL]  = accZero[ROLL];
      eepromConfig.AccZero_P1[PITCH] = accZero[PITCH];
      eepromConfig.AccZeroNormZ_P1   = accZero[YAW];
      
      // Correct polarity of AccZeroNormZ as per orientation
      eepromConfig.AccZeroNormZ_P1 *= Acc_Pol[Config.Orientation_P1][YAW];
      
      // Flag that normal cal done
      eepromConfig.Main_flags |= (1 << normal_cal_done_P1);

      // Save new calibration and flash LED for confirmation
      //EEPROM.put(0x00, eepromConfig);
      Serial.println("Normal Accel P1 Calibration Complete....");
    }
  }
  else
  // Calibrate inverted acc
  {
    // P2 or same
    if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
    {
      // Only update the inverted cal value if preceded by a normal calibration
      if (eepromConfig.Main_flags & (1 << normal_cal_done_P2))
      {
        // Get average zero value (over 32 readings)
        eepromConfig.AccZeroInvZ_P2 = 0;

        for (i = 0; i < 32; i++)
        {
          readRawMPU();
          eepromConfig.AccZeroInvZ_P2 += accADC_P2[YAW];
          delay(10);
        }

        // Round and divide by 32
        eepromConfig.AccZeroInvZ_P2 = ((eepromConfig.AccZeroInvZ_P2 + 16) >> 5);    // Inverted zero point
        
        // Correct polarity of AccZeroInvZ as per orientation
        eepromConfig.AccZeroInvZ_P2 *= Acc_Pol[Config.Orientation_P2][YAW];

        // Test if board is actually inverted relative to board orientation.
        if (eepromConfig.AccZeroInvZ_P2 < 0)
        {
          // Reset zero to halfway between min and max Z
          eepromConfig.AccZeroDiff_P2 = ((eepromConfig.AccZeroNormZ_P2 - eepromConfig.AccZeroInvZ_P2) >> 1);
          
           // Config.AccZero_P2[YAW] is now half-way in between
          eepromConfig.AccZero_P2[YAW] = eepromConfig.AccZeroNormZ_P2 - eepromConfig.AccZeroDiff_P2;

          // Flag that inverted cal done
          eepromConfig.Main_flags |= (1 << inv_cal_done_P2);

          // Save new calibration and flash LED for confirmation
          //EEPROM.put(0x00, eepromConfig);
        }
      }
    } // Orientation-specific code
    
    // P1
    else 
    {
      // Only update the inverted cal value if preceded by a normal calibration
      if (eepromConfig.Main_flags & (1 << normal_cal_done_P1))
      {
        // Get average zero value (over 32 readings)
        eepromConfig.AccZeroInvZ_P1 = 0;

        for (i = 0; i < 32; i++)
        {
          readRawMPU();
          eepromConfig.AccZeroInvZ_P1 += accADC_P1[YAW];
          delay(10);
        }

        // Round and divide by 32
        eepromConfig.AccZeroInvZ_P1 = ((eepromConfig.AccZeroInvZ_P1 + 16) >> 5);    // Inverted zero point
      
        // Correct polarity of AccZeroInvZ as per orientation
        eepromConfig.AccZeroInvZ_P1 *= Acc_Pol[Config.Orientation_P1][YAW];

        // Test if board is actually inverted relative to board orientation.
        if (eepromConfig.AccZeroInvZ_P1 < 0)
        {
          // Reset zero to halfway between min and max Z
          eepromConfig.AccZeroDiff_P1 = ((eepromConfig.AccZeroNormZ_P1 - eepromConfig.AccZeroInvZ_P1) >> 1);
          
          // Config.AccZero_P1[YAW] is now half-way in between
          eepromConfig.AccZero_P1[YAW] = eepromConfig.AccZeroNormZ_P1 - eepromConfig.AccZeroDiff_P1;

          // Flag that inverted cal done
          eepromConfig.Main_flags |= (1 << inv_cal_done_P1);

          // Save new calibration and flash LED for confirmation
          //EEPROM.put(0x00, eepromConfig);
        }
      }     
    }

  } // Calibrate inverted acc
}

//////////////////////////////////////
//////////////////////////////////////

void CalibrateGyrosFast(void)
{
  uint8_t i;
  
  Serial.println("Gyro Fast Calibration Start....");
  
  // Work out which orientation we are calibrating.
  // Only need to do this if the orientations differ.
  // Just do P2 if orientations the same.
  // Will not save new calibration when different and not firmly in P1 or p2.
  if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
  {
    // Clear gyro zeros for the orientation that we are calibrating
    memset(&eepromConfig.gyroZero_P2[ROLL],0,(sizeof(int16_t) * NUMBEROFAXIS));

    // Calculate average over 32 reads
    for (i = 0; i < 32; i++)
    {
      readRawMPU();        // Updates gyroADC_P1/P2[] with the correct orientation-based RPY

      eepromConfig.gyroZero_P2[ROLL]  += gyroADC_P2[ROLL];
      eepromConfig.gyroZero_P2[PITCH] += gyroADC_P2[PITCH];
      eepromConfig.gyroZero_P2[YAW]   += gyroADC_P2[YAW];
    }

    // Average readings for all axis
    for (i = 0; i < NUMBEROFAXIS; i++)
    {
      eepromConfig.gyroZero_P2[i] = (eepromConfig.gyroZero_P2[i] >> 5);
    }
  }
  // P1
  else if (transition <= 5)
  {
    // Clear gyro zeros for the orientation that we are calibrating
    memset(&eepromConfig.gyroZero_P1[ROLL],0,(sizeof(int16_t) * NUMBEROFAXIS));

    // Calculate average over 32 reads
    for (i = 0; i < 32; i++)
    {
      readRawMPU();        // Updates gyroADC_P1/P2[] with the correct orientation-based RPY

      eepromConfig.gyroZero_P1[ROLL]  += gyroADC_P1[ROLL];
      eepromConfig.gyroZero_P1[PITCH] += gyroADC_P1[PITCH];
      eepromConfig.gyroZero_P1[YAW]   += gyroADC_P1[YAW];
    }

    // Average readings for all axis
    for (i = 0; i < NUMBEROFAXIS; i++)
    {
      eepromConfig.gyroZero_P1[i] = (eepromConfig.gyroZero_P1[i] >> 5); // Divide by 32
    }
  }
  
  //EEPROM.put(0x00, eepromConfig);

  Serial.println("Gyro Fast Calibration Complete....");
}

//////////////////////////////////////
//////////////////////////////////////

bool CalibrateGyrosSlow(void)
{
  float     GyroSmooth[NUMBEROFAXIS];
  int16_t   GyroOld[NUMBEROFAXIS] = {0,0,0};
  uint16_t  Stable_counter = 0; 
  uint8_t   axis;
  uint8_t   Gyro_seconds = 0;
  bool      Gyros_Stable = false;

  unsigned long startMillis;
  unsigned long currentMillis;
  const unsigned long period = 1000;

  Serial.println("Gyro Slow Calibration Start....");
  
  // Populate Config.gyroZero[] with ballpark figures
  // This makes slow calibrate setting much more quickly
  CalibrateGyrosFast(); 
  
  // Optimise starting point for each board
  for (axis = 0; axis < NUMBEROFAXIS; axis++)
  {
    // Work out which orientation we are calibrating
    // Only need to do this if the orientations differ
    if ((transition > 95) || (Config.P1_Reference == NO_ORIENT))
    {
      GyroSmooth[axis] = eepromConfig.gyroZero_P2[axis];
    }
    else
    {
      GyroSmooth[axis] = eepromConfig.gyroZero_P1[axis];  
    }   
  }
  
  // Wait until gyros stable. Timeout after CAL_TIMEOUT seconds
  startMillis = millis();
  
  while (!Gyros_Stable && ((Gyro_seconds <= CAL_TIMEOUT)))
  {
    // Update status timeout
    currentMillis = millis();

    // Count elapsed seconds
    if (currentMillis - startMillis >= period)
    {
      Gyro_seconds++;
      startMillis = millis();
    }

    readRawMPU();

    // Calculate very long rolling average
    for (axis = 0; axis < NUMBEROFAXIS; axis++) 
    {
      GyroSmooth[axis] = ((GyroSmooth[axis] * (float)999) + (float)(gyroADC[axis])) / (float)1000;
      
      // See if changing
      if (GyroOld[axis] != (int16_t)GyroSmooth[axis])
      {
        Gyros_Stable = false;
        Stable_counter = 0;
      }
    
      // Save old reading
      GyroOld[axis] = (int16_t)GyroSmooth[axis];
    }
    
    // Increment stable counter to measure how long we are still
    Stable_counter++;
    
    // If stable for 5 seconds, do a quick calibrate
    if (Stable_counter > CAL_STABLE_TIME)
    {
      Gyros_Stable = true;  
      CalibrateGyrosFast();   
    }
    
    delay(1);

    // Otherwise the original saved values are used
  }
  
  Serial.println("Gyro Slow Calibration Complete....");
  
  // Return success or failure
  return(Gyros_Stable);
}

//////////////////////////////////////
//////////////////////////////////////
