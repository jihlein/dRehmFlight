void initEEPROM() {
  eepromConfig.eepromVersion   = 1;
  eepromConfig.AccZeroNormZ_P1 = 0;
  eepromConfig.AccZeroDiff_P1  = 0;
  eepromConfig.AccZeroInvZ_P1  = 0;
  eepromConfig.AccZeroNormZ_P2 = 0;
  eepromConfig.AccZeroDiff_P2  = 0;
  eepromConfig.AccZeroInvZ_P2  = 0;
  eepromConfig.Main_flags      = 0;
  
  for (uint8_t i = 0; i < NUMBEROFAXIS; i++) {
    eepromConfig.AccZero_P2[i]  = 0;
    eepromConfig.AccZero_P1[i]  = 0;
    eepromConfig.gyroZero_P1[i] = 0;
    eepromConfig.gyroZero_P2[i] = 0;
  }
}
