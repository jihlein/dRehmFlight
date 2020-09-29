// 32 bit multiply/scale for broken GCC
// Returns immediately if multiplier is 100, 0 or -100
int16_t scale32(int16_t value16, int16_t multiplier16)
{
  int32_t temp32 = 0;
  int32_t mult32 = 0;

  // No change if 100% (no scaling)
  if (multiplier16 == 100)
  {
    return value16;
  }

  // Reverse if -100%
  else if (multiplier16 == -100)
  {
    return -value16;  
  }

  // Zero if 0%
  else if (multiplier16 == 0)
  {
    return 0; 
  }

  // Only do the scaling if necessary
  else
  {
    // GCC is broken bad regarding multiplying 32 bit numbers, hence all this crap...
    mult32 = multiplier16;
    temp32 = value16;
    temp32 = temp32 * mult32;

    // Divide by 100 and round to get scaled value
    temp32 = (temp32 + (int32_t)50) / (int32_t)100; // Constants need to be cast up to 32 bits
    value16 = (int16_t)temp32;
  }

  return value16;
}
