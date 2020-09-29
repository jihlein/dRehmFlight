//***********************************************************
//* imu.c
//*
//* IMU code ported from KK2V1_1V12S1Beginner code
//* by Rolf Bakke and Steveis
//*
//* Ported to OpenAeroVTOL by David Thompson (C)2014
//*
//***********************************************************

/////////////////////////////////////
// Defines
/////////////////////////////////////

#define acc_0_4G_SQ       2621.0f  // (0.40 * ACCSENSITIVITY) * (0.40 * ACCSENSITIVITY)
#define acc_0_85G_SQ     11837.0f  // (0.85 * ACCSENSITIVITY) * (0.85 * ACCSENSITIVITY)
#define acc_1_15G_SQ     21668.0f  // (1.15 * ACCSENSITIVITY) * (1.15 * ACCSENSITIVITY)
#define acc_1_6G_SQ      41943.0f  // (1.60 * ACCSENSITIVITY) * (1.60 * ACCSENSITIVITY)
#define GYROSENSRADIANS 0.017045f  // Calculate factor for Gyros to report directly in rad/s
                                   // For +/-2000 deg/s FS, gyro sensitivity for 12 bits (+/-2048) = (4000/4096) = 0.97656 deg/s/lsb
                                   // 0.97656 * Pi/180 = 0.017044 rad/s/lsb
#define MAXDELTAANGLE      0.2618f // Limit possible instantaneous change in angle to +/-15 degrees (720 deg/s)
#define SMALLANGLEFACTOR     0.66f // Empirically calculated to produce exactly 20 at 20 degrees

/////////////////////////////////////
// Globals
/////////////////////////////////////

float AccAnglePitch, AccAngleRoll;
float EulerAngleRoll, EulerAnglePitch;
float GyroPitchVC, GyroRollVC, GyroYawVC;
float VectorA, VectorB;
float VectorNewA, VectorNewB;

float VectorX = 0;  // Initialise the vector to point straight up
float VectorY = 0;
float VectorZ = 1;

/////////////////////////////////////
/////////////////////////////////////

void imu_update(float intervalf)
{
  float     tempf;
  int8_t    axis;
  uint32_t  roll_sq, pitch_sq, yaw_sq;
  uint32_t  AccMag = 0;
    
  //************************************************************
  // Acc LPF
  //************************************************************  

  // Smooth Acc signals - note that accSmooth is in [ROLL, PITCH, YAW] order
  for (axis = 0; axis < NUMBEROFAXIS; axis++)
  {
    // Use raw accADC[axis] as source for acc values when filter off
    accSmooth[axis] =  -accADC[axis];
  }
  
  // Add correction data to gyro inputs based on difference between Euler angles and acc angles
  AccAngleRoll  = accSmooth[ROLL] * SMALLANGLEFACTOR;
  AccAnglePitch = accSmooth[PITCH] * SMALLANGLEFACTOR;

  // Alter the gyro sources to the IMU as required.
  // Using gyroADCalt[] always assures that the right gyros are associated with the IMU
  GyroRollVC  = gyroADCalt[ROLL];
  GyroPitchVC = gyroADCalt[PITCH];
  GyroYawVC   = gyroADCalt[YAW];

  // Calculate acceleration magnitude.
  roll_sq  = (accADC[ROLL] * accADC[ROLL]);
  pitch_sq = (accADC[PITCH] * accADC[PITCH]);
  yaw_sq   = (accADC[YAW] * accADC[YAW]);
  AccMag   = roll_sq + pitch_sq + yaw_sq;
  
  // Add acc correction if inside local acceleration bounds and not inverted according to VectorZ
  // NB: new dual autolevel code needs acc correction at least temporarily when switching profiles.
  // This is actually a kind of Complementary Filter
  //if ((AccMag > acc_0_85G_SQ) && (AccMag < acc_1_15G_SQ) && (VectorZ > 0.5)) // Original code
  
  // New test code - only adjust when in acc mag limits and when upright or dual AL code
  if  (((AccMag > acc_0_85G_SQ) && (AccMag < acc_1_15G_SQ) && (VectorZ > 0.5) && (Config.P1_Reference == NO_ORIENT)) || // Same as always when "Same" 
     ((AccMag > acc_0_4G_SQ) && (AccMag < acc_1_6G_SQ) && (Config.P1_Reference != NO_ORIENT))) 
  {
    // Default Config.CF_factor is 6 (1 - 10 = 10% to 100%, 6 = 60%)
    tempf = (EulerAngleRoll - AccAngleRoll) / 10;
    tempf = tempf * (12 - Config.CF_factor); 
    GyroRollVC = GyroRollVC + tempf;
    
    tempf = (EulerAnglePitch - AccAnglePitch) / 10;
    tempf = tempf * (12 - Config.CF_factor);
    GyroPitchVC = GyroPitchVC + tempf;
  }

  // Rotate up-direction 3D vector with gyro inputs
  Rotate3dVector(intervalf);
  ExtractEulerAngles();
  
  // Upscale to 0.01 degrees resolution and copy to angle[] for display
  angle[ROLL] = (int16_t)(EulerAngleRoll * -100);
  angle[PITCH] = (int16_t)(EulerAnglePitch * -100);
}

void Rotate3dVector(float intervalf)
{
  float theta;
  
  // Rotate around X axis (pitch)
  theta = thetascale(GyroPitchVC, intervalf);
  VectorA = VectorY;
  VectorB = VectorZ;
  RotateVector(theta);
  VectorY = VectorNewA;
  VectorZ = VectorNewB;

  // Rotate around Y axis (roll)
  theta = thetascale (GyroRollVC, intervalf);
  VectorA = VectorX;
  VectorB = VectorZ;
  RotateVector(theta);
  VectorX = VectorNewA;
  VectorZ = VectorNewB;

  // Rotate around Z axis (yaw)
  theta = thetascale(GyroYawVC, intervalf);
  VectorA = VectorX;
  VectorB = VectorY;
  RotateVector(theta);
  VectorX = VectorNewA;
  VectorY = VectorNewB;
}

void RotateVector(float angle)
{
  VectorNewA = VectorA * small_cos(angle) - VectorB * small_sine(angle);
  VectorNewB = VectorA * small_sine(angle) + VectorB * small_cos(angle);
}

float thetascale(float gyro, float intervalf)
{
  float theta;
  
  // intervalf = time in seconds since last measurement
  // GYROSENSRADIANS = conversion from raw gyro data to rad/s
  // theta = actual number of radians moved

  theta = (gyro * GYROSENSRADIANS * intervalf);
  
  // The sin() and cos() functions don't appreciate large 
  // input values. Limit the input values to +/-15 degrees. 
  
  if (theta > MAXDELTAANGLE)
  {
    theta = MAXDELTAANGLE;
  }
  
  if (theta < -MAXDELTAANGLE)
  {
    theta = -MAXDELTAANGLE;
  }
  
  return theta;
}

// Small angle approximations of Sine, Cosine
// NB:  These *only* work for small input values.
//    Larger values will produce fatal results
float small_sine(float angle)
{
  // sin(angle) = angle
  return angle;
}

float small_cos(float angle)
{
  // cos(angle) = (1 - (angle^2 / 2))
  float temp;
  
  temp = (angle * angle) / 2;
  temp = 1 - temp;

  return temp;
}

void ExtractEulerAngles(void)
{
  EulerAngleRoll = ext2(VectorX);
  EulerAnglePitch = ext2(VectorY);
}

float ext2(float Vector)
{
  float temp;
  
  // Rough translation to Euler angles
  temp = Vector * 90;

  // Change 0-90-0 to 0-90-180 so that
  // swap happens at 100% inverted
  if (VectorZ < 0)
  {
    // CW rotations
    if (temp > 0)
    {
      temp = 180 - temp;
    }
    // CCW rotations
    else
    {
      temp = -180 - temp;
    }
  }

  return (temp);
}

void reset_IMU(void)
{
  // Initialise the vector to point straight up
  VectorX = 0;
  VectorY = 0;
  VectorZ = 1;
  
  // Initialise internal vectors and attitude 
  VectorA = 0;
  VectorB = 0;
  EulerAngleRoll = 0;
  EulerAnglePitch = 0;
}
