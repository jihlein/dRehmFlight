#define NUMBEROFAXIS 3
#define NUMBEROFORIENTS 4

enum MainFlags      {inv_cal_done_P1 = 0, normal_cal_done_P1, inv_cal_done_P2, normal_cal_done_P2};
enum Orientation    {UP_BACK = 0, UP_LEFT, UP_FRONT, UP_RIGHT};
enum Polarity       {NORMAL = 0, REVERSED};
enum RPYArrayIndex  {ROLL = 0, PITCH, YAW, ZED};
enum Reference      {NO_ORIENT = 0, EARTH, MODEL};

typedef struct {
  int8_t  CF_factor;
  int8_t  Orientation_P1;
  int8_t  Orientation_P2;
  int8_t  P1_Reference;
} CONFIG_STRUCT;

CONFIG_STRUCT Config;

typedef struct {
  uint8_t eepromVersion;
  int16_t AccZero_P1[NUMBEROFAXIS];
  int16_t AccZeroNormZ_P1;
  int16_t AccZeroDiff_P1;
  int16_t AccZeroInvZ_P1;
  int16_t AccZero_P2[NUMBEROFAXIS];
  int16_t AccZeroNormZ_P2;
  int16_t AccZeroDiff_P2;
  int16_t AccZeroInvZ_P2;
  int16_t gyroZero_P1[NUMBEROFAXIS];
  int16_t gyroZero_P2[NUMBEROFAXIS];
  uint8_t Main_flags;
} EEPROM_STRUCT;

EEPROM_STRUCT eepromConfig;

int16_t accADC[NUMBEROFAXIS];
int16_t accADC_P1[NUMBEROFAXIS];
int16_t accADC_P2[NUMBEROFAXIS];
float   accSmooth[NUMBEROFAXIS];
float   accVertf = 0.0;

int16_t gyroADC[NUMBEROFAXIS];
int16_t gyroADCalt[NUMBEROFAXIS];
int16_t gyroADC_P1[NUMBEROFAXIS];
int16_t gyroADC_P2[NUMBEROFAXIS];

int16_t  angle[2];

int16_t transition = 0;
