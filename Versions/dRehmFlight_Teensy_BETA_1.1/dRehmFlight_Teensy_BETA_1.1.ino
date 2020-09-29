//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Version: Beta 1.1

/*
 * 
 * If you are using this for an academic or scholarly project, please credit me in any presentations or publications:
 *
 * Nicholas Rehm
 * Department of Aerospace Engineering
 * University of Maryland
 * College Park 20742
 * Email: nrehm@umd.edu
 *
 */

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//CREDITS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Skeleton code for reading and initializing MPU6050 borrowed from:
https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by
brian.taylor@bolderflight.com
http://www.bolderflight.com

*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//USER-SPECIFIED DEFINES

//Uncomment only one receiver type
//#define USE_PPM_RX
//#define USE_PWM_RX
#define USE_SBUS_RX

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//REQUIRED LIBRARIES
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //commanding any extra actuators, installed with teensyduino installer
#include <EEPROM.h>

#if defined USE_SBUS_RX
  #include <SBUS.h>   //sBus interface
#endif

#include <MPU9250.h>
MPU9250 mpu9250(SPI2,36);

#include "typedefs.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//USER-SPECIFIED VARIABLES
//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;    //integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;    //max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode (maximum ~400)
float maxPitch = 30.0;   //max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode (maximum ~400)
float maxYaw = 160.0;    //max yaw rate in deg/sec (maximum ~400)

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (if using controlANGLE2, MUST be 0.0. Suggested default for controlANGLE is 0.05)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;      //Yaw P-gain
float Ki_yaw = 0.05;     //Yaw I-gain
float Kd_yaw = 0.00015;   //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//DECLARE PINS
//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the IMU
//Radio:
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;
//Motor pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;
//PWM outputs:
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;
const int servo4Pin = 9;
const int servo5Pin = 10;
const int servo6Pin = 11;
const int servo7Pin = 12;
PWMServo servo1;  //create servo object to control a servo
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//DECLARE GLOBAL VARIABLES
//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
//Radio comm:
float channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
float channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif

int radio_command = 1;

//IMU:
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float roll_correction, pitch_correction;
float beta = 0.04; //madgwick filter parameter 
float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//SETUP
void setup() {
  Serial.begin(500000); //usb serial
  delay(1500);

  EEPROM.get(0x00, eepromConfig);
    
  if (eepromConfig.eepromVersion != 1) {
    initEEPROM();
    EEPROM.put(0x00, eepromConfig);
  }

  loadSettings();
  
  //Initialize all pins
  pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  //Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  delay(20);

  //Initialize radio communication
  #if defined USE_PPM_RX
    readPPM_setup(PPM_Pin);
  #elif defined USE_PWM_RX
    readPWM_setup(ch1Pin, ch2Pin, ch3Pin, ch4Pin, ch5Pin, ch6Pin);
  #elif defined USE_SBUS_RX
    sbus.begin();
  #else
    #error No RX type defined
  #endif

  //Set radio channels to default (safe) values
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialize IMU communication
  initMPU();

  delay(10);

  //Get IMU error to calibrate attitude, assuming vehicle is level
  //calculate_IMU_error();
  //calibrateAttitude(); //helps to warm up IMU and Madgwick filter

  delay(10);

  //Arm servo channels
  servo1.write(0); //command servo angle from 0-180 degrees (1000 - 2000 PWM)
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  
  delay(10);

  //Arm motors
  m1_command_PWM = 125;
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  commandMotors();
  
  delay(200);
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,150,70); //numBlinks, upTime (ms), downTime (ms) -- 3 quick blinks indicates entering main loop!
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//MAIN LOOP
void loop() {
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0; 

  loopBlink(); //indicate we are in main loop with short blink every 1.5 seconds

  if (Serial.available() > 0) {
    uint8_t msgByte = Serial.read();

    if (msgByte == 71 || msgByte == 103) {  // "G" or "g" received as msgByte
      CalibrateGyrosSlow();
    }

    if (msgByte == 78 || msgByte == 110) {  // "N" or "n" recevied as msgByte
      CalibrateAcc(NORMAL);
    }

    if (msgByte == 82 || msgByte == 114) {  // "R" or "r" recevied as msgByte
      CalibrateAcc(REVERSED);
    }
  }
  
  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE
  //printRadioData();     //radio pwm values 
  //printRawSbus();       //prints raw sbus channels
  //printDesiredState();  //desired vehicle state commanded in either degrees or degrees/sec
  //printGyroData();      //prints filtered gyro data direct from IMU
  //printAccelData();     //prints filtered accelerometer data direct from IMU
  printRollPitch();     //prints roll, pitch, and yaw angles in degrees from Madgwick filter 
  //printPIDoutput();     //prints computed stabilized PID variables from controller and desired setpoint
  //printMotorCommands(); //prints the values being written to the motors
  //printLoopRate();      //prints the time between loops in microseconds
  
  //Get vehicle state
  readMPU(); //pulls raw gyro and accel data from IMU and LP filters to remove noise
  imu_update(dt);
  
  //Sensor_PID(dt);
  //CalculatePID();
  //ProcessMixer();
  
  scaleCommands(); //scales motor commands to 125-250 range (oneshot125 protocol) and servo commands to 0-180 (for servo library)

  //Throttle cut check
  throttleCut(); //directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); 
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);
    
  //Get vehicle commands for next loop iteration
  getCommands(); //pulls current available radio commands
  failSafe(); //prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(1000);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//FUNCTIONS

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //between -1 and 1
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //between -maxYaw and +maxYaw

  roll_passthru = roll_des/(2*maxRoll);
  pitch_passthru = pitch_des/(2*maxPitch);
  yaw_passthru = yaw_des/(2*maxYaw);
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 - 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motors and servos.
   */
  //Quad mixing
  //m1 = front left, m2 = front right, m3 = back right, m4 = back left
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0 is zero throttle if connecting to ESC for conventional PWM, 1 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;

  //Example use of the linear fader for float type variables. Linearly interpolates between minimum and maximum values for Kp_pitch_rate variable based on state of channel 6
  /*
  if (channel_6_pwm > 1500){
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 5.5, 1, 2000); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
  }
  else if (channel_6_pwm < 1500) {
    Kp_pitch_rate = floatFaderLinear(Kp_pitch_rate, 0.1, 0.3, 2.5, 0, 2000); //parameter, minimum value, maximum value, fadeTime, state (0 min or 1 max), loop frequency
  }
  */
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. The radio commands are retrieved from a function in the readPWM
   * file separate from this one which is running a bunch of interrupts to continuously update the radio readings. 
   * The raw radio commands are being filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  radio_command = 1;
  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.610128;  
      float bias  = 894.7529; 
      channel_1_pwm = sbusChannels[0] * scale + bias;
      channel_2_pwm = sbusChannels[1] * scale + bias;
      channel_3_pwm = sbusChannels[2] * scale + bias;
      channel_4_pwm = sbusChannels[3] * scale + bias;
      channel_5_pwm = sbusChannels[4] * scale + bias;
      channel_6_pwm = sbusChannels[5] * scale + bias;     
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.2;
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands().
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  digitalWrite(m5Pin, HIGH);
  digitalWrite(m6Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 6 ) { //keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, LOW);
      wentLow = wentLow + 1;
      flagM5 = 1;
    } 
    if ((m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
      digitalWrite(m6Pin, LOW);
      wentLow = wentLow + 1;
      flagM6 = 1;
    } 
  }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //constrain param within max bounds
  
  return param;
}


void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
   * Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
   * minimum for oneshot125 protocol, 1000 is minimum for standard PWM) if channel 5 is high. This is the last function 
   * called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
   * the motors to anything other than minimum value. Safety first. 
   */
  if (channel_5_pwm > 1500) {
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
    m5_command_PWM = 120;
    m6_command_PWM = 120;
  }
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters scattered around this code.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.println(channel_6_pwm);
  }
}

void printRawSbus() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    #if defined USE_SBUS_RX
      Serial.print(F(" CH1: "));
      Serial.print(sbusChannels[0]);
      Serial.print(F(" CH2: "));
      Serial.print(sbusChannels[1]);
      Serial.print(F(" CH3: "));
      Serial.print(sbusChannels[2]);
      Serial.print(F(" CH4: "));
      Serial.print(sbusChannels[3]);
      Serial.print(F(" CH5: "));
      Serial.print(sbusChannels[4]);
      Serial.print(F(" CH6: "));
      Serial.println(sbusChannels[5]);
    #else
      Serial.println("Error - sBus not selected....");
    #endif
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(gyroADC_P1[ROLL]);
    Serial.print(F(" GyroY: "));
    Serial.print(gyroADC_P1[PITCH]);
    Serial.print(F(" GyroZ: "));
    Serial.println(gyroADC_P1[YAW]);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(accADC_P1[ROLL]);
    Serial.print(F(" AccY: "));
    Serial.print(accADC_P1[PITCH]);
    Serial.print(F(" AccZ: "));
    Serial.println(accADC_P1[YAW]);
  }
}

void printRollPitch() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(angle[ROLL]);
    Serial.print(F(" pitch: "));
    Serial.println(angle[PITCH]);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command: "));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command: "));
    Serial.println(m6_command_PWM);
  }
}

void printLoopRate() {
    if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}
