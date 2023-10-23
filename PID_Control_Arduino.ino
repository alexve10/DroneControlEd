////////////////////////////////////////////////////////////////////////////////////
// LIBRERIAS
////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>  //Include the Wire.h library so we can comunicate with the gyro
#include "MPU9250.h"
#include "I2Cdev.h"  // Tof e IMU
#include <Servo.h>   //Motores

////////////////////////////////////////////////////////////////////////////////////
// DEFINIR PINES
////////////////////////////////////////////////////////////////////////////////////

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define ESC_PIN1 12
#define ESC_PIN2 9
#define ESC_PIN3 10
#define ESC_PIN4 11

//Definir los 4 Motores Brushless
Servo motorFrontRight;
Servo motorRearRight;
Servo motorRearLeft;
Servo motorFrontLeft;

// Serial Comunication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;
int delay1=1000, setpoint_yaw=0, delay2=0;
byte modo;
//Definir el IMU
MPU9250 mpu;

////////////////////////////////////////////////////////////////////////////////////
// Declaring Variables
////////////////////////////////////////////////////////////////////////////////////

int motorFRSpeed, motorFLSpeed, motorRRSpeed, motorRLSpeed;
int throttle, battery_voltage;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

float pid_p_pitch, pid_i_pitch, pid_d_pitch;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

////////////////////////////////////////////////////////////////////////////////////
// PID GAIN AND LIMIT SETTINGS
////////////////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 1.5;   //Gain setting for the roll P-Controller
float pid_i_gain_roll = 0.04;  //Gain setting for the roll I-Controller
float pid_d_gain_roll = 20.0;  //Gain setting for the roll D-Controller

int pid_max_roll = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;   //Gain setting for the pitch P-Controller
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-Controller
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-Controller

int pid_max_pitch = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_yaw = 4.0;   //Gain setting for the yaw P-Controller
float pid_i_gain_yaw = 0.03;  //Gain setting for the yaw I-Controller
float pid_d_gain_yaw = 0.0;   //Gain setting for the yaw D-Controller

int pid_max_yaw = 400;  //Maximum output of the PID-Controller (+/-)

////////////////////////////////////////////////////////////////////////////////////
// SETUP ROUTINE
////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(38400);
  Wire.begin();
  delay(1000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  // Initialize ESC
  motorFrontLeft.attach(ESC_PIN1);
  motorRearRight.attach(ESC_PIN2);
  motorFrontRight.attach(ESC_PIN3);
  motorRearLeft.attach(ESC_PIN4);

  motorFrontLeft.writeMicroseconds(MIN_SIGNAL);
  motorFrontRight.writeMicroseconds(MIN_SIGNAL);
  motorRearRight.writeMicroseconds(MIN_SIGNAL);
  motorRearLeft.writeMicroseconds(MIN_SIGNAL);
  delay(2000);
  Serial.println("ESC Initialized.");
}

void loop() {

  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();
    throttle = delay1;
    if (throttle <= 1100) {
      modo = 0;
    } else {
      modo = 1;
    }
    newData = false;
  }

  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 5) {
      gyro_roll_input = mpu.getRoll();
      gyro_pitch_input = mpu.getPitch();
      gyro_yaw_input = mpu.getYaw();
      
      //gyro_yaw_input = gyro_yaw_input - 180;
      prev_ms = millis();
    }
  }

  if (modo == 1) {
    calculate_pid();
    if (abs(gyro_roll_input)<=0.3){
      //pid_output_roll = 0;
      pid_i_mem_roll = pid_i_mem_roll *0.5;
    }
    if (abs(gyro_pitch_input)<=0.3){
      //pid_output_pitch = 0;
      pid_i_mem_pitch = pid_i_mem_pitch*0.5;
    }

    if (abs(pid_yaw_setpoint-gyro_yaw_input)<=0.5){
      pid_output_yaw = 0;
      pid_i_mem_yaw = 0;
    }
    
    motorFLSpeed = throttle - pid_output_yaw;  //- pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW)
    motorFRSpeed = throttle + pid_output_yaw;  // + pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
    motorRRSpeed = throttle - pid_output_yaw;  // + pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW)
    motorRLSpeed = throttle + pid_output_yaw;  // - pid_output_roll + pid_output_yaw;

    /*motorFLSpeed = throttle + pid_output_pitch - pid_output_roll; //- pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW)
    motorFRSpeed = throttle - pid_output_pitch - pid_output_roll; // + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
    motorRRSpeed = throttle - pid_output_pitch + pid_output_roll; // - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW)
    motorRLSpeed = throttle + pid_output_pitch + pid_output_roll; // + pid_output_yaw;  */

    motorFLSpeed = limit_PWM(motorFLSpeed);
    motorFRSpeed = limit_PWM(motorFRSpeed);
    motorRRSpeed = limit_PWM(motorRRSpeed);
    motorRLSpeed = limit_PWM(motorRLSpeed);

    motorFrontLeft.writeMicroseconds(motorFLSpeed);
    motorRearRight.writeMicroseconds(motorRRSpeed);
    motorRearLeft.writeMicroseconds(motorRLSpeed);
    motorFrontRight.writeMicroseconds(motorFRSpeed);
    showData();

  } else {
    motorFLSpeed =1000;
    motorFRSpeed =1000;
    motorRLSpeed =1000;
    motorRRSpeed =1000;
    pid_i_mem_pitch = 0;
    motorFrontLeft.writeMicroseconds(motorFLSpeed);
    motorRearRight.writeMicroseconds(motorRRSpeed);

    motorFrontRight.writeMicroseconds(motorFRSpeed);
    motorRearLeft.writeMicroseconds(motorRLSpeed);
    showData();
  }
}


void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;

  pid_p_pitch = pid_p_gain_pitch * pid_error_temp;
  pid_i_pitch = pid_i_mem_pitch;
  pid_d_pitch = pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

  pid_output_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void readSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}


void Divir_datos() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");  // get the first part - p1
  delay1 = atoi(strtokIndx);
  delay1 = bound(delay1, 1000, 1600);

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  pid_yaw_setpoint = atoi(strtokIndx);       // convert this part to a float
  pid_yaw_setpoint = bound(pid_yaw_setpoint, -180, 180);
  /*pid_i_gain_roll = atof(strtokIndx);       // convert this part to a float
  pid_i_gain_roll = bound(pid_i_gain_roll, 0, 2);
  pid_roll_setpoint = atoi(strtokIndx);       // convert this part to a float
  pid_roll_setpoint = bound(pid_roll_setpoint, -180, 180); 
  pid_pitch_setpoint = atoi(strtokIndx);       // convert this part to a float
  pid_pitch_setpoint = bound(pid_pitch_setpoint, -180, 180); */
}


void showData() {
  //float SPEED1 = (delay1 - 1000) / 10;
  //Serial.print("Motor speed 1: ");
  Serial.print(percentage(delay1),2);
  //Serial.print("%");
  Serial.print(",");
  Serial.print(gyro_roll_input,2);
  Serial.print(",");
  Serial.print(gyro_pitch_input,2);
  Serial.print(",");
  Serial.print(gyro_yaw_input,2);
  Serial.print(",");
  //Serial.print(pid_yaw_setpoint,2);
  //Serial.print(" ");
  /*Serial.print(pid_output_pitch);*/
  //Serial.print(" ");
  //Serial.print(pid_p_pitch);
  //Serial.print(" ");
  Serial.print(pid_yaw_setpoint,2);
  //Serial.print(" ");
  //Serial.print(pid_d_pitch);
  Serial.print(",");
  Serial.print(motorFLSpeed);
  Serial.print(",");
  Serial.print(motorFRSpeed);
  Serial.print(",");
  Serial.print(motorRRSpeed);
  Serial.print(",");
  Serial.println(motorRLSpeed);
}

float bound(float x, float x_min, float x_max) {
  if (x < x_min) { x = x_min; }
  if (x > x_max) { x = x_max; }
  return x;
}

float percentage(float x){
    float percentage = (x - 1000) / 10;
    return percentage;
}
float limit_PWM(float output) {
  if (output > 1650) output = 1650;
  else if (output < 1100) {
    output = 1100;
  }
  return output;
}
