////////////////////////////////////////////////////////////////////////////////////
// BNO055
////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#define BNO055_ADDRESS 0x28

#define BNO055_EUL_Heading_LSB 0x1A
#define BNO055_EUL_Heading_MSB 0x1B
#define BNO055_EUL_Roll_LSB 0x1C
#define BNO055_EUL_Roll_MSB 0x1D
#define BNO055_EUL_Pitch_LSB 0x1E
#define BNO055_EUL_Pitch_MSB 0x1F

////////////////////////////////////////////////////////////////////////////////////
// DEFINIR PINES
////////////////////////////////////////////////////////////////////////////////////

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

// Serial Comunication
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;
int flag_datos;

byte modo;

////////////////////////////////////////////////////////////////////////////////////
// Declaring Variables
////////////////////////////////////////////////////////////////////////////////////
int motorFRSpeed, motorFLSpeed, motorRRSpeed, motorRLSpeed;
int throttle, battery_voltage;
unsigned long loop_timer;

float pid_p_pitch, pid_i_pitch, pid_d_pitch;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float acc_x_input, acc_y_input, acc_z_input, mag_x_input, mag_y_input, mag_z_input;
float desfase_roll = 0, desfase_pitch = 0, desfase_yaw = 0;
unsigned long esc_timer1, esc_timer2, esc_timer3, esc_timer4;
unsigned long ESC_Value1 = 1000, ESC_Value2 = 1000, ESC_Value3 = 1000, ESC_Value4 = 1000;
float val_porcentaje = 0.6;

////////////////////////////////////////////////////////////////////////////////////
// PID GAIN AND LIMIT SETTINGS
////////////////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 3.0;   //Gain setting for the roll P-Controller
float pid_i_gain_roll = 0.05;  //Gain setting for the roll I-Controller
float pid_d_gain_roll = 30.0;  //Gain setting for the roll D-Controller

int pid_max_roll = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-Controller
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-Controller
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-Controller

int pid_max_pitch = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_yaw = 4.0;   //Gain setting for the yaw P-Controller
float pid_i_gain_yaw = 0.03;  //Gain setting for the yaw I-Controller
float pid_d_gain_yaw = 0.0;   //Gain setting for the yaw D-Controller

int pid_max_yaw = 400;  //Maximum output of the PID-Controller (+/-)

//

String datos;

////////////////////////////////////////////////////////////////////////////////////
// SETUP ROUTINE
////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);

  // Configurar el sensor BNO055
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(0x3D);  // Registro de control
  Wire.write(0x0C);  // Poner el sensor en modo IMU
  Wire.write(0x3B);  // Regitsro de Unidades
  Wire.endTransmission();

  DDRD |= B11110000;  //Configure digital port 7,6,5 and 4 as output                                                                  //Configure digital poort 4, 5, 6 and 7 as output.

  LeerBNO();
  pid_roll_setpoint = gyro_roll_input;
  pid_pitch_setpoint = gyro_pitch_input;
  delay(1000);
}

void loop() {
  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();

    if (throttle <= 1090) {
      modo = 0;
    } 
    else {
      modo = 1;
    }

    newData = false;
  }

  LeerBNO();

  if (modo == 1) {
    calculate_pid();
    if (abs(pid_roll_setpoint - gyro_roll_input) <= 3.0) {
      //pid_output_roll = 0;
      pid_i_mem_roll = pid_i_mem_roll * val_porcentaje;
    }
    if (abs(pid_pitch_setpoint - gyro_pitch_input) <= 3.0) {
      //pid_output_pitch = 0;
      pid_i_mem_pitch = pid_i_mem_pitch * val_porcentaje;
    }

    if (abs(pid_yaw_setpoint - gyro_yaw_input) <= 3.0) {
      pid_i_mem_yaw = pid_i_mem_yaw * val_porcentaje;
    }

    motorFLSpeed = throttle - pid_output_pitch + pid_output_roll;  //- pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW)
    motorFRSpeed = throttle - pid_output_pitch - pid_output_roll;  // + pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
    motorRRSpeed = throttle + pid_output_pitch - pid_output_roll;  // + pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW)
    motorRLSpeed = throttle + pid_output_pitch + pid_output_roll;  // - pid_output_roll + pid_output_yaw;

    motorFLSpeed = bound(motorFLSpeed, 1100, 1700);
    motorFRSpeed = bound(motorFRSpeed, 1100, 1700);
    motorRRSpeed = bound(motorRRSpeed, 1100, 1700);
    motorRLSpeed = bound(motorRLSpeed, 1100, 1700);
    
    /*motorRRSpeed = motorRRSpeed+20;
    motorRLSpeed = motorRLSpeed+13*/;

    PWM_Signal();
    showData();

  }

  else {
    motorFLSpeed = 1000;
    motorFRSpeed = 1000;
    motorRLSpeed = 1000;
    motorRRSpeed = 1000;
    pid_i_mem_pitch = 0;
    pid_i_mem_roll = 0;
    pid_i_mem_yaw = 0;
    PWM_Signal();

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


void Divir_datos() {                    // split the data into its parts
  char* strtokIndx;                     // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");  // get the first part - p1
  flag_datos = atoi(strtokIndx);

  if (flag_datos == 1) {
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    throttle = atoi(strtokIndx);
    throttle = bound(throttle, 1000, 2000);
  }

  else if (flag_datos == 2) {
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_p_gain_roll = atof(strtokIndx);
    pid_p_gain_roll = bound(pid_p_gain_roll, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_i_gain_roll = atof(strtokIndx);
    pid_i_gain_roll = bound(pid_i_gain_roll, 0, 10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_d_gain_roll = atof(strtokIndx);
    pid_d_gain_roll = bound(pid_d_gain_roll, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_p_gain_pitch = atof(strtokIndx);
    pid_p_gain_pitch = bound(pid_p_gain_pitch, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_i_gain_pitch = atof(strtokIndx);
    pid_i_gain_pitch = bound(pid_i_gain_pitch, 0, 10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_d_gain_pitch = atof(strtokIndx);
    pid_d_gain_pitch = bound(pid_d_gain_pitch, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_p_gain_yaw = atof(strtokIndx);
    pid_p_gain_yaw = bound(pid_p_gain_yaw, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_i_gain_yaw = atof(strtokIndx);
    pid_i_gain_yaw = bound(pid_i_gain_yaw, 0, 10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_d_gain_yaw = atof(strtokIndx);
    pid_d_gain_yaw = bound(pid_d_gain_yaw, 0, 100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_roll_setpoint = atof(strtokIndx);
    pid_roll_setpoint = bound(pid_roll_setpoint, -40, 40);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_pitch_setpoint = atof(strtokIndx);
    pid_pitch_setpoint = bound(pid_pitch_setpoint, -40, 40);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    pid_yaw_setpoint = atof(strtokIndx);
    pid_yaw_setpoint = bound(pid_yaw_setpoint, -360, 360);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    val_porcentaje = atof(strtokIndx);
    val_porcentaje = bound(val_porcentaje, 0, 1);
  }
}


void PWM_Signal() {

  /*esc_timer1 = micros() + motorFLSpeed;
  esc_timer2 = micros() + motorFRSpeed;
  esc_timer3 = micros() + motorRRSpeed;
  esc_timer4 = micros() + motorRLSpeed;*/


  esc_timer1 = micros() + motorFLSpeed;
  esc_timer3 = micros() + motorFRSpeed;
  esc_timer4 = micros() + motorRRSpeed;
  esc_timer2 = micros() + motorRLSpeed;


  PORTD |= B11110000;  //Set P12, P11, P10 and P9 to HIGH

  while (PORTD >= 16) {
    if (micros() >= esc_timer1) PORTD &= B11101111;  //Set 4 to low
    if (micros() >= esc_timer2) PORTD &= B11011111;  //Set 5 to low
    if (micros() >= esc_timer3) PORTD &= B10111111;  //Set 6 to low
    if (micros() >= esc_timer4) PORTD &= B01111111;  //Set 7 to low
  }
}

void showData() {
  Serial.print(gyro_roll_input,1);
  Serial.print(",");
  Serial.print(gyro_pitch_input,1);
  Serial.print(",");
  Serial.print(gyro_yaw_input,1);
  Serial.print(",");
  Serial.print(motorFLSpeed);
  Serial.print(",");
  Serial.print(motorRLSpeed);
  Serial.print(",");
  Serial.print(motorFRSpeed);
  Serial.print(",");
  Serial.println(motorRRSpeed);
  /*Serial.print(",");
  Serial.print(pid_roll_setpoint);
  Serial.print(",");
  Serial.println(pid_pitch_setpoint);*/
}

void LeerBNO() {
  // Leer los valores de roll, pitch y yaw en formato de 16 bits (little-endian)
  gyro_yaw_input = readRegister16(BNO055_EUL_Heading_LSB) / 16.0;
  gyro_pitch_input = readRegister16(BNO055_EUL_Roll_LSB) / 16.0;
  gyro_roll_input = readRegister16(BNO055_EUL_Pitch_LSB) / 16.0;
  //gyro_roll_input = gyro_roll_input-3.19;
  //gyro_pitch_input = gyro_pitch_input +0.94;
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 2);
  int16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}

float bound(float x, float x_min, float x_max) {
  if (x < x_min) { x = x_min; }
  if (x > x_max) { x = x_max; }
  return x;
}

