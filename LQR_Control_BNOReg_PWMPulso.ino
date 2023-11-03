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
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19

////////////////////////////////////////////////////////////////////////////////////
// DEFINIR PINES
////////////////////////////////////////////////////////////////////////////////////

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

// Serial Comunication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;
int flag_datos;
//int delay1, delay2 = 1000, delay3, delay4, delay5, delay6, delay7, delay8, delay9, delay10, delay11, delay12, delay13, flag_datos;
byte modo;

////////////////////////////////////////////////////////////////////////////////////
// Declaring Variables
////////////////////////////////////////////////////////////////////////////////////
int motorFRSpeed, motorFLSpeed, motorRRSpeed, motorRLSpeed;
int throttle, battery_voltage;
unsigned long loop_timer;
float vel_gyro_roll_input, vel_gyro_pitch_input, vel_gyro_yaw_input;

float lqr_p_pitch, lqr_i_pitch, lqr_d_pitch;
float lqr_error_temp;
float lqr_i_mem_roll, lqr_roll_setpoint, gyro_roll_input, lqr_output_roll, lqr_last_roll_d_error;
float lqr_i_mem_pitch, lqr_pitch_setpoint, gyro_pitch_input, lqr_output_pitch, lqr_last_pitch_d_error;
float lqr_i_mem_yaw, lqr_yaw_setpoint, gyro_yaw_input, lqr_output_yaw, lqr_last_yaw_d_error;

unsigned long esc_timer1, esc_timer2, esc_timer3, esc_timer4;
unsigned long ESC_Value1 = 1000, ESC_Value2 = 1000, ESC_Value3 = 1000, ESC_Value4 = 1000;

////////////////////////////////////////////////////////////////////////////////////
// PID GAIN AND LIMIT SETTINGS
////////////////////////////////////////////////////////////////////////////////////

float lqr_k1_roll = 2;   //Gain setting for the roll P-Controller
float lqr_k2_roll = 0.5;   //Gain setting for the roll D-Controller
float lqr_ki_roll = 0.005;  //Gain setting for the roll I-Controller

int lqr_max_roll = 400;  //Maximum output of the PID-Controller (+/-)

float lqr_k1_pitch = lqr_k1_roll;  //Gain setting for the pitch P-Controller
float lqr_k2_pitch = lqr_k2_roll;  //Gain setting for the pitch D-Controller
float lqr_ki_pitch = lqr_ki_roll;  //Gain setting for the pitch I-Controller

int lqr_max_pitch = 400;  //Maximum output of the PID-Controller (+/-)

float lqr_k1_yaw = 0.5;   //Gain setting for the yaw P-Controller
float lqr_k2_yaw = 0.5;   //Gain setting for the yaw P-Controller
float lqr_ki_yaw = 0.03;  //Gain setting for the yaw I-Controller

int lqr_max_yaw = 400;  //Maximum output of the PID-Controller (+/-)

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

  delay(1000);
}

void loop() {

  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();
  
    if (throttle <= 1090) {
      modo = 0;
    } else {
      modo = 1;
    }
    newData = false;
  }

  LeerBNO();

  if (modo == 1) {
    calculate_lqr();

    /*if (abs(lqr_roll_setpoint - gyro_roll_input) <= 4.0) {
      lqr_i_mem_roll = lqr_i_mem_roll * 0.8;
    }
    if (abs(lqr_pitch_setpoint - gyro_pitch_input) <= 4.0) {
      //pid_output_pitch = 0;
      lqr_i_mem_pitch = lqr_i_mem_pitch * 0.8;
    }

    if (abs(lqr_yaw_setpoint - gyro_yaw_input) <= 4.0) {
      lqr_i_mem_yaw = lqr_i_mem_yaw * 0.8;
    }*/

    motorFLSpeed = throttle - lqr_output_pitch + lqr_output_roll;  //- pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW)
    motorFRSpeed = throttle - lqr_output_pitch - lqr_output_roll;  // + pid_output_roll + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
    motorRRSpeed = throttle + lqr_output_pitch - lqr_output_roll;  // + pid_output_roll - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW)
    motorRLSpeed = throttle + lqr_output_pitch + lqr_output_roll;  // - pid_output_roll + pid_output_yaw;

    motorFLSpeed = bound(motorFLSpeed, 1100, 1700);
    motorFRSpeed = bound(motorFRSpeed, 1100, 1700);
    motorRRSpeed = bound(motorRRSpeed, 1100, 1700);
    motorRLSpeed = bound(motorRLSpeed, 1100, 1700);

    PWM_Signal();
    showData();
  }

  else {
    motorFLSpeed = 1000;
    motorFRSpeed = 1000;
    motorRLSpeed = 1000;
    motorRRSpeed = 1000;
    lqr_i_mem_pitch = 0;
    lqr_i_mem_roll = 0;
    lqr_i_mem_yaw = 0;
    PWM_Signal();

    showData();
  }
}


void calculate_lqr() {
  //Roll calculations
  lqr_error_temp = gyro_roll_input - lqr_roll_setpoint;
  lqr_i_mem_roll += lqr_ki_roll * lqr_error_temp;
  if (lqr_i_mem_roll > lqr_max_roll) lqr_i_mem_roll = lqr_max_roll;
  else if (lqr_i_mem_roll < lqr_max_roll * -1) lqr_i_mem_roll = lqr_max_roll * -1;

  lqr_output_roll =  lqr_i_mem_roll - (lqr_k1_roll * gyro_roll_input + lqr_k2_roll * vel_gyro_roll_input);
  if (lqr_output_roll > lqr_max_roll) lqr_output_roll = lqr_max_roll;
  else if (lqr_output_roll < lqr_max_roll * -1) lqr_output_roll = lqr_max_roll * -1;

  lqr_last_roll_d_error = lqr_error_temp;

  //Pitch calculations
  lqr_error_temp = gyro_pitch_input - lqr_pitch_setpoint;
  lqr_i_mem_pitch += lqr_ki_pitch * lqr_error_temp;
  if (lqr_i_mem_pitch > lqr_max_pitch) lqr_i_mem_pitch = lqr_max_pitch;
  else if (lqr_i_mem_pitch < lqr_max_pitch * -1) lqr_i_mem_pitch = lqr_max_pitch * -1;

  lqr_output_pitch =  lqr_i_mem_pitch - (lqr_k1_pitch * gyro_pitch_input + lqr_k2_pitch * vel_gyro_pitch_input);
  if (lqr_output_pitch > lqr_max_pitch) lqr_output_pitch = lqr_max_pitch;
  else if (lqr_output_pitch < lqr_max_pitch * -1) lqr_output_pitch = lqr_max_pitch * -1;

  lqr_last_pitch_d_error = lqr_error_temp;

  //Yaw calculations
  lqr_error_temp = gyro_yaw_input - lqr_yaw_setpoint;
  lqr_i_mem_yaw += lqr_ki_yaw * lqr_error_temp;
  if (lqr_i_mem_yaw > lqr_max_yaw) lqr_i_mem_yaw = lqr_max_yaw;
  else if (lqr_i_mem_yaw < lqr_max_yaw * -1) lqr_i_mem_yaw = lqr_max_yaw * -1;

  lqr_output_yaw =  lqr_i_mem_yaw - (lqr_k1_yaw * gyro_yaw_input + lqr_k2_yaw * vel_gyro_yaw_input);
  if (lqr_output_yaw > lqr_max_yaw) lqr_output_yaw = lqr_max_yaw;
  else if (lqr_output_yaw < lqr_max_yaw * -1) lqr_output_yaw = lqr_max_yaw * -1;

  lqr_last_yaw_d_error = lqr_error_temp;
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
  flag_datos = atoi(strtokIndx);

  if(flag_datos == 1){
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    throttle = atoi(strtokIndx);
    throttle = bound(throttle, 1000, 2000);
  }

  else if (flag_datos == 2) {
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_ki_roll = atof(strtokIndx);
    lqr_ki_roll = bound(lqr_ki_roll,0,10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k1_roll = atof(strtokIndx);
    lqr_k1_roll = bound(lqr_k1_roll,0,100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k2_roll = atof(strtokIndx);
    lqr_k2_roll = bound(lqr_k2_roll,0,100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_ki_pitch = atof(strtokIndx);
    lqr_ki_pitch = bound(lqr_ki_pitch,0,10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k1_pitch = atof(strtokIndx);
    lqr_k1_pitch = bound(lqr_k1_pitch,0,10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k2_pitch = atof(strtokIndx);
    lqr_k2_pitch = bound(lqr_k2_pitch,0,100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_ki_yaw = atof(strtokIndx);
    lqr_ki_yaw = bound(lqr_ki_yaw,0,10);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k1_yaw = atof(strtokIndx);
    lqr_k1_yaw = bound(lqr_k1_yaw,0,100);
    strtokIndx = strtok(NULL, ",");  // get the first part - p1
    lqr_k2_yaw = atof(strtokIndx);
    lqr_k2_yaw = bound(lqr_k2_yaw,0,100);
  }
}


void PWM_Signal() {
  esc_timer1 = micros() + motorFLSpeed;
  esc_timer2 = micros() + motorFRSpeed;
  esc_timer3 = micros() + motorRRSpeed;
  esc_timer4 = micros() + motorRLSpeed;

  PORTD |= B11110000;  //Set P12, P11, P10 and P9 to HIGH

  while (PORTD >= 16) {
    if (micros() >= esc_timer1) PORTD &= B11101111;  //Set 4 to low
    if (micros() >= esc_timer2) PORTD &= B11011111;  //Set 5 to low
    if (micros() >= esc_timer3) PORTD &= B10111111;  //Set 6 to low
    if (micros() >= esc_timer4) PORTD &= B01111111;  //Set 7 to low
  }
}

void showData() {
  Serial.print(gyro_roll_input);
  Serial.print(",");
  Serial.print(gyro_pitch_input);
  Serial.print(",");
  Serial.print(gyro_yaw_input);
  Serial.print(",");
  Serial.print(motorFLSpeed);
  Serial.print(",");
  Serial.print(motorFRSpeed);
  Serial.print(",");
  Serial.print(motorRRSpeed);
  Serial.print(",");
  Serial.println(motorRLSpeed);
}

void LeerBNO() {
  // Leer los valores de roll, pitch y yaw en formato de 16 bits (little-endian)
  gyro_yaw_input = readRegister16(BNO055_EUL_Heading_LSB) / 16.0;
  gyro_pitch_input = readRegister16(BNO055_EUL_Roll_LSB) / 16.0;
  gyro_roll_input = readRegister16(BNO055_EUL_Pitch_LSB) / 16.0;

  vel_gyro_roll_input = readRegister16(BNO055_GYR_DATA_Z_LSB) / 16.0;
  vel_gyro_pitch_input = readRegister16(BNO055_GYR_DATA_Y_LSB) / 16.0;
  vel_gyro_yaw_input = readRegister16(BNO055_GYR_DATA_X_LSB) / 16.0;
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
