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

#define CALIB_STAT 0x35


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
int flag_datos, flag_cal;

byte modo;

////////////////////////////////////////////////////////////////////////////////////
// Declaring Variables
////////////////////////////////////////////////////////////////////////////////////
int motorFRSpeed, motorFLSpeed, motorRRSpeed, motorRLSpeed;
int throttle, battery_voltage;
unsigned long loop_timer, esc_loop_timer;

unsigned long lastTime, now, t1, t2;


float pid_p_pitch, pid_i_pitch, pid_d_pitch;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_output_roll_last, pid_output_pitch_last, pid_output_yaw_last;

float acc_x_input, acc_y_input, acc_z_input, mag_x_input, mag_y_input, mag_z_input;
float desfase_roll = 0, desfase_pitch = 0, desfase_yaw = 0;
unsigned long esc_timer1, esc_timer2, esc_timer3, esc_timer4;
unsigned long ESC_Value1 = 1000, ESC_Value2 = 1000, ESC_Value3 = 1000, ESC_Value4 = 1000;
float val_porcentaje = 0.6;

double gyro_pitch, gyro_roll, gyro_yaw;
float roll_level_adjust, pitch_level_adjust, yaw_level_adjust;
float angle_yaw, angle_pitch, angle_roll;
////////////////////////////////////////////////////////////////////////////////////
// PID GAIN AND LIMIT SETTINGS
////////////////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 1.3;   //Gain setting for the roll P-Controller
float pid_i_gain_roll = 0.04;  //Gain setting for the roll I-Controller
float pid_d_gain_roll = 15.0;  //Gain setting for the roll D-Controller

int pid_max_roll = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-Controller
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-Controller
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-Controller

int pid_max_pitch = 400;  //Maximum output of the PID-Controller (+/-)

float pid_p_gain_yaw = 4.0;   //Gain setting for the yaw P-Controller
float pid_i_gain_yaw = 0.02;  //Gain setting for the yaw I-Controller
float pid_d_gain_yaw = 0.0;   //Gain setting for the yaw D-Controller

int pid_max_yaw = 400;  //Maximum output of the PID-Controller (+/-)

////////////////////////////////////////////////////////////////////////////////////
// SETUP ROUTINE
////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(57600);
  Wire.begin();
  TWBR = 12;  //Set the I2C clock speed to 400kHz.
  delay(1000);

  // Configurar el sensor BNO055
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(0x3D);  // Registro de control
  Wire.write(0x0C);  // Poner el sensor en modo IMU
  Wire.write(0x3B);  // Regitsro de Unidades
  Wire.endTransmission();

  DDRD |= B11110000;  //Configure digital port 7,6,5 and 4 as output

  delay(1000);
  loop_timer = micros();  //Set the timer for the next loop.
}

void loop() {
  now = micros();
  leerVelocidad();
  leerAngulos();
  gyro_roll_input = -((gyro_roll_input * 0.8) + ((gyro_roll) * 0.2));     //Gyro pid input is deg/sec.
  gyro_pitch_input = -((gyro_pitch_input * 0.8) + ((gyro_pitch) * 0.2));  //Gyro pid input is deg/sec.
  gyro_yaw_input = -((gyro_yaw_input * 0.8) + ((gyro_yaw) * 0.2));        //Gyro pid input is deg/sec.
  
  /*readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();

    if (throttle <= 1090) {
      modo = 0;
    } else {
      modo = 1;
    }

    newData = false;
  }*/
  if (modo == 1) {
    calculate_pid();

    //battery_voltage = battery_voltage * 0.92 + (analogRead(0)) * 0.10039;  // 0.09853
    //Turn on the led if battery voltage is to low.
    //if (battery_voltage < 1000 && battery_voltage > 600) digitalWrite(13, HIGH);

    //Turn on the led if battery voltage is to low.
    //if (battery_voltage < 1000 && battery_voltage > 600);

    if (flag_datos == 2) {
      pid_output_roll = pid_output_roll_last;
      pid_output_pitch = pid_output_pitch_last;
      pid_output_yaw = pid_output_yaw_last;
      flag_datos = 0;
    }

    pid_output_roll_last = pid_output_roll;
    pid_output_pitch_last = pid_output_pitch;
    pid_output_yaw_last = pid_output_yaw;

    /*motorFLSpeed = throttle - pid_output_pitch + pid_output_roll;
    motorFRSpeed = throttle - pid_output_pitch - pid_output_roll;
    motorRRSpeed = throttle + pid_output_pitch - pid_output_roll;
    motorRLSpeed = throttle + pid_output_pitch + pid_output_roll;*/

    motorFLSpeed = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    motorFRSpeed = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
    motorRRSpeed = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    motorRLSpeed = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;

    /*motorFLSpeed = throttle - pid_output_yaw;  //Calculate the pulse for esc 1 (front-right - CCW)
    motorFRSpeed = throttle + pid_output_yaw;  //Calculate the pulse for esc 2 (rear-right - CW)
    motorRRSpeed = throttle - pid_output_yaw;  //Calculate the pulse for esc 3 (rear-left - CCW)
    motorRLSpeed = throttle + pid_output_yaw;*/

    motorFLSpeed = bound(motorFLSpeed, 1100, 1700);
    motorFRSpeed = bound(motorFRSpeed, 1100, 1700);
    motorRRSpeed = bound(motorRRSpeed, 1100, 1700);
    motorRLSpeed = bound(motorRLSpeed, 1100, 1700);
  }

  else {
    motorFLSpeed = 1000;
    motorFRSpeed = 1000;
    motorRLSpeed = 1000;
    motorRRSpeed = 1000;
    pid_i_mem_pitch = 0;
    pid_i_mem_roll = 0;
    pid_i_mem_yaw = 0;
  }
  //t1=micros();
  //t2=micros();
  PWM_Signal();
  //later = micros();
  showData();
  //lastTime = now;
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

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
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

  else if (flag_datos == 3) {
    while (flag_cal == 0) {
      getCalibration();
    }
  }
}


void PWM_Signal() {

  while (micros() - loop_timer < 4000)
    ;  //We wait until 4000us are passed.
  lastTime = loop_timer;
  loop_timer = micros();  //Set the timer for the next loop.
  esc_timer1 = loop_timer + motorFLSpeed;
  esc_timer3 = loop_timer + motorFRSpeed;
  esc_timer4 = loop_timer + motorRRSpeed;
  esc_timer2 = loop_timer + motorRLSpeed;

  t1 = micros();
  PORTD |= B11110000;  //Set P12, P11, P10 and P9 to HIGH
  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars); 
    Divir_datos();

    if (throttle <= 1090) {
      modo = 0;
    } else {
      modo = 1;
    }

    newData = false;
  }

  //leerVelocidad();
  //leerAngulos();
  while (PORTD >= 16) {
    esc_loop_timer = micros();
    if (esc_loop_timer >= esc_timer1) PORTD &= B11101111;  //Set 4 to low
    if (esc_loop_timer >= esc_timer2) PORTD &= B11011111;  //Set 5 to low
    if (esc_loop_timer >= esc_timer3) PORTD &= B10111111;  //Set 6 to low
    if (esc_loop_timer >= esc_timer4) PORTD &= B01111111;  //Set 7 to low
  }
  t2 = micros();
}

void showData() {

  Serial.print(angle_roll, 2);
  Serial.print(",");
  Serial.print(angle_pitch, 2);
  Serial.print(",");
  Serial.print(angle_yaw, 2);
  Serial.print(",");
  Serial.print(motorFLSpeed);
  Serial.print(",");
  Serial.print(motorRLSpeed);
  Serial.print(",");
  Serial.print(motorFRSpeed);
  Serial.print(",");
  Serial.println(motorRRSpeed);
  /*double loop_freq = (double)(loop_timer - lastTime);
  double timeChange = (double)(t2 - t1);
  //Serial.print("Ts (ms): ");
  Serial.print(",");
  Serial.print(loop_freq / 1000, 2);
  Serial.print(",");
  Serial.print(timeChange / 1000, 2);
  Serial.println(" ");*/
  /*
  Serial.print(",");
  Serial.print(gyro_yaw_input - pid_yaw_setpoint);
  Serial.print(",");
  Serial.print(pid_i_mem_yaw);
  Serial.print(",");
  Serial.print((gyro_yaw_input - pid_yaw_setpoint) - pid_last_yaw_d_error);
  Serial.print(",");
  Serial.println(pid_output_yaw);*/
}


void leerAngulos() {

  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(BNO055_EUL_Heading_LSB);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 6);
  while (Wire.available() < 6)
    ;
  angle_yaw = (Wire.read() | (Wire.read() << 8)) / 16.0;
  angle_pitch = (Wire.read() | (Wire.read() << 8)) / 16.0;
  angle_roll = (Wire.read() | (Wire.read() << 8)) / 16.0;
  pitch_level_adjust = angle_pitch * 15;  //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;
  yaw_level_adjust = angle_yaw * 15;
  //gyro_yaw_input= gyro_yaw_input/16;
}
void leerVelocidad() {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(BNO055_GYR_DATA_X_LSB);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 6);
  while (Wire.available() < 6)
    ;
  gyro_roll = ((Wire.read() | (Wire.read() << 8)) / 16.0);
  gyro_pitch = ((Wire.read() | (Wire.read() << 8)) / 16.0);
  gyro_yaw = ((Wire.read() | (Wire.read() << 8)) / 16.0);
  
}


float bound(float x, float x_min, float x_max) {
  if (x < x_min) { x = x_min; }
  if (x > x_max) { x = x_max; }
  return x;
}

void getCalibration() {

  int stat_cal = readRegister16(CALIB_STAT) - 3840;
  if (stat_cal < 250) {
    flag_cal = 0;
  } else if (stat_cal >= 254) {
    flag_cal = 1;
  }

  Serial.print("Calibrando...");
  Serial.print(" ");
  Serial.println(stat_cal, BIN);

  while (micros() - loop_timer < 4000)
    ;  //We wait until 4000us are passed.
  lastTime = loop_timer;
  loop_timer = micros();  //Set the timer for the next loop.
  esc_timer1 = loop_timer + motorFLSpeed;
  esc_timer3 = loop_timer + motorFRSpeed;
  esc_timer4 = loop_timer + motorRRSpeed;
  esc_timer2 = loop_timer + motorRLSpeed;

  PORTD |= B11110000;  //Set P12, P11, P10 and P9 to HIGH
  while (PORTD >= 16) {
    esc_loop_timer = micros();
    if (esc_loop_timer >= esc_timer1) PORTD &= B11101111;  //Set 4 to low
    if (esc_loop_timer >= esc_timer2) PORTD &= B11011111;  //Set 5 to low
    if (esc_loop_timer >= esc_timer3) PORTD &= B10111111;  //Set 6 to low
    if (esc_loop_timer >= esc_timer4) PORTD &= B01111111;  //Set 7 to low
  }
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 2);
  int16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}
