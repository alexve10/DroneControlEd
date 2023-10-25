#include <Wire.h>  //Include the Wire.h library so we can comunicate with the gyro
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
unsigned long esc_timer1, esc_timer2, esc_timer3, esc_timer4;
unsigned long ESC_Value1=1000, ESC_Value2=1000, ESC_Value3=1000, ESC_Value4=1000;

// Serial Comunication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;
int delay1=1000, delay2 = 1000, delay3 = 1000, delay4 = 1000;
byte modo;

//Definir el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float acc_x_input, acc_y_input, acc_z_input, mag_x_input, mag_y_input, mag_z_input, gyro_roll_input, gyro_pitch_input, gyro_yaw_input;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  DDRD |= B11110000;  //Configure digital port 7,6,5 and 4 as output                                                                  //Configure digital poort 4, 5, 6 and 7 as output.
  //DDRB |= B00011110; //Configure digital port 12 and 11 as output
  
  //pinMode(12,OUTPUT)

}

void loop() {
  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();
    ESC_Value1 = delay1;
    ESC_Value2 = delay2;
    ESC_Value3 = delay3;
    ESC_Value4 = delay4;

    newData = false;
  }

  PWM_Signal();
  LeerBNO();
  showData();
  
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
  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  delay2 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  delay3 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  delay4 = atoi(strtokIndx);

}


void PWM_Signal(){
  ESC_Value1 = bound(ESC_Value1, 1000, 2000);
  ESC_Value2 = bound(ESC_Value2, 1000, 2000);
  ESC_Value3 = bound(ESC_Value3, 1000, 2000);
  ESC_Value4 = bound(ESC_Value4, 1000, 2000);

  esc_timer1 = micros()+ESC_Value1;
  esc_timer2 = micros()+ESC_Value2;
  esc_timer3 = micros()+ESC_Value3;
  esc_timer4 = micros()+ESC_Value4;

  PORTD |= B11110000; //Set P12, P11, P10 and P9 to HIGH

  while(PORTD >= 16){
    if(micros() >= esc_timer1)PORTD &= B11101111; //Set 4 to low
    if(micros() >= esc_timer2)PORTD &= B11011111; //Set 5 to low
    if(micros() >= esc_timer3)PORTD &= B10111111; //Set 6 to low
    if(micros() >= esc_timer4)PORTD &= B01111111; //Set 7 to low

  }
}

void showData() {
  Serial.print(gyro_roll_input,4);
  Serial.print(" ");
  Serial.print(gyro_pitch_input,4);
  Serial.print(" ");
  Serial.print(gyro_yaw_input,4);
  Serial.print(" ");
  Serial.print(ESC_Value1);
  Serial.print(" ");
  Serial.print(ESC_Value2);
  Serial.print(" ");
  Serial.print(ESC_Value3);
  Serial.print(" ");
  Serial.println(ESC_Value4);
  
}

void LeerBNO(){
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  gyro_roll_input = orientationData.orientation.z;
  gyro_pitch_input = orientationData.orientation.y;
  gyro_yaw_input = orientationData.orientation.x;

}

float bound(float x, float x_min, float x_max) {
  if (x < x_min) { x = x_min; }
  if (x > x_max) { x = x_max; }
  return x;
}
