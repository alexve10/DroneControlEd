#include <Wire.h>  //Include the Wire.h library so we can comunicate with the gyro
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
unsigned long esc_timer;
unsigned long ESC_Value=1000;

// Serial Comunication
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
boolean newData = false;
int delay1=1000;
byte modo;

//Definir el IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float acc_x_input, acc_y_input, acc_z_input, mag_x_input, mag_y_input, mag_z_input, gyro_roll_input, gyro_pitch_input, gyro_yaw_input;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  DDRB |= B00010000; //Configure digital port 12 and 13 as output
  //pinMode(12,OUTPUT)

}

void loop() {
  readSerial();
  if (newData == true) {
    strcpy(tempChars, receivedChars);  // Copia para proteger la data original ya que strtok() reemplaza las comas con  "\0"
    Divir_datos();
    ESC_Value = delay1;
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
}


void PWM_Signal(){
  ESC_Value = bound(ESC_Value, 1000, 2000);
  esc_timer = micros()+ESC_Value;
  // put your main code here, to run repeatedly:
  PORTB |= B00010000; //Set P12 to HIGH
  while(micros() < esc_timer); //wait 1500s  
  PORTB &= B11101111; //Set P12 to low
}

void showData() {
  Serial.print(ESC_Value);
  Serial.print(" ");
  Serial.print(gyro_roll_input,4);
  Serial.print(" ");
  Serial.print(gyro_pitch_input,4);
  Serial.print(" ");
  Serial.println(gyro_yaw_input,4);
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
