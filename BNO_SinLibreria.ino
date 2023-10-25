#include <Wire.h>
#define BNO055_ADDRESS 0x28

#define BNO055_EUL_Heading_LSB 0x1A
#define BNO055_EUL_Heading_MSB 0x1B
#define BNO055_EUL_Roll_LSB 0x1C
#define BNO055_EUL_Roll_MSB 0x1D
#define BNO055_EUL_Pitch_LSB 0x1E
#define BNO055_EUL_Pitch_MSB 0x1F

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Configurar el sensor BNO055
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(0x3D);  // Registro de control
  Wire.write(0x0C);  // Poner el sensor en modo IMU
  Wire.write(0x3B);  // Regitsro de Unidades
  Wire.endTransmission();
}

void loop() {
  // Leer los valores de roll, pitch y yaw en formato de 16 bits (little-endian)
  float yaw = readRegister16(BNO055_EUL_Heading_LSB) / 16.0;
  float pitch = readRegister16(BNO055_EUL_Roll_LSB) / 16.0;
  float roll = readRegister16(BNO055_EUL_Pitch_LSB) / 16.0;

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);
}

int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDRESS, 2);
  int16_t value = Wire.read() | (Wire.read() << 8);
  return value;
}
