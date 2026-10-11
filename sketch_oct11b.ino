
#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

const uint8_t MPU_ADDR = 0x68;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Power management register
  Wire.write(0x00);  // Wake up sensor
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("ERROR: MPU6050 not responding!");
    while (true) delay(1000);
  }

  Serial.println("MPU6050 initialized!");
  Serial.println("Accelerometer and Gyroscope live feedback");
  Serial.println("------------------------------------------------");
}

void loop() {
  // Read 14 bytes starting from accelerometer registers
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    Serial.println("I2C communication error!");
    delay(500);
    return;
  }

  byte count = Wire.requestFrom(MPU_ADDR, (uint8_t)14);

  if (count != 14) {
    Serial.println("Failed to read sensor data!");
    delay(500);
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  int16_t temp = (Wire.read() << 8) | Wire.read();

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Default MPU6050 sensitivity settings
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  float temperature = temp / 340.0 + 36.53;

  Serial.print("ACC (g): X=");
  Serial.print(accelX, 2);
  Serial.print(" Y=");
  Serial.print(accelY, 2);
  Serial.print(" Z=");
  Serial.print(accelZ, 2);

  Serial.print(" | GYRO (deg/s): X=");
  Serial.print(gyroX, 2);
  Serial.print(" Y=");
  Serial.print(gyroY, 2);
  Serial.print(" Z=");
  Serial.print(gyroZ, 2);

  Serial.print(" | TEMP: ");
  Serial.print(temperature, 1);
  Serial.println(" C");

  delay(100);
}
