#include <Arduino.h>
#include <Wire.h>

#define ICM20948_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  // Wake up
  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(0x06); // PWR_MGMT_1
  Wire.write(0x01); // Wake
  Wire.endTransmission();
  
  delay(100);
}

void loop() {
  uint8_t data[6];
  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(0x2D); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20948_ADDR, 6);
  
  if (Wire.available() == 6) {
    for (int i=0; i<6; i++) data[i] = Wire.read();
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3];
    int16_t az = (data[4] << 8) | data[5];
    
    Serial.printf("A: %d, %d, %d\n", ax, ay, az);
  }
  delay(500);
}
