#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t sys, gyro, accel, mag;
void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("BNO055 niet gevonden");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  Serial.println("Kompas OK");
}

void loop() {
  imu::Vector<3> euler =
      bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x();  //kompasrichting in graden
  Serial.println(heading);
  delay(20);
  }