/*
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  Serial.println("TEST OK");
}

void loop() {

  delay(1000);
}
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{
  Serial.begin(9600);
  Wire.begin(21, 22);

  if (!bno.begin())
  {
    Serial.println("BNO055 NIET GEVONDEN");
    while (1)
      ;
  }

  delay(1000);
  Serial.println("BNO055 OK");
}

void loop()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  Serial.print("Yaw: ");
  Serial.print(euler.x());
  Serial.print(" Pitch: ");
  Serial.print(euler.y());
  Serial.print(" Roll: ");
  Serial.println(euler.z());
  Serial.println("Hallo");
  delay(500);
}
  