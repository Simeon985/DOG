
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("BNO055 niet gevonden");
    while (1);
  }

  delay(1000);
  Serial.println("Kompas OK");
}

void loop() {
  imu::Vector<3> euler =
      bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float heading = euler.x();   // kompasrichting

  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" graden");

  delay(500);
  }