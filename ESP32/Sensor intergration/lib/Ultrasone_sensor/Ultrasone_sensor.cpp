#include <Arduino.h>
#include <Ultrasone_sensor.h>

void Ultrasone_sensor::read_distance(float &distance){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = (duration*.0343)/2;
}

bool Ultrasone_sensor::begin() {

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig, LOW);
  delay(50);   // let sensor settle

  // test measurement
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long testDuration = pulseIn(echo, HIGH, 30000); // 30ms timeout

  if (testDuration == 0) {
    return false;   // no echo -> problem
  }

  return true;
}