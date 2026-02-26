#include <Arduino.h>
#include <Ultrasone_sensor.h>

float Ultrasone_sensor::read_distance(float &distance){
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = (duration*.0343)/2;
}

void Ultrasone_sensor::begin(){
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}