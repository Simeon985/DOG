#include <Arduino.h>
#include <Ultrasone_sensor.h>

static Ultrasone_sensor* _instance = nullptr;

static void IRAM_ATTR echoISR_wrapper(void* arg) {
    Ultrasone_sensor* sensor = (Ultrasone_sensor*)arg;
    sensor->echoISR();
}

<<<<<<< HEAD
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
=======
void Ultrasone_sensor::echoISR() {
    if (digitalRead(echo)) {
        _echoStart = micros();          // rising edge
    } else {
        _echoEnd   = micros();          // falling edge
        _echoReady = true;
    }
}

void Ultrasone_sensor::trigger() {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
}

void Ultrasone_sensor::update() {

    unsigned long now = millis();

    // Trigger every 30 ms
    if (now - lastTrigger >= 30) {
        trigger();
        lastTrigger = now;
    }
    // If ISR completed echo pulse
    if (_echoReady) {

        noInterrupts();
        duration = _echoEnd - _echoStart;
        _echoReady = false;
        interrupts();
        distance = (duration * 0.0343f) / 2.0f;
    }
}



// Replaces read_distance() — just returns last known good value
float Ultrasone_sensor::get_distance() {
    update();
    return distance;
}

bool Ultrasone_sensor::begin() {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trig, LOW);
    delay(50);

    _instance = this;                  // register singleton for ISR
    attachInterruptArg(
    digitalPinToInterrupt(echo),
    echoISR_wrapper,
    this,
    CHANGE
);

    // test measurement
    trigger();
    delay(30);
    return (_echoReady);               // begin() returns true if echo came back
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
}