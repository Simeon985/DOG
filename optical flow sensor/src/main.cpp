#include <Arduino.h>
#include "Optical_flow_sensor.h"

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_CS   5

// put function declarations here:
Optical_Flow_Sensor flow(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, PAA5100);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== USER CODE START ===");
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(true){
      Serial.println("Initialization of the flow sensor failed");
    }
  }
}

int16_t deltaX,deltaY;
void loop() {

  // put your main code here, to run repeatedly:
  // Get motion count since last call
  flow.readMotionCount(&deltaX, &deltaY);

  Serial.print("X: ");
  Serial.print(deltaX);
  Serial.print(", Y: ");
  Serial.print(deltaY);
  Serial.print("\n");

  delay(100);
}

// put function definitions here:
