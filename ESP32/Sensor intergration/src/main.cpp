#include <Arduino.h>
#include <Optical_flow_sensor.h>
#include <Ultrasone_sensor.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#define PIN_SCK_OFS_1  18
#define PIN_MISO_OFS_1 19
#define PIN_MOSI_OFS_1 23
#define PIN_CS_OFS_1   5
#define PIN_TRIG_US_1 9
#define PIN_ECHO_US_1 10
#define PIN_SDA_IMU 21
#define PIN_SCL_IMU 22
#define IMU_SENSOR_ID 55

// put function declarations here:
Optical_Flow_Sensor flow(PIN_SCK_OFS_1, PIN_MISO_OFS_1, PIN_MOSI_OFS_1, PIN_CS_OFS_1, PAA5100);
Ultrasone_sensor ultra(PIN_TRIG_US_1, PIN_ECHO_US_1);
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  if (!flow.begin()) {
    while(true){
      Serial.println("Initialization of the flow sensor failed");
    }
  }
  ultra.begin();
  Wire.begin(PIN_SDA_IMU, PIN_SCL_IMU);
  if (!bno.begin()) {
    Serial.println("BNO055 niet gevonden");
    while (1);
  }

  delay(1000);
}

int16_t deltaX,deltaY;
float distance;
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

  ultra.read_distance(distance);
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);

  imu::Vector<3> euler =
    bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  float heading = euler.x();   // kompasrichting

  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" graden");

  delay(500);
}

// put function definitions here:
