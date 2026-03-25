#include <Arduino.h>
#include <Optical_flow_sensor.h>
#include <Ultrasone_sensor.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <MD_MAX72xx.h>

#define PIN_SCK_OFS  18
#define PIN_MISO_OFS 19
#define PIN_MOSI_OFS 23
#define PIN_CS_OFS_1 5
#define PIN_CS_OFS_2 14

#define PIN_TRIG_US_1 17
#define PIN_ECHO_US_1 16

#define PIN_SDA_IMU 21
#define PIN_SCL_IMU 22
#define IMU_SENSOR_ID 55

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES	2
#define CLK_PIN_LED   25  // or SCK
#define DATA_PIN_LED  32  // or MOSI
#define CS_PIN_LED    26  // or SS

//put function declarations here:

Optical_Flow_Sensor flow1(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_1, PAA5100);
Optical_Flow_Sensor flow2(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_2, PAA5100);
Ultrasone_sensor ultra(PIN_TRIG_US_1, PIN_ECHO_US_1);
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
hw_timer_t *timer = NULL;
volatile bool timerFired = false;
void IRAM_ATTR onTimer() {
    timerFired = true;  // keep ISR short!
}
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN_LED, CLK_PIN_LED, CS_PIN_LED, MAX_DEVICES);

void setup() {
  // put your setup code here, to run once:

  #if DEBUG
    Serial.begin(921600);
  #endif
  delay(1000);


  if (!bno.begin()) {
    while (true){
    Serial.println("BNO055 niet gevonden");
    }
  }
  Serial.println("BNO055 initialized");



  if (!flow1.begin()) {
    while(true){
      Serial.println("Initialization of the flow sensor 1 failed");
    }
  }
  Serial.println("Flow sensor 1 initialized");

  if (!flow2.begin()) {
    while(true){
      Serial.println("Initialization of the flow sensor 2 failed");
    }
  }
  Serial.println("Flow sensor 2 initialized");


  ultra.begin();
  Wire.begin(PIN_SDA_IMU, PIN_SCL_IMU);

  bno.setExtCrystalUse(true);
  timer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz (1 tick = 1µs)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);  // 1/10 000 µs = 100 Hz
  timerAlarmEnable(timer);

  //initialize the LED displays
  if (!mx.begin()){
    Serial.println("\nMD_MAX72XX initialization failed");
  }
  Serial.println("\nMD_MAX72XX initialization succeeded");
  Serial.println("Timing flow\n");
  mx.clear();
  mx.update();
}

int16_t deltaX1, deltaY1;
int16_t deltaX2, deltaY2;
unsigned long previous_time = micros();
unsigned long current_time;
float distance;
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> lin_acc;
float heading;
float gyro_x;
float lin_acc_x;
float lin_acc_y;

void loop() {
  // put your main code here, to run repeatedly:
  //Get motion count since last call
  if (timerFired){
    timerFired = false;

  // reading data optical flow sensors
  flow1.readMotionCount(&deltaX1, &deltaY1);
  flow2.readMotionCount(&deltaX2, &deltaY2);

  // reading data ultrasone sensors
  ultra.read_distance(distance);

  // //reading data IMU
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  heading = euler.x();
  gyro_x = gyro.z();
  lin_acc_x = lin_acc.x();
  lin_acc_y = lin_acc.y();
current_time = micros();
const u_int8_t heart[8] = {
  0b00000000,  // Row 0: ........
  0b01100110,  // Row 1: .XX..XX.
  0b11111111,  // Row 2: XXXXXXXX
  0b11111111,  // Row 3: XXXXXXXX
  0b01111110,  // Row 4: .XXXXXX.
  0b00111100,  // Row 5: ..XXXX..
  0b00011000,  // Row 6: ...XX...
  0b00000000   // Row 7: ........
};

// Display the heart
for (u_int8_t row = 0; row < 8; row++) {
  mx.setRow(row, heart[row]);
}

  //data IMU
  //print everything in one line
  Serial.print(current_time-previous_time);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(gyro_x);
  Serial.print("   ");
  Serial.print(lin_acc_x);
  Serial.print(" ");
  Serial.print(lin_acc_y);
  Serial.print("   ");
  Serial.print(deltaX1);
  Serial.print(" ");
  Serial.print(deltaY1);
  Serial.print("   ");
  Serial.print(deltaX2);
  Serial.print(" ");
  Serial.print(deltaY2);
  Serial.print("   ");
  Serial.println(distance);
  previous_time = current_time;
  }
}

// put function definitions here:
