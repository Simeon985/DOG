#include <Arduino.h>
#include <Optical_flow_sensor.h>
#include <Ultrasone_sensor.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <MD_MAX72xx.h>

#define  DEBUG  1

#if  DEBUG
#define PRINT(s, x) { Serial.print(F(s)); Serial.print(x); }
#define PRINTS(x) Serial.print(F(x))
#define PRINTD(x) Serial.println(x, DEC)

#else
#define PRINT(s, x)
#define PRINTS(x)
#define PRINTD(x)

#endif

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
// Optical_Flow_Sensor flow1(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_1, PAA5100);
// Optical_Flow_Sensor flow2(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_2, PAA5100);
// Ultrasone_sensor ultra(PIN_TRIG_US_1, PIN_ECHO_US_1);
// Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
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
  // if (!flow1.begin()) {
  //   while(true){
  //     Serial.println("Initialization of the flow sensor 1 failed");
  //   }
  // }
  // if (!flow2.begin()) {
  //   while(true){
  //     Serial.println("Initialization of the flow sensor 2 failed");
  //   }
  // }
  // ultra.begin();
  // Wire.begin(PIN_SDA_IMU, PIN_SCL_IMU);
  // if (!bno.begin()) {
  //   Serial.println("BNO055 niet gevonden");
  //   while (1);
  // }
  // bno.setExtCrystalUse(true);
  // timerBegin(timer number, prescaler, count up)
  timer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz (1 tick = 1µs)
  timerAttachInterrupt(timer, &onTimer, true);
  // timerAlarmWrite(timer, ticks, repeat)
  timerAlarmWrite(timer, 100000, true);  // 1/10 000 µs = 100 Hz
  timerAlarmEnable(timer);

  //initialize the LED displays
  if (!mx.begin()){
    Serial.println("\nMD_MAX72XX initialization failed");
  }
  
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
  // flow1.readMotionCount(&deltaX1, &deltaY1);
  // flow2.readMotionCount(&deltaX2, &deltaY2);

  //   // reading data ultrasone sensors
  //   ultra.read_distance(distance);

  // //reading data IMU
  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // heading = euler.x();
  // gyro_x = gyro.z();
  // lin_acc_x = lin_acc.x();
  // lin_acc_y = lin_acc.y();

    // printing all the data

  current_time = micros();
  mx.setPoint(0,0,1);
  mx.update();
  // Serial.print(current_time - previous_time);
  // Serial.print("micros \n");

    // // data US
    //Serial.print("Distance: ");
    //Serial.println(distance);

    //driver for the LED screens
    // for (u_int8_t i = 3; i < 5; i++){
    //   mx.setRow(i, 255);
    //   mx.setColumn(i, 255);
    // }
  //mx.setPoint(0,0,1);
  //data IMU
  //print everything in one line
  // Serial.print(current_time-previous_time);
  // Serial.print(" ");
  // Serial.print(heading);
  // Serial.print(" ");
  // Serial.print(gyro_x);
  // Serial.print("   ");
  // Serial.print(lin_acc_x);
  // Serial.print(" ");
  // Serial.print(lin_acc_y);
  // Serial.print("   ");
  // Serial.print(deltaX1);
  // Serial.print(" ");
  // Serial.print(deltaY1);
  // Serial.print("   ");
  // Serial.print(deltaX2);
  // Serial.print(" ");
  // Serial.print(deltaY2);
  // Serial.print("   ");
  // Serial.println(distance);
  // previous_time = current_time;
  }
}

// put function definitions here:
