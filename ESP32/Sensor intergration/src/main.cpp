#include <Arduino.h>
#include <Optical_flow_sensor.h>
#include <Matrix_LED.h>
#include <Ultrasone_sensor.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>



#define PIN_SCK_OFS  18
#define PIN_MISO_OFS 19
#define PIN_MOSI_OFS 23
#define PIN_CS_OFS_1 5
#define PIN_CS_OFS_2 14

#define PIN_TRIG_US_2 17
#define PIN_ECHO_US_2 16
#define PIN_TRIG_US_1 32
#define PIN_ECHO_US_1 34

#define PIN_SDA_IMU 22
#define PIN_SCL_IMU 21
#define IMU_SENSOR_ID 55
<<<<<<< HEAD
<<<<<<< HEAD
#define TIMER_INTERVAL 50000 // 50000 MICROs = 20 Hz
=======
#define TIMER_INTERVAL 10000 // 50000 MICROs = 20 Hz
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
=======
#define TIMER_INTERVAL 100000 // 50000 MICROs = 20 Hz
>>>>>>> 78d5bee5deb08ca3a02406559424a55c75b51375



//put function declarations here:
Optical_Flow_Sensor flow1(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_1, PAA5100);
Optical_Flow_Sensor flow2(PIN_SCK_OFS, PIN_MISO_OFS, PIN_MOSI_OFS, PIN_CS_OFS_2, PAA5100);
Ultrasone_sensor ultra1(PIN_TRIG_US_1, PIN_ECHO_US_1);
Ultrasone_sensor ultra2(PIN_TRIG_US_2, PIN_ECHO_US_2);
<<<<<<< HEAD
//Adafruit_BNO055 bno = Adafruit_BNO2055(IMU_SENSOR_ID);
=======
Adafruit_BNO055 bno = Adafruit_BNO055(IMU_SENSOR_ID);
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
Matrix_LED animation;
hw_timer_t *timer = NULL;
volatile bool timerFired = false;
void IRAM_ATTR onTimer() {
    timerFired = true;  // keep ISR short!
}

void print_error(String sensor){
  while(true){
    Serial.print("Initialization of the ");
    Serial.print(sensor);
    Serial.println(" failed");
  }
}

void setup() {
  Serial.begin(921600);
  delay(1000);

<<<<<<< HEAD
  // if (!bno.begin()) { print_error("BNO055 sensor)");}
  // Serial.println("BNO055 initialized");
=======
  if (!bno.begin()) { print_error("BNO055 sensor)");}

>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08

  if (!flow1.begin()) { print_error("Flow sensor 1)");}
  Serial.println("Flow sensor 1 initialized");

  if (!flow2.begin()) { print_error("Flow sensor 2)");}
  Serial.println("Flow sensor 2 initialized");

<<<<<<< HEAD
  if (!ultra1.begin()) {print_error("Ultrasone sensor 1");}
  Serial.println("Ultrasone sensor 1 initialized");
  if (!ultra2.begin()) {print_error("Ultrasone sensor 2");}
  Serial.println("Ultrasone sensor 2 initialized");
<<<<<<< HEAD
  // if (!Wire.begin(PIN_SDA_IMU, PIN_SCL_IMU)) { print_error("BNO055 sensor)");}
  // Serial.println("BNO055 wire initialized");


  //bno.setExtCrystalUse(true); 
=======
=======
  // if (!ultra1.begin()) {print_error("Ultrasone sensor 1");}
  // Serial.println("Ultrasone sensor 1 initialized");
  // if (!ultra2.begin()) {print_error("Ultrasone sensor 2");}
  // Serial.println("Ultrasone sensor 2 initialized");
>>>>>>> 78d5bee5deb08ca3a02406559424a55c75b51375
  if (!Wire.begin(PIN_SDA_IMU, PIN_SCL_IMU)) { print_error("BNO055 sensor)");}
  Serial.println("BNO055 wire initialized");



  //bno.setExtCrystalUse(true);
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
  timer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz (1 tick = 1µs)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_INTERVAL, true);  // 1/10 000 µs = 100 Hz
  timerAlarmEnable(timer);

<<<<<<< HEAD

=======
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
}


int16_t deltaX1, deltaY1;
int16_t deltaX2, deltaY2;
unsigned long previous_time = micros();
unsigned long current_time;
float distance1=0;
float distance2=0;
<<<<<<< HEAD
// imu::Vector<3> euler;
// imu::Vector<3> gyro;
// imu::Vector<3> lin_acc;
=======
imu::Vector<3> euler;
imu::Vector<3> gyro;
imu::Vector<3> lin_acc;
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
float heading = 0;
float gyro_x = 0;
float lin_acc_x = 0;
float lin_acc_y = 0;
int state_led=0;


void loop() {
  //Get motion count since last call
  if (timerFired){
    timerFired = false;

  // reading data optical flow sensors
  flow1.readMotionCount(&deltaX1, &deltaY1);
  flow2.readMotionCount(&deltaX2, &deltaY2);

  // reading data ultrasone sensors
<<<<<<< HEAD
<<<<<<< HEAD
  ultra1.read_distance(distance1);
  ultra2.read_distance(distance2);

  //reading data IMU
  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // heading = euler.x();
  // gyro_x = gyro.z();
  // lin_acc_x = lin_acc.x();
  // lin_acc_y = lin_acc.y(); 
=======
  distance1=ultra1.get_distance();
  distance2=ultra2.get_distance();
=======
  // distance1=ultra1.get_distance();
  // distance2=ultra2.get_distance();
>>>>>>> 78d5bee5deb08ca3a02406559424a55c75b51375

  //reading data IMU
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  heading = euler.x();
  gyro_x = gyro.z();
  lin_acc_x = lin_acc.x();
  lin_acc_y = lin_acc.y();
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08

  // updating time
  current_time = micros();

  // updating animation
<<<<<<< HEAD
  animation.render();
=======
  animation.update(current_time,distance1);
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08

  //print everything in one line

  Serial.print(heading);
  Serial.print(" ");
  Serial.print(gyro_x);
  Serial.print("   ");
  Serial.print(lin_acc_x);
  Serial.print(" ");
  Serial.print(lin_acc_y);
  Serial.print("   ");
  Serial.print(distance1);
  Serial.print(" ");
  Serial.print(distance2);
<<<<<<< HEAD
  Serial.print(" ");
=======
  Serial.print("   ");
>>>>>>> e48cb9e3ee88c59a8769192c9f5b5818bb4a5d08
  Serial.print(deltaX1);
  Serial.print(" ");
  Serial.print(deltaY1);
  Serial.print("   ");
  Serial.print(deltaX2);
  Serial.print(" ");
  Serial.print(deltaY2);
  Serial.print("   ");
  Serial.println(current_time-previous_time);
  previous_time = current_time;
  }
}