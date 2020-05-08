//#include <Arduino.h>
#include "Wire.h"
#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"
#include <Servo.h>
//include <ESP32Servo.h>
#include "SingleEMAFilterLib.h"
#include <SoftwareSerial.h>
SoftwareSerial Camera(13, 12); // RX, TX

SingleEMAFilter<float> CameraEMAfilter(0.2);
SingleEMAFilter<float> accelXEMAfilter(0.1);
SingleEMAFilter<float> accelYEMAfilter(0.1);
SingleEMAFilter<float> accelZEMAfilter(0.1);
SingleEMAFilter<float> angleXEMAfilter(0.1);
SingleEMAFilter<float> angleYEMAfilter(0.1);

float camera_raw;
float camera_filter;
float  camera_offset;

int calibration_status = 1;
int regulation_status = 1;
bool manual_status = 0;
int calibration_pin = 2;
int regulation_pin = 3;
int manual_pin = 4;
bool new_manual_status = 1;
bool old_manual_status = 1;

int r_dir = 12;         // potentiometer value for servo_r
int b_dir = 13;         // potentiometer value for servo_b
int r_sensorValue = 0;  // analog value for sero_r pin
int b_sensorValue = 0;  // analog value for sero_r pin
int servo_angle_r;      // servo_r angle control
int servo_angle_b;      // servo_b angle control
int servo_r[181];       // tranformation 0/-180° to -90/90°
int servo_b[181];       // tranformation 0/-180° to -90/90°


float accel_x;          // measurement of board x angle
float accel_y;          // measurement of board y angle
float accel_z;

float angle_x;          // measurement of board x angle
float angle_y;          // measurement of board y angle
float angle_z;

// tranfer matrix caracteristics
float BB1 = 0.046;
float BB2 = -0.3421;
float C11 = -0.0286;
float C12 = 0.03;
float C21 = -0.0281;
float C22 = -0.0279;
float tmp = C11 * C22 - C12 * C21;

float tmp_x;            // tmp variable
float tmp_y;            // tmp variable

int r;
int b;
int i;

//accelerometer offset correction
// set up the servo to get stable marble position - then correct the accelerometer x and y acceleration
int x_offset = 7;
int y_offset = 15;

float cal_accx;
float cal_accy;
float cal_accz;

unsigned long millis_tmp;

LSM303C myIMU;                  // accelerometer set up
Servo myservor;                 //red shaft
Servo myservob;                 //blue shaft


float Ay = 0;
float Ax = 0;

void setup() {

  Camera.begin(9600);
  Serial.begin(9600);
  myservor.attach(9);
  myservob.attach(10);
  // Serial.println("Starting Marble system");

  i = 180;
  for (int r = 0 ; r <= 180 ; r++) {
    servo_r[r] = i ;
    i = i - 1;
  }

  i = 0;
  for (int b = 0 ; b <= 180 ; b++) {
    servo_b[b] = i ;
    i = i + 1;
  }


  Wire.begin();             //set up I2C bus, comment out if using SPI mode
  Wire.setClock(400000L);   //clock stretching, comment out if using SPI mode
  if (myIMU.begin() != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while (1);
  }

  millis_tmp = 0;





  pinMode(calibration_pin, INPUT_PULLUP);
  pinMode(regulation_pin, INPUT_PULLUP);
  pinMode(manual_pin, INPUT_PULLUP);


  servo_positioning();
  tmp = C11 * C22 - C12 * C21;
  servo_angle_r = (C22 * (Ax - BB1) - C12 * (Ay - BB2)) / tmp;
  servo_angle_b = (-C21 * (Ax - BB1) + C11 * (Ay - BB2)) / tmp;

}

void loop() {


  new_manual_status = digitalRead(manual_pin);

  if ( new_manual_status != old_manual_status )
  {
    if ( new_manual_status == LOW )
    {
      if ( manual_status == false ) {
        manual_status = true;
      }
      else                    {
        manual_status = false;
      }
    }
    old_manual_status = new_manual_status;
    regulation_status = 1;
    calibration_status = 1;
  }

  if (digitalRead(calibration_pin) == 0) {
    //  Serial.println("Calibration Mode");
    board_calibration();
    manual_status = true;
    delay(500);
  }

   else if (digitalRead(regulation_pin) == 0) {
    //  Serial.println("Regulation Mode");
    millis_tmp = millis();
    //    Serial.println(" ");
    Serial.print("time");
    Serial.print(",\t");
    Serial.print("consigne");
    Serial.print(",\t");
    Serial.println("mesure");
    Serial.println(",\t");
    //  Serial.print(" / ");
    // Serial.println("  position bille = ");

    for (i = 0; i <= 360; i++) {
      //  delay(1);
      Ay = 0.5 * sin(3.1416 / 180 * i * 7);
      Ax = 0;
      servo_angle_r = (C22 * (Ax - BB1) - C12 * (Ay - BB2)) / tmp;
      servo_angle_b = (-C21 * (Ax - BB1) + C11 * (Ay - BB2)) / tmp;
      servo_positioning();
      data_monitoring();

      // camera_raw = Camera.parseFloat();
      //   CameraEMAfilter.AddValue(camera_raw);
      //     camera_filter = CameraEMAfilter.GetLowPass();
      //      angleXEMAfilter.AddValue(angle_x);
      //      angle_x = angleXEMAfilter.GetLowPass();
      //  angleYEMAfilter.AddValue(angle_y);
      //  angle_y = angleYEMAfilter.GetLowPass();

      //  if (angle_y <= 5 & angle_y >= -5) {

      Serial.print(millis() - millis_tmp);
      Serial.print(",\t");
      // Serial.print(" / ");
      //    Serial.print(",\t");
      Serial.print(Ay);
      Serial.print(",\t");
      //    Serial.print(" / ");
      Serial.print(angle_y);
      Serial.println(",\t");
      //     Serial.print(" / ");
      //     Serial.println(camera_raw);


      // }

    }

    manual_status == true;
  }

  else if (manual_status == false) {
    Ay = map(analogRead(r_dir), 0, 1023, -4000.00, 4000.00) / 1000.00;
    Ax = map(analogRead(b_dir), 0, 1023, -4000.00, 4000.00) / 1000.00;
    servo_angle_r = (C22 * (Ax - BB1) - C12 * (Ay - BB2)) / tmp;
    servo_angle_b = (-C21 * (Ax - BB1) + C11 * (Ay - BB2)) / tmp;
    servo_positioning();
  }






  if (millis() - millis_tmp >= 1000) {
    data_monitoring();
  }



}


void servo_positioning() {
  myservor.write(map(servo_angle_r, -90, 90, 0, 180));
  myservob.write(map(servo_angle_b, 90, -90, 0, 180));
}


void data_monitoring() {


  accel_x = myIMU.readAccelX();
  delay(8);
  accel_y = myIMU.readAccelY();
  accel_z = myIMU.readAccelZ();

  //delay(5);
  //  accelXEMAfilter.AddValue(accel_x);
  //  accel_x = accelXEMAfilter.GetLowPass();
  //  delay(5);
  //  accelYEMAfilter.AddValue(accel_y);
  //  accel_y = accelYEMAfilter.GetLowPass();
  //  delay(5);
  //  accelZEMAfilter.AddValue(accel_z);
  //  accel_z = accelZEMAfilter.GetLowPass();


  //      Serial.print("\nAccelerometer:\n");
  //      Serial.print(" X = ");
  //      Serial.println(angle_x - x_offset, 4);
  //      Serial.print(" Y = ");
  //      Serial.println(angle_y - y_offset, 4);
  //      Serial.print(" Z = ");
  //      Serial.println(angle_z, 4);


  angle_x = 180 - atan2(accel_y - y_offset, accel_z) * 180 / 3.14;
  if (angle_x >= 300) {
    angle_x = -360 + angle_x;
  }
  angle_y = 180 - atan2(accel_x - x_offset, accel_z) * 180 / 3.14;
  if (angle_y >= 300) {
    angle_y = -360 + angle_y;
  }
  //    Serial.print("\nAccelerometer:\n");
  //    Serial.print(" X = ");
  //    Serial.println(angle_x, 4);
  //    Serial.print(" Y = ");
  //    Serial.println(angle_y, 4);



}

void board_calibration() {
  /* calculation of the following transfer matrix

    |Sr|   |C11  C12 |   |BB1|   |Ax|
    |  | x |         | + |  | = |  |
    |Sb|   |C21  c22 |   |BB2|   |Ay|

    Sr = angle of servo_r
    Sb = angle of servo_b

    Ax = board inclinaison - x direction
    Ay = board inclinaison - y direction

  */

  // Step 1 - calculation of B1 & B2
  servo_angle_r = 0;
  servo_angle_b = 0;
  servo_positioning();
  delay(1000);
  data_monitoring();
  BB1 = angle_x;
  BB2 = angle_y;

  // Step 2 - calculation of C12 & C22
  servo_angle_r = 0;
  servo_angle_b = 60;
  servo_positioning();
  delay(1000);
  data_monitoring();
  C12 = (angle_x - BB1) / servo_angle_b;
  C22 = (angle_y - BB2) / servo_angle_b;


  // Step 3 - calculation of C11 & C12
  servo_angle_r = 60;
  servo_angle_b = 0;
  servo_positioning();
  delay(1000);
  data_monitoring();
  C11 = (angle_x - BB1) / servo_angle_r;
  C21 = (angle_y - BB2) / servo_angle_r;
  Serial.print("BB1 = ");
  Serial.print(BB1, 4);
  Serial.print(" /  BB2 = ");
  Serial.println(BB2, 4);
  Serial.print("C12 = ");
  Serial.print(C12, 4);
  Serial.print(" /  C22 = ");
  Serial.println(C22, 4);
  Serial.print("C11 = ");
  Serial.print(C11, 4);
  Serial.print(" /  C21 = ");
  Serial.println(C21, 4);


  // Check results
  servo_angle_r = 60;
  servo_angle_b = -60;
  servo_positioning();
  delay(1000);
  data_monitoring();
  tmp_x = servo_angle_r * C11 + servo_angle_b * C12 + BB1;
  tmp_y = servo_angle_r * C21 + servo_angle_b * C22 + BB2;
  Serial.print("servo_r = ");
  Serial.print(r_sensorValue);
  Serial.print("servo_b = ");
  Serial.println(b_sensorValue);
  Serial.print("measured angle x = ");
  Serial.print(angle_x);
  Serial.print("computed angle x = ");
  Serial.println(tmp_x);
  Serial.print("measured angle y = ");
  Serial.print(angle_y);
  Serial.print("computed angle y = ");
  Serial.println(tmp_y);



  // Check the possibility of inverse matrix
  tmp = C11 * C22 - C12 * C21;
  // Serial.print("C11*C22-C12*C21 =");
  // Serial.println(tmp, 4);

  // horizontallity calcultation

  Ax = 0;
  Ay = 0;

  servo_angle_r = (C22 * (Ax - BB1) - C12 * (Ay - BB2)) / tmp;
  servo_angle_b = (-C21 * (Ax - BB1) + C11 * (Ay - BB2)) / tmp;
  Serial.print("*****************servo_angle_r =");
  Serial.print(servo_angle_r);
  Serial.print("*****************servo_angle_b =");
  Serial.print(servo_angle_b);
  servo_positioning();
  delay(500);
  data_monitoring();
}
