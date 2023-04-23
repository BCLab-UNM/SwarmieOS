//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Wire.h>

//Custom libraries located in arduino directory
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <Odometry.h>

// Constants
// #define PI 3.14159265358979323846
#define RAD2DEG(radianAngle) (radianAngle * 180.0 / PI)
#define DEG2RAD(degreeAngle) (degreeAngle * PI / 180.0)

////////////////
////Settings////
////////////////

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input

//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;

//Serial (USB <--> Intel NUC)
unsigned long watchdogTimer = 1000; //fail-safe in case of communication link failure (in ms)
unsigned long lastCommTime = 0; //time of last communication from NUC (in ms)

////////////////////////////
////Class Instantiations////
////////////////////////////

L3G gyroscope;
LSM303 magnetometer_accelerometer;
LPS pressure;
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Odometry odom = Odometry(rightEncoderA, rightEncoderB, leftEncoderA, leftEncoderB);
LSM303::vector<int16_t> acc;
L3G::vector<int16_t> gyro;
LSM303::vector<int16_t> mag;

/////////////
////Setup////
/////////////

void setup()
{
  Serial.begin(460800);
  while (!Serial) {} //wait for Serial to complete initialization before moving on
  Wire.begin();
  if (imuStatus()) {imuInit();}
}


/////////////////
////Main Loop////
/////////////////

void loop() {
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  if (!gyroscope.timeoutOccurred() && !magnetometer_accelerometer.timeoutOccurred()) {
    //Collect updated values
    acc = magnetometer_accelerometer.a;
    gyro = gyroscope.g;
    mag = magnetometer_accelerometer.m;
    Serial.print("IMU,1,");
    Serial.print(acc.x); 
    Serial.print(",");
    Serial.print(acc.y); 
    Serial.print(",");
    Serial.print(acc.z); 
    Serial.print(",");
    Serial.print(mag.x); 
    Serial.print(",");
    Serial.print(mag.y); 
    Serial.print(",");
    Serial.print(mag.z); 
    Serial.print(",");
    Serial.print(gyro.x);
    Serial.print(",");
    Serial.print(gyro.y);
    Serial.print(",");
    Serial.println(gyro.z);
  } //end if data is good
  
  odom.update();
  Serial.print("ODOM,1,");
  Serial.print(odom.left);
  Serial.print(",");
  Serial.print(odom.right);
  Serial.print(",");
  Serial.println(odom.clock);
               
  if (Serial.available()) {
    char c = Serial.read();
      if (c == 'v') {
        int speedL = Serial.parseInt();
        int speedR = Serial.parseInt();
        
        if (speedL >= 0 && speedR >= 0) {
          move.forward(speedL, speedR);
        }
        else if (speedL <= 0 && speedR <= 0) {
          move.backward(speedL*-1, speedR*-1);
        }
        else if (speedL <= 0 && speedR >= 0) {
          move.rotateLeft(speedL*-1, speedR);
        }
        else {
          move.rotateRight(speedL, speedR*-1);
        }
      }
      else if (c == 's') {move.stop();}
      lastCommTime = millis();
  }
  if (millis() - lastCommTime > watchdogTimer) { move.stop(); }
} //end loop function


////////////////////////////
////Initializer Functions///
////////////////////////////

//Initialize gyroscope, accelerometer, magnetometer, and pressure gauge
void imuInit() {
  gyroscope.init();
  gyroscope.enableDefault();
  gyroscope.setTimeout(1);

  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();

  magnetometer_accelerometer.setTimeout(1);

  pressure.init();
  pressure.enableDefault();
}


////////////////////////////
////Diagnostic Functions////
////////////////////////////

//Check for valid I2C connection to verify IMU
bool imuStatus() {
  byte numberOfDevices = 0;
  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (!error) { numberOfDevices++; }
  } //end for all devices
  return (numberOfDevices > 0);
} // end imuStatus function
