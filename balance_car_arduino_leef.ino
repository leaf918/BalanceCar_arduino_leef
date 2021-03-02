///////////////////////////////////
#include "L298N/L298N.h"
const unsigned int motor_pin_a_1=4;
const unsigned int motor_pin_a_2=5;
const unsigned int motor_pin_b_1=6;
const unsigned int motor_pin_b_2=7;

L298N myMotor(motor_pin_a_1,motor_pin_a_2);
///////////////////////////////////

///////////////////////////////////
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev/I2Cdev.h"
#include "MPU6050_light/MPU6050_light.h"


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu;
MPU6050 mpu(Wire);

int16_t ax,ay,az,tmp,gx,gy,gz;
///////////////////////////////////
// Hall encoder
#include "Encoder/Encoder.h"

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(10,11);
long oldPosition  = -999;

//   avoid using pins with LEDs attached
///////////////////////////////////
#include "TrivialKalmanFilter/TrivialKalmanFilter.h"
#define ONE_WIRE_BUS A1         // The pin DS18B20 is connected to.
#define DT_COVARIANCE_RK 4.7e-3 // Estimation of the noise covariances (process)
#define DT_COVARIANCE_QK 1e-5   // Estimation of the noise covariances (observation)
TrivialKalmanFilter<float> filter(DT_COVARIANCE_RK, DT_COVARIANCE_QK);

void setup() {
    Serial.begin(115200);//Initialize the serial port
    // Initialize the MPU6050
    mpu_initialize();
    // Do nothing for 2 milliseconds
    delay(2);
//    HallEncoderInit();
}
void mpu_initialize(){

    Wire.begin();

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(); // gyro and accelero
    Serial.println("Done!\n");
}
//void HallEncoderInit() {
//    Direction = true;//default -> Forward
//    pinMode(encoderB, INPUT);
//    attachInterrupt(0, hallChangeHandler1, CHANGE);
//}

//void hallChangeHandler1() {
//    int Lstate = digitalRead(encoderA);
//    if ((encoderALast == LOW) && Lstate == HIGH) {
//        int val = digitalRead(encoderB);
//        if (val == LOW && Direction) {
//            Direction = false; //Reverse
//        } else if (val == HIGH && !Direction) {
//            Direction = true;  //Forward
//        }
//    }
//    encoderALast = Lstate;
//
//    if (!Direction) duration++;
//    else duration--;
//}

void loop() {
    // put your main code here, to run repeatedly:
//    myMotor.forward();
//    delay(3000);
//    myMotor.backward();
//    delay(3000);
    // read raw accel/gyro measurements from device
    Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
    Serial.print("\tY: ");Serial.print(mpu.getAccY());
    Serial.print("\tZ: ");Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
    Serial.print("\tY: ");Serial.print(mpu.getGyroY());
    Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
    Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
    Serial.print("\tY: ");Serial.print(mpu.getAngleY());
    Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));
    // hall encoder
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;
        Serial.println(newPosition);
    }

}
