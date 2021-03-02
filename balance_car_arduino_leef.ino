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
#include "MPU6050/MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

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

void setup() {
    Serial.begin(115200);//Initialize the serial port
    // Initialize the MPU6050
    mpu.initialize();
    // Do nothing for 2 milliseconds
    delay(2);
//    HallEncoderInit();
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
//    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    //获取当前内部温度
//    tmp = mpu.getTemperature();
//    // display tab-separated accel/gyro x/y/z values
//    Serial.print("ax = ");
//    Serial.print(ax);
//    Serial.print(" | ay = ");
//    Serial.print(ay);
//    Serial.print(" | az = ");
//    Serial.print(az);
//    Serial.print(" | tmp = ");
//    Serial.print(tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
//    Serial.print(" | gx = ");
//    Serial.print(gx);
//    Serial.print(" | gy = ");
//    Serial.print(gy);
//    Serial.print(" | gz = ");
//    Serial.println(gz);
//    delay(400);
    // hall encoder
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
        oldPosition = newPosition;
        Serial.println(newPosition);
    }

}
