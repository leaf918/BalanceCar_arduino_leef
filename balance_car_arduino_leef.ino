// Motor Driver
#include "L298N/L298N.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev/I2Cdev.h"
#include "MPU6050_light/MPU6050_light.h"
// Hall encoder
#include "Encoder/Encoder.h"
// Kalman filter
#include "TrivialKalmanFilter/TrivialKalmanFilter.h"
// ms timer
#include "MsTimer2/MsTimer2.h"
// PID control
# include "AutoPID/AutoPID.h"

/////////////////////////////////// Block Motor
const unsigned int motor_pin_a_1 = 4;
const unsigned int motor_pin_a_2 = 5;
const unsigned int motor_pin_b_1 = 6;
const unsigned int motor_pin_b_2 = 7;
L298N my_motor_left(3, 4,5);
L298N my_motor_right(motor_pin_b_1, motor_pin_b_2);

/////////////////////////////////// Block MPU6050
// TODO mpu6050_light自带滤波？需要检查下
MPU6050 mpu(Wire);
int16_t ax, ay, az, tmp, gx, gy, gz;
/////////////////////////////////// Block MPU6050--GY25Z
// 自带滤波，直接读出角度值

/////////////////////////////////// Block Hall Encoder
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
const unsigned int motor_hall_pin_interrupt_left = 2;
const unsigned int motor_hall_pin_digit_left = 11;

const unsigned int motor_hall_pin_interrupt_right = 3;
const unsigned int motor_hall_pin_digit_right = 12;

Encoder motor_encoder_left(motor_hall_pin_interrupt_left, motor_hall_pin_digit_left);
Encoder motor_encoder_right(motor_hall_pin_interrupt_right, motor_hall_pin_digit_right);
//long oldPosition = -999;
//   avoid using pins with LEDs attached
/////////////////////////////////// Block Kalman Filter
// TrivialKalmanFilter is too simple to be used in this case.
#define DT_COVARIANCE_RK 4.7e-3 // Estimation of the noise covariances (process)
#define DT_COVARIANCE_QK 1e-5   // Estimation of the noise covariances (observation)
TrivialKalmanFilter<float> filter(DT_COVARIANCE_RK, DT_COVARIANCE_QK);
/// temp kalman filter paras
///////////////////////Kalman_Filter////////////////////////////
// Covariance of gyroscope noise
float Q_angle = 0.001;

// Covariance of gyroscope drift noise
float Q_gyro = 0.003;

// Covariance of accelerometer
float R_angle = 0.5;
char C_0 = 1;

// The filter sampling time.
float dt = 0.005;

// a function containing the Kalman gain is used to
// calculate the deviation of the optimal estimate.
float K1 = 0.05;
float K_0, K_1, t_0, t_1;
float angle_err;

// Gyroscope drift
float q_bias;

float accelz = 0;
double angle;
float angle_speed;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0},
                 {0, 1}};
float PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////
//////////////////////PID parameters///////////////////////////////
double kp = 1, ki = 0, kd = 0.62;                   //angle loop parameters
double kp_speed = 3.6, ki_speed = 0.080, kd_speed = 0;   // speed loop parameters
double setp0 = 0; //angle balance point
int PD_pwm;  //angle output
float pwm1 = 0, pwm2 = 0;
///////////////////////angle parameters//////////////////////////////
float angle_X; //Calculate the tilt angle variable about the X axis from the acceleration
float angle_Y; //Calculate the tilt angle variable about the Y axis from the acceleration
float angle0 = 1; //Actual measured angle (ideally 0 degrees)
float Gyro_x, Gyro_y, Gyro_z;  //Angular angular velocity by gyroscope calculation
///////////////////////angle parameters//////////////////////////////
////////////////////////////////PI variable parameters//////////////////////////
float speeds_filterold = 0;
float positions = 0;
int flag1;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;
double setPoint;
double output_pid;
#define OUTPUT_MIN -255
#define OUTPUT_MAX 255

//AutoPID myPID(&angle, &setPoint, &output_pid, 0, 255, &kp, &ki, &kd);
AutoPID myPID(&angle, &setPoint, &output_pid, OUTPUT_MIN, OUTPUT_MAX, kp, ki, kd);

void setup() {
    Serial.begin(115200);//Initialize the serial port
    mpu_initialize();
    delay(2);
    myPID.setTimeStep(8);
//    MsTimer2::set(500, Interrupt_Service_Routine);    //5ms ; execute the function Interrupt_Service_Routine once
//    MsTimer2::start();    //start interrupt
}

int c;

//void loop() {
//    c++;
//    if (c < 8000 * 1.2) {
//        return;
//    }
//    c = 0;
//    Serial.println("Event 0.");
//    mpu.update();
//    angle = filter.update(mpu.getAngleX());
//    if (abs(angle) > 25) {
//        my_motor_right.setSpeed(0);
//        my_motor_left.setSpeed(0);
//        return;
//    }
//    myPID.run();
//
//    Serial.print("angle :");
//    Serial.print(angle);
//    Serial.print("pid out :");
//    Serial.print(output_pid);
//    Serial.println();
//    my_motor_right.setSpeed(abs(output_pid));
//    my_motor_left.setSpeed(abs(output_pid));
//    Serial.println("speed left ");
//    Serial.print(my_motor_left.getSpeed());
//    if (output_pid > 0) {
//        my_motor_left.forward();
//        my_motor_right.forward();
//    } else {
//        my_motor_left.backward();
//        my_motor_right.backward();
//    }
//}

void loop(){
//    my_motor_left.setSpeed(100);
//    my_motor_left.forward();
//    Serial.println("+");
//    delay(3000);
//
////    my_motor_left.stop();
//    my_motor_left.setSpeed(55);
//    Serial.println("==");
//    delay(1000);
//
//    my_motor_left.setSpeed(100);
//    my_motor_left.backward();
//    Serial.println("-");
//
//    delay(3000);
}

void mpu_initialize() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(status);
    while (status != 0) {} // stop everything if could not connect to MPU6050
    delay(1000);
    Serial.print("MPU6050 done: ");
}
