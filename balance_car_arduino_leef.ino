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

/////////////////////////////////// Block Motor
const unsigned int motor_pin_a_1 = 4;
const unsigned int motor_pin_a_2 = 5;
const unsigned int motor_pin_b_1 = 6;
const unsigned int motor_pin_b_2 = 7;
L298N my_motor_left(motor_pin_a_1, motor_pin_a_2);
L298N my_motor_right(motor_pin_b_1, motor_pin_b_2);

/////////////////////////////////// Block MPU6050

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
float angle;
float angle_speed;

float Pdot[4] = {0, 0, 0, 0};
float P[2][2] = {{1, 0},
                 {0, 1}};
float PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////


void setup() {
    Serial.begin(115200);//Initialize the serial port
    mpu_initialize();
    delay(2);
    MsTimer2::set(5, Interrupt_Service_Routine);    //5ms ; execute the function Interrupt_Service_Routine once
    MsTimer2::start();    //start interrupt
}

void mpu_initialize() {
    Wire.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {} // stop everything if could not connect to MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(); // gyro and accelero
    Serial.println("MPU6050 Done!\n");
}

void Interrupt_Service_Routine() {
    tmp = mpu.getTemp();
    ax = mpu.getAngleX();
    ay = mpu.getAngleY();
    az = mpu.getAngleZ();
    gx = mpu.getGyroX();
    gy = mpu.getGyroY();
    gz = mpu.getGyroZ();

//    angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
    /////////////////////////////angle calculate///////////////////////
//    void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
    {
        // Radial rotation angle calculation formula; negative sign is direction processing
        Angle = -atan2(ay, az) * (180 / PI);

        // The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
        Gyro_x = -gx / 131;

        // KalmanFilter
        Kalman_Filter(Angle, Gyro_x);
    }
    //get angle and Kalman filtering
    PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD angle loop control
    // Do PWM calculate
    pwm2 = -PD_pwm - PI_pwm;           //assign the final value of PWM to motor
    pwm1 = -PD_pwm - PI_pwm;
    if (angle > 25 || angle < -25) {
        pwm1 = pwm2 = 0;
    }
    // Determine the motor’s steering and speed by the positive and negative of PWM
    my_motor_left.setSpeed(abs(pwm1));
    my_motor_right.setSpeed(abs(pwm2));
    pwm1 > 0 ? my_motor_left.forward() : my_motor_left.backward();
    pwm2 > 0 ? my_motor_right.forward() : my_motor_right.backward();

    cc++;
    if (cc >= 8)     //5*8=40，40ms entering once speed PI algorithm
    {
//        speedpiout();
        float speeds = (motor_encoder_left.read() + motor_encoder_right.read()) * 1.0;      //Vehicle speed  pulse value
//        pulseright = pulseleft = 0;      //Clear
        motor_encoder_left.write(0);
        motor_encoder_right.write(0);
        speeds_filterold *= 0.7;         //first-order complementary filtering
        speeds_filter = speeds_filterold + speeds * 0.3;
        speeds_filterold = speeds_filter;
        positions += speeds_filter;
        positions = constrain(positions, -3550, 3550);    //Anti-integral saturation
        PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);      //speed loop control PI

        cc = 0;  //Clear
    }
}


void Kalman_Filter(double angle_m, double gyro_m) {
    angle += (gyro_m - q_bias) * dt;          //Prior estimate
    angle_err = angle_m - angle;

    Pdot[0] = Q_angle - P[0][1] - P[1][0];    //Differential of azimuth error covariance
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;    //A priori estimation error covariance differential integral
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    //Intermediate variable of matrix multiplication
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    //Denominator
    E = R_angle + C_0 * PCt_0;
    //gain value
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;  //Intermediate variable of matrix multiplication
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;    //Posterior estimation error covariance
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    q_bias += K_1 * angle_err;    //Posterior estimate
    angle_speed = gyro_m - q_bias;   //The differential of the output value gives the optimal angular velocity
    angle += K_0 * angle_err; ////Posterior estimation to get the optimal angle
}
