#include "MotorController.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <random>
#include <thread>

#include <wiringPi.h>
#include <mcp23017.h>

#include <MPU9250_Master_I2C.h>

#include <gpio_mutex.h>
#include <pid.h>

#define MOTOR_LEFT_PWM  26 // BCM=12, physical=32
#define MOTOR_LEFT_DIR	22 // BCM=6,  physical=31
#define MOTOR_RIGHT_PWM 23 // BCM=13, physical=33
#define MOTOR_RIGHT_DIR	27 // BCM=16, physical=36

// mcp23017
#define PIN_12V_ENABLE	100

void MotorController::stop(){
	speedControlThreadStatus_ = T_DO_STOP;
	while (speedControlThreadStatus_ == T_DO_STOP) {
		delay(50);
	}
}

int MotorController::start(double speedLimit) {
    speedLimit_ = speedLimit < MAX_SPEED ? speedLimit : MAX_SPEED;

	wiringPiSetup();

	i2c0Lock();
	mcp23017Setup(100, 0x20);
	pinMode(PIN_12V_ENABLE, OUTPUT);
	digitalWrite(PIN_12V_ENABLE, 0);
	i2c0Unlock();

	pinMode(MOTOR_LEFT_PWM, PWM_OUTPUT);
	pinMode(MOTOR_LEFT_DIR, OUTPUT);
	pinMode(MOTOR_RIGHT_PWM, PWM_OUTPUT);
	pinMode(MOTOR_RIGHT_DIR, OUTPUT);

	motorsStop();

	thread th(&MotorController::speedControlThread, this);
	th.detach();

	buildSpeedMapping();

	int count = 0;
	while (speedControlThreadStatus_ == T_NOT_INITIALIZED && count++ < 200) {
		delay(50);
	}

	if (count >= 200) return -1; // error initializing thread

	return 0;
}

void MotorController::drive(double linearVelocity, double angularVelocity) {
    if (linearVelocity == 0 && angularVelocity != 0) {
        // set speed to something very low but not 0
        // so that PID controller kicks in
        linearVelocity = 0.001;
    }

    pthread_mutex_lock(&speedLock_);
    speed_ = linearVelocity * 1000;
    pthread_mutex_unlock(&speedLock_);

    pthread_mutex_lock(&orientationLock_);
    orientation_ += angularVelocity * timer_.next();
    if (orientation_ >= 360) orientation_ -= 360;
    else if (orientation_ < 0) orientation_ += 360;
    pthread_mutex_unlock(&orientationLock_);

    if (linearVelocity == 0 && angularVelocity == 0) {
        motorsStop();
    }

    printf("%f/%f\n", speed_, orientation_);
}

void MotorController::motorsStop() {
    pwmWrite(MOTOR_LEFT_PWM, 0);
    digitalWrite(MOTOR_LEFT_DIR, 0);

    pwmWrite(MOTOR_RIGHT_PWM, 0);
    digitalWrite(MOTOR_RIGHT_DIR, 0);
}

void MotorController::buildSpeedMapping() {
    double a = 15.0654;
    double b = 1.420508;
    double c = -0.001347318;
    double d = 7.702788e-7;
    double e = -2.205491e-10;
    double f = 2.810674e-14;

    for (int i = 1; i <= MAX_SPEED; i++) {
        // quintic regression
        double digit = (a) + (b * i) + (c * pow(i,2)) + (d * pow(i,3)) + (e * pow(i,4)) + (f * pow(i,5));
        speedMapping_[i] = round(digit);
//        cout << i << ": " << round(digit) << endl;
    }

    speedMapping_[0] = 0;
}

void MotorController::speedControlThread() {
    // Start the MPU9250
    MPU9250_Master_I2C imu(MPUIMU::AFS_2G, MPUIMU::GFS_1000DPS, MPU9250::MFS_16BITS, MPU9250::M_100Hz, 0x04);
    switch (imu.begin()) {
    case MPUIMU::ERROR_NONE:
        break;
    case MPUIMU::ERROR_CONNECT:
        return;
    case MPUIMU::ERROR_IMU_ID:
        return;
    case MPUIMU::ERROR_MAG_ID:
        return;
    case MPUIMU::ERROR_SELFTEST:
        return;
    }
    imu.setMyMagCalibration();

    motorsStop();
    driverVoltageOn();

    PID pid;
    speedControlThreadStatus_ = T_RUNNING;
    bool driving = false;
    double angleDeltaAvg = 0;
    int notMovingCounter = 0;

    while (speedControlThreadStatus_ == T_RUNNING) {
        if (speed_ == 0) {
            driving = false;
            imu.calibrateGyro();
        } else {
            // first time driving: setup PID controller
            if (!driving) {
                pid = PID(20, MAX_SPEED, -MAX_SPEED, 70, 1, 0.002);
            }
            driving = true;
        }

        if (driving) {
            pthread_mutex_lock(&orientationLock_);
            double o = orientation_;
            pthread_mutex_unlock(&orientationLock_);

            bool invert;
            double angleDelta = imu.getAngleDelta(invert, o);

            pthread_mutex_lock(&speedLock_);
            if (speed_ > speedLimit_) speed_ = speedLimit_;
            double s = speed_;
            pthread_mutex_unlock(&speedLock_);

            // if we're turning on the spot (s == 1), check if we've
            // come to a stop already and if so, stop motors
            if (s == 1) {
                double d = abs(angleDelta);
                angleDeltaAvg = (angleDeltaAvg * 10 + d) / 11;

                // if deviation is less then 0.5 degrees for 100 times
                // we assume that we've come to a stop
                if (d - angleDeltaAvg < 0.5) notMovingCounter++;
                else notMovingCounter = 0;

                if (notMovingCounter > 100) {
                    pthread_mutex_lock(&speedLock_);
                    speed_ = 0;
                    pthread_mutex_unlock(&speedLock_);
                    motorsStop();
                    notMovingCounter = 0;
                    angleDeltaAvg = 0;
                }
            }

            if (invert) pid.invert();
            double diff = pid.calculate(0, angleDelta);

            double half = abs(diff) * 0.5;
            double left = 0, right = 0;

            if (diff > 0) {
                left = half * -1;
                right = half * 1;
            } else {
                left = half * 1;
                right = half * -1;
            }

            left += s;
            right += s;

            if (speedControlThreadStatus_ == T_RUNNING && speed_ != 0) {
                applySpeedToWheels(left, right);
//                printf("%f,%f,%d\n", angleDelta, angleDeltaAvg, notMovingCounter);
            }
        }
    }

    driverVoltageOff();
    speedControlThreadStatus_ = T_STOPPED;
}

void MotorController::driverVoltageOn() {
    i2c0Lock();
    digitalWrite(PIN_12V_ENABLE, 1);
    i2c0Unlock();
}

void MotorController::driverVoltageOff() {
    i2c0Lock();
    digitalWrite(PIN_12V_ENABLE, 0);
    i2c0Unlock();
}

void MotorController::applySpeedToWheels(double left, double right) {
    if (left > MAX_SPEED) left = MAX_SPEED;
    else if (left < -MAX_SPEED) left = -MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;
    else if (right < -MAX_SPEED) right = -MAX_SPEED;

    int realLeft = speedMapping_[abs(round(left))];
    int realRight = speedMapping_[abs(round(right))];

//    printf("%f,%f,%d,%d\n", left, right, realLeft, realRight);

    if (left > 0) {
        digitalWrite(MOTOR_LEFT_DIR, 1);
    } else  {
        digitalWrite(MOTOR_LEFT_DIR, 0);
    }
    pwmWrite(MOTOR_LEFT_PWM, realLeft);

    if (right > 0) {
        digitalWrite(MOTOR_RIGHT_DIR, 1);
    } else {
        digitalWrite(MOTOR_RIGHT_DIR, 0);
    }
    pwmWrite(MOTOR_RIGHT_PWM, realRight);
}
