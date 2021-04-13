#include "MotorController.hpp"
#include "RovyMotorController.h"

#include <math.h>
#include <thread>

#include <wiringPi.h>
#include <mcp23017.h>

#include <gpio_mutex.h>
#include <pid.h>
#include <mutex_t265.h>

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

int MotorController::start(double speedLimit, double rotationLimit) {
    speedLimit_ = speedLimit < MAX_SPEED ? speedLimit : MAX_SPEED;
    rotationLimit_ = rotationLimit < MAX_ROTATION ? rotationLimit : MAX_ROTATION;

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

    pthread_mutex_lock(&velocityLock_);
    linearVel_ = linearVelocity * 1000;
    angularVel_ = angularVelocity;
    pthread_mutex_unlock(&velocityLock_);

    if (linearVelocity == 0 && angularVelocity == 0) {
        motorsStop();
    }
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
        //cout << i << ": " << round(digit) << endl;
    }

    speedMapping_[0] = 0;
}

void MotorController::speedControlThread() {
    motorsStop();
    driverVoltageOn();

    PID linearVelPid, angularVelPid;
    speedControlThreadStatus_ = T_RUNNING;
    bool driving = false;
    uint oldIndex = 0;
    int oldIndexCount = 0;

    while (speedControlThreadStatus_ == T_RUNNING) {
        delay(5);

        pthread_mutex_lock(&velocityLock_);
        if (linearVel_ > speedLimit_) linearVel_ = speedLimit_;
        double targetSpeed = linearVel_;
        if (angularVel_ > rotationLimit_) angularVel_ = rotationLimit_;
        double targetTheta = angularVel_;
        pthread_mutex_unlock(&velocityLock_);

        if (targetSpeed == 0) {
            driving = false;
        } else {
            // first time driving: setup PID controller
            if (!driving) {
                linearVelPid = PID(5, speedLimit_, -speedLimit_, 0.001, 0.04, 0.005);
                angularVelPid = PID(5, rotationLimit_, -rotationLimit_, 0.01, 0.1, 0.005);
            }
            driving = true;
        }

        if (driving) {
            // get current linear velocity from odometry (stored in shared memory)
            uint index;
            float linearVelocity = 0, angularVelocity = 0;
            t265_get_data(&index, &linearVelocity, &angularVelocity);

            // index shouldn't be the same, if so camera might have crashed and we should stop
            if (index == oldIndex) {
                if (oldIndexCount++ > 10) {
                    motorsStop();
                    pthread_mutex_lock(&velocityLock_);
                    linearVel_ = 0;
                    angularVel_ = 0;
                    pthread_mutex_unlock(&velocityLock_);
                    continue;
                }
            } else {
                oldIndex = index;
                oldIndexCount = 0;
            }

            double outSpeed = targetSpeed;
            if (targetSpeed != 1) {
                outSpeed = linearVelPid.calculate(targetSpeed, linearVelocity * 1000);
                //cout << targetSpeed << "," << linearVelocity << "," << outSpeed << endl;
            }

            double outTheta = angularVelPid.calculate(targetTheta, angularVelocity);
            //cout << targetTheta << "," << angularVelocity << "," << outTheta << endl;

            double left = -outTheta*500 + outSpeed;
            double right = outTheta*500 + outSpeed;

            if (speedControlThreadStatus_ == T_RUNNING && targetSpeed != 0) {
                applySpeedToWheels(left, right);
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
