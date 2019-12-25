#include <iostream>
#include <sstream>
#include <string.h>
#include <signal.h>

#include <wiringPi.h>
#include <mcp23017.h>

#include <MPU9250_Master_I2C.h>

#include <gpio_mutex.h>

#include "pid.h"

using namespace std;

#define MOTOR_LEFT_PWM  26 // BCM=12, physical=32
#define MOTOR_LEFT_DIR	22 // BCM=6,  physical=31
#define MOTOR_RIGHT_PWM 23 // BCM=13, physical=33
#define MOTOR_RIGHT_DIR	27 // BCM=16, physical=36

// mcp23017
#define PIN_12V_ENABLE	110

static bool stopSpeedControlThread = false;
static bool orientationThreadInit = false;
static double targetSpeed = 0;
static pthread_mutex_t speedLock = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")

void driverVoltageOn() {
	i2c0Lock();
	digitalWrite(PIN_12V_ENABLE, 1);
	i2c0Unlock();
}

void driverVoltageOff() {
	i2c0Lock();
	digitalWrite(PIN_12V_ENABLE, 0);
	i2c0Unlock();
}

void motorsStop() {
	pwmWrite(MOTOR_LEFT_PWM, 0);
	digitalWrite(MOTOR_LEFT_DIR, 0);

	pwmWrite(MOTOR_RIGHT_PWM, 0);
	digitalWrite(MOTOR_RIGHT_DIR, 0);
}

void errorAndExit(const char *msg) {
	cout << msg << endl;
	exit(-1);
}

void applySpeedToWheels(double left, double right) {
	if (left > 0) {
		digitalWrite(MOTOR_LEFT_DIR, 1);
	} else  {
		digitalWrite(MOTOR_LEFT_DIR, 0);
	}
	pwmWrite(MOTOR_LEFT_PWM, abs(left));

	if (right > 0) {
		digitalWrite(MOTOR_RIGHT_DIR, 1);
	} else {
		digitalWrite(MOTOR_RIGHT_DIR, 0);
	}
	pwmWrite(MOTOR_RIGHT_PWM, abs(right));
}

void *speedControlThread(void *vargp) {
    // Start the MPU9250
	MPU9250_Master_I2C imu(MPUIMU::AFS_2G, MPUIMU::GFS_250DPS, MPU9250::MFS_16BITS, MPU9250::M_100Hz, 0x04);
	switch (imu.begin()) {
	case MPUIMU::ERROR_NONE:
		break;
	case MPUIMU::ERROR_CONNECT:
		errorAndExit("Connection error"); break;
	case MPUIMU::ERROR_IMU_ID:
		errorAndExit("Bad IMU device ID"); break;
	case MPUIMU::ERROR_MAG_ID:
		errorAndExit("Bad magnetometer device ID"); break;
	case MPUIMU::ERROR_SELFTEST:
		errorAndExit("Failed self-test"); break;
	}
	imu.setMyMagCalibration();

	PID pid;
	orientationThreadInit = true;
	bool driving = false;
	double speed = 0;

	while (!stopSpeedControlThread) {
		if (targetSpeed == 0) {
			driving = false;
			imu.calibrateGyro();
		} else {
			// first time driving: setup PID controller
			if (!driving) {
				pid = PID(20, 1024, -1024, 100, 0.7, 0.002);
				speed = 0;
			}
			driving = true;
		}

		if (driving) {
			bool invert;
			double angleDelta = imu.getAngleDelta(invert);
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

			pthread_mutex_lock(&speedLock);
			if (speed < targetSpeed) {
				speed += min(50.0, targetSpeed - speed);
			} else if (speed > targetSpeed) {
				speed -= min(50.0, speed - targetSpeed);
			}
			pthread_mutex_unlock(&speedLock);

			left += speed;
			right += speed;

			if (!stopSpeedControlThread && targetSpeed != 0) {
				applySpeedToWheels(left, right);
//				printf("%f,%f,%f,%f\n", angleDelta, diff, left, right);
			}
		}
	}

    return NULL;
}

void terminate(int s){
	driverVoltageOff();
	i2c0Unlock();
	exit(0);
}

int main(int argc, char *argv[]) {
	signal(SIGINT, terminate);
	signal(SIGTERM, terminate);

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

	pthread_t threadId;
	int err = pthread_create(&threadId, NULL, &speedControlThread, NULL);
	if (err) errorAndExit("Thread creation failed");
	err = pthread_detach(threadId);
	if (err) errorAndExit("Failed to detach Thread");

	while (!orientationThreadInit) {
		delay(100);
	}

	cout << "Init done!" << endl;

//	// testing loop
//	while (1) {
//		delay(1000 * 7);
//		motorsStop();
//		driverVoltageOn();
//		speed = 500;
//		delay(1000 * 200);
//		motorsStop();
//		driverVoltageOff();
//		speed = 0;
//	}

	motorsStop();
	driverVoltageOn();

	int c;
	do {
		c = getchar();
		switch (c) {
		case 'w':
	    	pthread_mutex_lock(&speedLock);
	    	targetSpeed += 100;
	    	pthread_mutex_unlock(&speedLock);
			break;
		case 's':
	    	pthread_mutex_lock(&speedLock);
	    	targetSpeed -= 100;
	    	pthread_mutex_unlock(&speedLock);
			break;
		case ' ':
			motorsStop();
	    	pthread_mutex_lock(&speedLock);
	    	targetSpeed = 0;
	    	pthread_mutex_unlock(&speedLock);
			break;
		}
	} while (c != '.');

	stopSpeedControlThread = true;
	motorsStop();
	driverVoltageOff();

	cout << "done!" << endl;
}
