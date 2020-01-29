#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <map>
#include <random>
#include <functional>
#include <termios.h>

#include <wiringPi.h>
#include <mcp23017.h>

#include <MPU9250_Master_I2C.h>

#include <gpio_mutex.h>
#include <pid.h>

#include "hc-sr04.h"

using namespace std;

#define MOTOR_LEFT_PWM  26 // BCM=12, physical=32
#define MOTOR_LEFT_DIR	22 // BCM=6,  physical=31
#define MOTOR_RIGHT_PWM 23 // BCM=13, physical=33
#define MOTOR_RIGHT_DIR	27 // BCM=16, physical=36

#define MAX_SPEED 2050 // mm/s

// mcp23017
#define PIN_12V_ENABLE	100

#define T_NOT_INITIALIZED	0
#define T_RUNNING 			1
#define T_DO_STOP			2
#define T_STOPPED			3

static int speedControlThreadStatus = T_NOT_INITIALIZED;
static int distanceMeasureThreadStatus = T_NOT_INITIALIZED;
static double speed = 0;
static double orientation = 0;
static map<int, int> speedMapping;
static pthread_mutex_t speedLock = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")
static pthread_mutex_t orientationLock = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")

//**************** terminal adjustments (read key without RETURN) ***************************
static struct termios old, current;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  current = old; /* make new settings same as old settings */
  current.c_lflag &= ~ICANON; /* disable buffered i/o */
  if (echo) {
      current.c_lflag |= ECHO; /* set echo mode */
  } else {
      current.c_lflag &= ~ECHO; /* set no echo mode */
  }
  tcsetattr(0, TCSANOW, &current); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character */
char getch(void)
{
	  char ch;
	  initTermios(0);
	  ch = getchar();
	  resetTermios();
	  return ch;
}
//**********************************************


void buildSpeedMapping() {
	double a = 15.0654;
	double b = 1.420508;
	double c = -0.001347318;
	double d = 7.702788e-7;
	double e = -2.205491e-10;
	double f = 2.810674e-14;

	for (int i = 1; i <= MAX_SPEED; i++) {
		// quintic regression
		double digit = (a) + (b * i) + (c * pow(i,2)) + (d * pow(i,3)) + (e * pow(i,4)) + (f * pow(i,5));
		speedMapping[i] = round(digit);
//		cout << i << ": " << round(digit) << endl;
	}

	speedMapping[0] = 0;
}

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
	if (left > MAX_SPEED) left = MAX_SPEED;
	else if (left < -MAX_SPEED) left = -MAX_SPEED;
	if (right > MAX_SPEED) right = MAX_SPEED;
	else if (right < -MAX_SPEED) right = -MAX_SPEED;

	int realLeft = speedMapping[abs(round(left))];
	int realRight = speedMapping[abs(round(right))];

//	printf("%f,%f,%d,%d\n", left, right, realLeft, realRight);

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

void *speedControlThread(void *vargp) {
    // Start the MPU9250
	MPU9250_Master_I2C imu(MPUIMU::AFS_2G, MPUIMU::GFS_1000DPS, MPU9250::MFS_16BITS, MPU9250::M_100Hz, 0x04);
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

	motorsStop();
	driverVoltageOn();

	PID pid;
	speedControlThreadStatus = T_RUNNING;
	bool driving = false;
	double angleDeltaAvg = 0;
	int notMovingCounter = 0;

	while (speedControlThreadStatus == T_RUNNING) {
		if (speed == 0) {
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
			pthread_mutex_lock(&orientationLock);
			double o = orientation;
			pthread_mutex_unlock(&orientationLock);

			bool invert;
			double angleDelta = imu.getAngleDelta(invert, o);

			pthread_mutex_lock(&speedLock);
			double s = speed;
			pthread_mutex_unlock(&speedLock);

			// if we're turning on the spot (s == 1), check if we've
			// come to a stop already and if so, stop motors
			if (s == 1) {
				double d = abs(angleDelta);
				angleDeltaAvg = (angleDeltaAvg * 10 + d) / 11;

				// if deviation is less then 0.5 degrees for 50 times
				// we assume that we've come to a stop
				if (d - angleDeltaAvg < 0.5) notMovingCounter++;
				else notMovingCounter = 0;

				if (notMovingCounter > 50) {
					pthread_mutex_lock(&speedLock);
					speed = 0;
					pthread_mutex_unlock(&speedLock);
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

			if (speedControlThreadStatus == T_RUNNING && speed != 0) {
				applySpeedToWheels(left, right);
//				printf("%f,%f,%d\n", angleDelta, angleDeltaAvg, notMovingCounter);
			}
		}
	}

	driverVoltageOff();
	speedControlThreadStatus = T_STOPPED;

    return NULL;
}

#define CHANGE_DIR_NO		0
#define CHANGE_DIR_LEFT 	1
#define CHANGE_DIR_RIGHT 	2
#define CHANGE_DIR_EITHER 	3

void *distanceMeasureThread(void *vargp) {
	HCSR04 hCSR04;
	distanceMeasureThreadStatus = T_RUNNING;
	auto gen = bind(uniform_int_distribution<>(0,1), default_random_engine());
	double oldSpeed = 0;
	bool resetFilter = false;
	int changeDir = CHANGE_DIR_NO;

	while (distanceMeasureThreadStatus == T_RUNNING) {
		double s2 = hCSR04.getFilteredDistance(2, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s5 = hCSR04.getFilteredDistance(5, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s6 = hCSR04.getFilteredDistance(6, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s1 = hCSR04.getFilteredDistance(1, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s7 = hCSR04.getFilteredDistance(7, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s4 = hCSR04.getFilteredDistance(4, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;
		double s3 = hCSR04.getFilteredDistance(3, resetFilter); delay(20); if (distanceMeasureThreadStatus != T_RUNNING) break;

		cout << fixed << setprecision(2)
			<< "s1:" << s1 << ", s2:" << s2 << ", s3:" << s3 << ", s4:" << s4 << ", s5:" << s5 << ", s6:" << s6 << ", s7:" << s7
			<< endl;

		if (speed == -1) {
			pthread_mutex_lock(&speedLock);
			speed = oldSpeed;
			pthread_mutex_unlock(&speedLock);

			resetFilter = false;
		}

		if (s1 < 30 || s2 < 30 || s3 < 30) {
			if (s6 < 30) {
				changeDir = CHANGE_DIR_LEFT;
			} else if (s7 < 30) {
				changeDir = CHANGE_DIR_RIGHT;
			} else {
				changeDir = CHANGE_DIR_EITHER;
			}
		} else if (s4 < 15) {
			changeDir = CHANGE_DIR_RIGHT;
		} else if (s5 < 15) {
			changeDir = CHANGE_DIR_LEFT;
		}

		if (changeDir != CHANGE_DIR_NO) {
			if (speed > 1) {
				pthread_mutex_lock(&speedLock);
				oldSpeed = speed;
				speed = -1;
				pthread_mutex_unlock(&speedLock);
				motorsStop();

				double newOrientation;

				switch (changeDir) {
				case CHANGE_DIR_LEFT:
					newOrientation = -90;
					break;
				case CHANGE_DIR_RIGHT:
					newOrientation = 90;
					break;
				case CHANGE_DIR_EITHER:
					if (gen()) newOrientation = -90;
					else newOrientation = 90;
					break;
				}

				pthread_mutex_lock(&orientationLock);
				orientation += newOrientation;
				if (orientation >= 360) orientation -= 360;
				pthread_mutex_unlock(&orientationLock);

				resetFilter = true;
				cout << "changed direction" << endl;

				delay(1000);
			}

			changeDir = CHANGE_DIR_NO;
		}

		delay(20);
	}

	distanceMeasureThreadStatus = T_STOPPED;

	return NULL;
}

void terminate(int s){
	distanceMeasureThreadStatus = T_DO_STOP;
	speedControlThreadStatus = T_DO_STOP;
	while (speedControlThreadStatus == T_DO_STOP || distanceMeasureThreadStatus == T_DO_STOP) {
		delay(50);
	}
	resetTermios();

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

	motorsStop();

	pthread_t threadId;
	int err = pthread_create(&threadId, NULL, &speedControlThread, NULL);
	if (err) errorAndExit("Thread creation failed");
	err = pthread_detach(threadId);
	if (err) errorAndExit("Failed to detach Thread");

	err = pthread_create(&threadId, NULL, &distanceMeasureThread, NULL);
	if (err) errorAndExit("Thread creation failed");
	err = pthread_detach(threadId);
	if (err) errorAndExit("Failed to detach Thread");

	buildSpeedMapping();

	while (speedControlThreadStatus == T_NOT_INITIALIZED) {
		delay(50);
	}

	cout << "Init done!" << endl;

	char c;
	do {
		do c = getch(); while (c == 27 || c == 91);
		switch (c) {
		case 68:
			//left
			pthread_mutex_lock(&orientationLock);
			orientation += 45;
			if (orientation >= 360) orientation -= 360;
			pthread_mutex_unlock(&orientationLock);

			// set speed to something very low but not 0
			// so that PID controller kicks in
			if (speed == 0) {
				pthread_mutex_lock(&speedLock);
				speed = 1;
				pthread_mutex_unlock(&speedLock);
			}
			break;
		case 67:
			//right
			pthread_mutex_lock(&orientationLock);
			orientation -= 45;
			if (orientation < 0) orientation += 360;
			pthread_mutex_unlock(&orientationLock);

			// set speed to something very low but not 0
			// so that PID controller kicks in
			if (speed == 0) {
				pthread_mutex_lock(&speedLock);
				speed = 1;
				pthread_mutex_unlock(&speedLock);
			}
			break;
		case 65:
			//up
			pthread_mutex_lock(&speedLock);
			speed += 100;
			pthread_mutex_unlock(&speedLock);
			break;
		case 66:
			//down
			pthread_mutex_lock(&speedLock);
			speed -= 100;
			pthread_mutex_unlock(&speedLock);
			break;
		case ' ':
			//stop
			pthread_mutex_lock(&speedLock);
			speed = 0;
			pthread_mutex_unlock(&speedLock);
			motorsStop();
			break;
		}
	} while (1);
}
