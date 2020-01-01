#include <iostream>
#include <sstream>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <map>
#include <termios.h>

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

#define MAX_SPEED 2050 // mm/s

// mcp23017
#define PIN_12V_ENABLE	110

static bool stopSpeedControlThread = false;
static bool orientationThreadInit = false;
static double speed = 0;
static map<int, int> speedMapping;
static pthread_mutex_t speedLock = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")

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

	while (!stopSpeedControlThread) {
		if (speed == 0) {
			driving = false;
			imu.calibrateGyro();
		} else {
			// first time driving: setup PID controller
			if (!driving) {
				pid = PID(20, MAX_SPEED, -MAX_SPEED, 100, 10, 0.002);
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
			left += speed;
			right += speed;
			pthread_mutex_unlock(&speedLock);

			if (!stopSpeedControlThread && speed != 0) {
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

	pthread_t threadId;
	int err = pthread_create(&threadId, NULL, &speedControlThread, NULL);
	if (err) errorAndExit("Thread creation failed");
	err = pthread_detach(threadId);
	if (err) errorAndExit("Failed to detach Thread");

	buildSpeedMapping();

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

	char c;
	do {
		do c = getch(); while (c == 27 || c == 91);
		switch (c) {
		case 68:
			//left
			break;
		case 67:
			//right
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
			motorsStop();
			pthread_mutex_lock(&speedLock);
			speed = 0;
			pthread_mutex_unlock(&speedLock);
			break;
		}
	} while (1);
}
