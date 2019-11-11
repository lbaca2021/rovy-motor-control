#include <iostream>
#include <sstream>
#include <string.h>
#include <signal.h>

#include <wiringpi/wiringPi.h>
#include <wiringpi/mcp23017.h>

#include <gpio_mutex.h>

using namespace std;

#define MOTOR_LEFT_PWM  26 // BCM=12, physical=32
#define MOTOR_LEFT_DIR	22 // BCM=6,  physical=31
#define MOTOR_RIGHT_PWM 23 // BCM=13, physical=33
#define MOTOR_RIGHT_DIR	27 // BCM=16, physical=36

// mcp23017
#define PIN_12V_ENABLE	110

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

	motorsStop();
	driverVoltageOn();

	int left = 0;
	int right = 0;

	int c;
	do {
		c = getchar();
		switch (c) {
		case 'q':
			left+=100;
			if (left > 0) {
				digitalWrite(MOTOR_LEFT_DIR, 1);
			} else {
				digitalWrite(MOTOR_LEFT_DIR, 0);
			}
			pwmWrite(MOTOR_LEFT_PWM, 600);
			delay(10);
			pwmWrite(MOTOR_LEFT_PWM, abs(left));
			cout << "left: " << left << endl;
			break;
		case 'a':
			left-=100;
			if (left > 0) {
				digitalWrite(MOTOR_LEFT_DIR, 1);
			} else {
				digitalWrite(MOTOR_LEFT_DIR, 0);
			}
			pwmWrite(MOTOR_LEFT_PWM, 600);
			delay(10);
			pwmWrite(MOTOR_LEFT_PWM, abs(left));
			cout << "left: " << left << endl;
			break;
		case 'w':
			right+=100;
			if (right > 0) {
				digitalWrite(MOTOR_RIGHT_DIR, 1);
			} else {
				digitalWrite(MOTOR_RIGHT_DIR, 0);
			}
			pwmWrite(MOTOR_RIGHT_PWM, 600);
			delay(10);
			pwmWrite(MOTOR_RIGHT_PWM, abs(right));
			cout << "right: " << right << endl;
			break;
		case 's':
			right-=100;
			if (right > 0) {
				digitalWrite(MOTOR_RIGHT_DIR, 1);
			} else {
				digitalWrite(MOTOR_RIGHT_DIR, 0);
			}
			pwmWrite(MOTOR_RIGHT_PWM, 600);
			delay(10);
			pwmWrite(MOTOR_RIGHT_PWM, abs(right));
			cout << "right: " << right << endl;
			break;
		case ' ':
			motorsStop();
			left = 0;
			right = 0;
			break;
		}
	} while (c != '.');

	motorsStop();
	driverVoltageOff();

	cout << "done!" << endl;
}
