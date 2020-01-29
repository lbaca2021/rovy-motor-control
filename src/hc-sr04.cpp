#include "hc-sr04.h"

#include <iostream>
#include <wiringPi.h>

#include <gpio_mutex.h>

// mcp23017
#define HCSR04_TRIGGER_PIN_1 109
#define HCSR04_TRIGGER_PIN_2 110
#define HCSR04_TRIGGER_PIN_3 111
#define HCSR04_TRIGGER_PIN_4 112
#define HCSR04_TRIGGER_PIN_5 113
#define HCSR04_TRIGGER_PIN_6 114
#define HCSR04_TRIGGER_PIN_7 115

#define HCSR04_ECHO_PIN 	4

using namespace std;

HCSR04::HCSR04() {
	// setup mcp23017 (16-bit IO expansion)
	i2c0Lock();
	pinMode(HCSR04_TRIGGER_PIN_1, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_2, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_3, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_4, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_5, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_6, OUTPUT);
	pinMode(HCSR04_TRIGGER_PIN_7, OUTPUT);

	digitalWrite(HCSR04_TRIGGER_PIN_1, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_2, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_3, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_4, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_5, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_6, 0);
	digitalWrite(HCSR04_TRIGGER_PIN_7, 0);
	i2c0Unlock();

	pinMode(HCSR04_ECHO_PIN, INPUT);
	pullUpDnControl(HCSR04_ECHO_PIN, PUD_DOWN);
}

HCSR04::~HCSR04(){}

double HCSR04::getFilteredDistance(int index, bool reset) {
	double d = 0;

	switch (index) {
	case 1:
		d = filter(&s1Prev_, getDistance(HCSR04_TRIGGER_PIN_1), reset); break;
	case 2:
		d = filter(&s2Prev_, getDistance(HCSR04_TRIGGER_PIN_2), reset); break;
	case 3:
		d = filter(&s3Prev_, getDistance(HCSR04_TRIGGER_PIN_3), reset); break;
	case 4:
		d = filter(&s4Prev_, getDistance(HCSR04_TRIGGER_PIN_4), reset); break;
	case 5:
		d = filter(&s5Prev_, getDistance(HCSR04_TRIGGER_PIN_5), reset); break;
	case 6:
		d = filter(&s6Prev_, getDistance(HCSR04_TRIGGER_PIN_6), reset); break;
	case 7:
		d = filter(&s7Prev_, getDistance(HCSR04_TRIGGER_PIN_7), reset); break;
	}

	return d;
}

void *trigger(void *pin) {
	int triggerPin = *(int *)pin;

	i2c0Lock();
	digitalWrite(triggerPin, 1);
	// Need to keep the PIN high for at least 10us.
	// An instruction on I2C bus takes 310us (high CPU load) to 780us (low CPU load) so no need to wait here.
	digitalWrite(triggerPin, 0);
	i2c0Unlock();

	return NULL;
}

double HCSR04::getDistance(int triggerPin) {
	// trigger happens in new thread because I2C is slow and we don't want to miss the ECHO
	pthread_t threadId;
	pthread_create(&threadId, NULL, &trigger, &triggerPin);

	while (digitalRead(HCSR04_ECHO_PIN) == 0) {
		timer_.start();
		delayMicroseconds(10);
	}

	double duration = 0;
	while (digitalRead(HCSR04_ECHO_PIN) == 1) {
		timer_.stop();
		duration = timer_.getDuration();
		delayMicroseconds(10);
	}

	// wait for trigger thread to terminate
	pthread_join(threadId, NULL);

	return duration * 17150;
}

double HCSR04::filter(double *previous, double current, bool reset) {
	double ret = current;
	if (!reset && (current - *previous) > 50) {
		ret = *previous;
		*previous += 50;
	} else {
		*previous = current;
	}
	return ret;
}
