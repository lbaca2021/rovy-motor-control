/*
 * hc-sr04.h
 *
 *  Created on: Jan. 26, 2020
 *      Author: daniel
 */

#ifndef HC_SR04_H_
#define HC_SR04_H_

#include <timer.h>

class HCSR04 {
public:
	HCSR04();
	~HCSR04();
	double getFilteredDistance(int triggerPin, bool reset = false);

private:
	double getDistance(int triggerPin);
	double filter(double *previous, double current, bool reset = false);

	Timer timer_;
	double s1Prev_=100, s2Prev_=100, s3Prev_=100, s4Prev_=100, s5Prev_=100, s6Prev_=100, s7Prev_=100;
};

#endif /* HC_SR04_H_ */
