/*
 * MotorController.hpp
 *
 *  Created on: Apr. 6, 2020
 *      Author: daniel
 */

#ifndef MOTORCONTROLLER_HPP_
#define MOTORCONTROLLER_HPP_

#include "RovyMotorController.h"
#include "timer.h"

#include <iostream>
#include <pthread.h>
#include <map>

using namespace std;

#define MAX_SPEED 2050 // mm/s

#define T_NOT_INITIALIZED   0
#define T_RUNNING           1
#define T_DO_STOP           2
#define T_STOPPED           3

class MotorController : public RovyMotorController {
public:
    MotorController() :
        speedControlThreadStatus_(T_NOT_INITIALIZED),
        speed_(0),
        speedLimit_(MAX_SPEED),
        orientation_(0)
    {
        speedLock_ = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")
        orientationLock_ = PTHREAD_MUTEX_INITIALIZER; // @suppress("Invalid arguments")
    }
    ~MotorController(){};

    int start(double speedLimit);
    void stop();
    void drive(double linearVelocity, double angularVelocity);

private:
    void motorsStop();
    void buildSpeedMapping();
    void speedControlThread();
    void driverVoltageOn();
    void driverVoltageOff();
    void applySpeedToWheels(double left, double right);

    int speedControlThreadStatus_;
    double speed_;
    double speedLimit_;
    double orientation_;
    map<int, int> speedMapping_;
    pthread_mutex_t speedLock_;
    pthread_mutex_t orientationLock_;
    Timer timer_;
};

RovyMotorController* RovyMotorController::Create() {
    return new MotorController();
}

#endif /* MOTORCONTROLLER_HPP_ */
