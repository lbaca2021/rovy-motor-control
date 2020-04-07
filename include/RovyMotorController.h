/*
 * Controller.h
 *
 *  Created on: Apr. 6, 2020
 *      Author: daniel
 */

#ifndef ROVYMOTORCONTROLLER_H_
#define ROVYMOTORCONTROLLER_H_

#define MANUAL_LEFT 0
#define MANUAL_RIGHT 1
#define MANUAL_UP 2
#define MANUAL_DOWN 3
#define MANUAL_STOP 4

class RovyMotorController {
public:
    virtual ~RovyMotorController(){}

    virtual int start(double speedLimit) = 0;
    virtual void stop() = 0;
    virtual void manualDrive(int command) = 0;

    static RovyMotorController* Create();
protected:
    RovyMotorController(){}
};

#endif /* ROVYMOTORCONTROLLER_H_ */
