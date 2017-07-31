#ifndef ROBOT_H
#define ROBOT_H


#include "include/CVector3d.h"
#include "include/CMaths.h"
#include "BasicRobot.h"


// Constants that are used for force control
#define khi 10
const cVector3d k (1000,1000,1000);


// The duration of the main loop iteration
double MAIN_LOOP_TIME_S = .0025; //seconds


class Robot : public BasicRobot
{
    public:
        Robot();
        virtual ~Robot();

        void computeInteractions (double t);
        void setForce();
        void ControllerNull();
        void ControllerMoveToPoint();
        void forceToSHM();
        void writeLog();
        void ControllerHoldAtPoint();

    protected:
    private:
        cVector3d computedPos;
        cVector3d computedVel;
        cVector3d computedForce;

        void computePos (double t);
        void computeVel (double t);
        void computeInteractions ();
};

#endif // ROBOT_H
