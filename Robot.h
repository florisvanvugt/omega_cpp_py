#ifndef ROBOT_H
#define ROBOT_H


#include "include/CVector3d.h"
#include "include/CMaths.h"
#include "BasicRobot.h"

//#define T 1 // seconds
//#define T_MS T*1000000L
//#define MOVEMENT_DURATION_SAMPLES 4000  //T_MS/MAIN_LOOP_ITERATION_TIME
#define khi 10

const cVector3d k (1000,1000,1000);
//const double MAIN_LOOP_ITERATION_TIME = 2500. ; // microseconds
//const double MAIN_LOOP_TIME_S = MAIN_LOOP_ITERATION_TIME*.000001; //seconds
const double MAIN_LOOP_TIME_S = .0025; //seconds


//const double Floris_loop_time = MAIN_LOOP_ITERATION_TIME*.000001; //.00025;
//const double main_loop_duration = MAIN_LOOP_ITERATION_TIME;

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
        void computedPositionToSHM();
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
