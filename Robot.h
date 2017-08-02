#ifndef ROBOT_H
#define ROBOT_H


#include "BasicRobot.h"

// The duration of the main loop iteration
double MAIN_LOOP_TIME_S = .005; //seconds


class Robot : public BasicRobot
{
 public:
  Robot();
  virtual ~Robot();
  
  void mainLoop();
  void compute_minjerk_target (double movt,double movdur);
  void setForce();
  void zeroForce();
  
  void ControllerNull();
  void ControllerMoveToPoint();
  void ControllerHoldAtPoint();
  
  void updateSHM();
  void writeLog();
  void sigint_handler(int s);
  
 protected:
  double fx;
  double fy;
  double fz;
  
 private:
  void compute_pd_forces ();
  void compute_p_forces ();
  double get_wall_time();
  //void computeVel (double t);
};

#endif // ROBOT_H
