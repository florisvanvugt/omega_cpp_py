#ifndef ROBOT_H
#define ROBOT_H


#include "BasicRobot.h"
#include "Controllers.h"
#include "instruction.h"

// The duration of the main loop iteration
double MAIN_LOOP_TIME_S = .001; //seconds


class Robot : public BasicRobot
{
 public:
  Robot();
  virtual ~Robot();

  void mainLoop();
  //void compute_minjerk_target (double movt,double movdur);
  void setForce(setforce_t* desired_f);
  void zeroForce();

  void updateSHM(setforce_t* desired_f);
  void writeLog();
  void sigint_handler(int s);

 protected:

 private:
  double get_wall_time();
  //void computeVel (double t);
};

#endif // ROBOT_H
