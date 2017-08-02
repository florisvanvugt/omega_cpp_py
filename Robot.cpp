#include "Robot.h"
#include <time.h>
#include <sys/time.h>
#include <math.h>       /* pow */
#include <memory.h>
#include <pthread.h>
#include <ostream>
#include <signal.h>

Robot robot;


Robot::Robot()
{
    BasicRobot();
}

Robot::~Robot()
{
}


/* 
   Compute the target position corresponding to the current time
   for a minimum-jerk trajectory.
 */
void Robot::compute_minjerk_target (double t){
  //cVector3d depth;
  //cVector3d result;
  shm_t *shm = getShm();
  //    double T = sh_memory->movement_duration;
  double position_prop = 10*pow(t,3)-15*pow(t,4)+6*pow(t,5); // minimum jerk position
  shm->desired_x = (shm->start_x) + (shm->target_x - shm->start_x)*position_prop;
  shm->desired_y = (shm->start_y) + (shm->target_y - shm->start_y)*position_prop;
  shm->desired_z = (shm->start_z) + (shm->target_z - shm->start_z)*position_prop;
}




/* Compute forces as defined by a PD controller.
   This reads the current position from shared memory
   and compares it against the target position (shm->target_{x,y,z})
*/
void Robot::compute_pd_forces ()
{
  shm_t * shm = getShm();
  double stiffness = shm->stiffness;

  // compute reaction force
  fx = -stiffness*(shm->x - shm->desired_x);
  fy = -stiffness*(shm->y - shm->desired_y);
  fz = -stiffness*(shm->z - shm->desired_z);
  
  //velocity  = cMul(-khi, velocity);
  //localForce.addr(localVel, localForce);
}




// Apply the forces that we computed
void Robot::setForce() {
  if (dhdSetForce (fx, fy, fz) < 0) {
    printDebug ("error: cannot set force (%s)", dhdErrorGetLastStr());
    sh_memory->quit = 1; // bail out
  }
}


// Remove any forces
void Robot::zeroForce() {
  fx=0;fy=0;fz=0;
  setForce();
}



void Robot::updateSHM() {
  sh_memory->fx = fx;
  sh_memory->fy = fy;
  sh_memory->fz = fz;
}


void Robot::ControllerNull() {
  fx = 0;
  fy = 0;
  fz = 0;
}


void Robot::ControllerMoveToPoint() {
  shm_t * sh_memory = getShm();

  //double T = sh_memory->movement_duration;
  (sh_memory->mds)=(sh_memory->movement_duration)/MAIN_LOOP_TIME_S;

  if ((sh_memory->movet) >= sh_memory->mds) {
    // If the movement is completed, switch back to the null controller...
    sh_memory->controller = 0;
    sh_memory->move_done = 1;
  }

  else {
    // Compute where I should be at the current time

    // So what we want, is a variable t that tells us where
    // we are in the movement, from 0 = beginning of movement
    // to 1 = end of movement.
    // However, move_t tells us which sample we are from the total
    // movement duration (for example 500 our of 1000)
    double movt = ((double)sh_memory->movet)/(sh_memory->mds);

    compute_minjerk_target(movt);
    compute_pd_forces();
    
    (sh_memory->movet)++;
  }
}



void Robot::ControllerHoldAtPoint() {
  //shm_t * shm = getShm();
  //computedPos.set(shm->target_x, shm->target_y, shm->target_z);
  sh_memory->desired_x = sh_memory->target_x;
  sh_memory->desired_y = sh_memory->target_y;
  sh_memory->desired_z = sh_memory->target_z;
  compute_pd_forces();
}



void Robot::writeLog(){
  FILE* fp = fopen("log.csv","a");
  fprintf(fp, "%f %f %f %f %f %f %li %li\n",sh_memory->x, sh_memory->y, sh_memory->z, fx, fy, fz, sh_memory->controller, sh_memory->quit);
  fclose(fp);
}






void Robot::mainLoop() {

  printDebug("Entering main loop");
  
  // We are about to enter the main loop. Now let's compute the time
  // we want to do the next iteration of the loop.
  double t0 = get_wall_time();
  //clock_t t0 = clock() ;
  double next_iteration_t = t0 + MAIN_LOOP_TIME_S;
    
  while (keep_going()) {

    // Update the main loop time if so instructed in the shared memory
    MAIN_LOOP_TIME_S = (sh_memory->main_loop_time);
    
    // Get the current time (for loop time computation)
    double t_loop_begin = get_wall_time();

    // Keep track of how often we went through this loop
    sh_memory->loop_iterator+=1;
    
    // Update SHM position et velocity
    readSensors();

    // Just to be sure, set the forces to zero (actual desired
    // forces will be written in the controllers).
    fx = 0; fy = 0; fz = 0;

    // Execute the controller that is currently selected,
    // which will compute the forces fx,fy,fz that we want to apply
    switch (sh_memory->controller) {
    case 0/* value */:
      ControllerNull(); break;
    case 1/*movement to point*/:
      ControllerMoveToPoint(); break;
    case 2/*hold at point*/:
      ControllerHoldAtPoint(); break;
    }

    // Apply the forces to the robot
    setForce();

    // Update the shared memory
    updateSHM();
    recordPosition();
    writeLog();

    // Get the current time
    double t1 = get_wall_time();

    // Compute how long this loop took
    sh_memory->loop_time = t1-t_loop_begin;
    sh_memory->clock = t1; // update the clock
    flush_sharedmem(); // Flush shared memory
    
    // Wait until the scheduled next iteration time
    while (t1 < next_iteration_t) {
      t1 = get_wall_time();
    }

    // Compute the time we want to do the NEXT iteration of the loop
    next_iteration_t += MAIN_LOOP_TIME_S;
    while (next_iteration_t<t1) {
      // If our current iteration really took too long, we should
      // drop a frame.
      printDebug ("Frame dropped: now (t=%d) next iteration time (t=%d) already passed",t1,next_iteration_t);
      next_iteration_t += MAIN_LOOP_TIME_S;
      sh_memory->dropped_iterations +=1;
    }
    
    sh_memory->iteration_time = t1-t_loop_begin; // Compute how long the whole iteration took, approximately

  }
  printDebug ("Ending main loop");

}




// Bailing out in case of an interruption signal
void sigint_handler(int s) {
  robot.printDebug("Caught signal %d",s);
  printf("Caught signal %d\n",s);
  //fx=0;fy=0;fz=0;
  robot.zeroForce();
  robot.close_sharedmem();
  robot.closeDevice();
  robot.printDebug("Stopping robot");
  robot.closeDebug();
  exit(1);
}




double Robot::get_wall_time(){
  /* Return the current system time ('wall time') in seconds */
  // courtesy https://stackoverflow.com/questions/17432502/how-can-i-measure-cpu-time-and-wall-clock-time-on-both-linux-windows
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    //  Handle error
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}






void launch() {
  
  //Robot robot;

  signal(SIGINT,sigint_handler); // set the Control+C handler
  
  // Start the debug log
  robot.openDebug();
  robot.printDebug("Starting robot");
  
  // Initialise the shared memory
  robot.printDebug("Opening shared memory");
  robot.open_sharedmem();
  shm_t * sh_memory = robot.getShm();
  sh_memory->controller = 0;
  sh_memory->loop_time  = 0;
  // Update the main loop time if so instructed in the shared memory
  sh_memory->main_loop_time = MAIN_LOOP_TIME_S;
  robot.printDebug("Operating loop time %f s",MAIN_LOOP_TIME_S);

  // Launch the robot
  robot.openDevice();

  // Launch the main loop
  robot.mainLoop();

  robot.close_sharedmem();
  robot.closeDevice();

  // Close the debug file
  robot.printDebug("Stopping robot");
  robot.closeDebug();
  
}






int main(int argc, char const *argv[]) {

  /*
  pthread_t          handle;
  pthread_create (&handle, NULL, launch, NULL);
  struct sched_param sp;
  memset (&sp, 0, sizeof(struct sched_param));
  sp.sched_priority = 10;
  pthread_setschedparam (handle, SCHED_RR, &sp);
  */
  launch();
  
  return 0;
}



