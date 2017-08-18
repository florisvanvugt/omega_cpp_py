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

   Arguments
   movt : the current time in the movement (t=0 is move start)
   movdur : the desired duration of the current movement
*/
void Robot::compute_minjerk_target (double movt,double movdur){
  shm_t *shm = getShm();

  /*
    source for minimum jerk computation:
    Flash, Tamar, and Neville Hogan. "The coordination of arm
    movements: an experimentally confirmed mathematical model." The
    journal of Neuroscience 5, no. 7 (1985): 1688-1703.
   */
  double tau = movt/movdur; // the proportion of the movement completed
  double position = 10*pow(tau,3)-15*pow(tau,4)+6*pow(tau,5); // minimum jerk position
  shm->desired_x = (shm->start_x) + (shm->target_x - shm->start_x)*position;
  shm->desired_y = (shm->start_y) + (shm->target_y - shm->start_y)*position;
  shm->desired_z = (shm->start_z) + (shm->target_z - shm->start_z)*position;

  // Compute the desired velocity
  double velocity = -30*pow(movt,4)/pow(movdur,5) + 60*pow(movt,3)/pow(movdur,4) - 30*pow(movt,2)/pow(movdur,3);
  shm->desired_vel_x = (shm->start_x - shm->target_x)*velocity;
  shm->desired_vel_y = (shm->start_y - shm->target_y)*velocity;
  shm->desired_vel_z = (shm->start_z - shm->target_z)*velocity;
  
}




/* 
   Compute forces as defined by a P controller (position-only)
   This reads the current position from shared memory
   and compares it against the target position (shm->target_{x,y,z})
*/
void Robot::compute_p_forces ()
{
  shm_t * shm = getShm();
  double stiffness = shm->stiffness;

  // compute reaction force
  fx = -stiffness*(shm->x - shm->desired_x);
  fy = -stiffness*(shm->y - shm->desired_y);
  fz = -stiffness*(shm->z - shm->desired_z);
  
}




/* 
   Compute forces as defined by a PD controller (position-and-derivative).
   This reads the current position from shared memory
   and compares it against the target position (shm->target_{x,y,z})
*/
void Robot::compute_pd_forces ()
{
  shm_t * shm = getShm();
  double stiffness = shm->stiffness;
  double damping   = shm->damping;

  // compute reaction force
  fx = -stiffness*(shm->x - shm->desired_x) - damping*(shm->vel_x - shm->desired_vel_x);
  fy = -stiffness*(shm->y - shm->desired_y) - damping*(shm->vel_y - shm->desired_vel_y);
  fz = -stiffness*(shm->z - shm->desired_z) - damping*(shm->vel_z - shm->desired_vel_z);
  
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
  setForce(); // apply
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

  double movt   = sh_memory->move_iterator*MAIN_LOOP_TIME_S; // how long we have been moving
  double movdur = sh_memory->movement_duration;      // how long this move should take

  if (movt >= movdur) {
    // If the movement is completed, switch back to the null controller...
    sh_memory->controller = 0;
    sh_memory->move_done = 1;
  }

  else {
    // Compute where I should be at the current time
    compute_minjerk_target(movt,movdur);
    compute_pd_forces();
    
    (sh_memory->move_iterator)++;
  }
}




void Robot::ControllerViscousForceField() {
  // A simple viscous force field controller.
  fx = -sh_memory->viscosity * sh_memory->vel_x;
  fy = -sh_memory->viscosity * sh_memory->vel_y;
  fz = -sh_memory->viscosity * sh_memory->vel_z;
}




void Robot::ControllerHoldAtPoint() {
  //shm_t * shm = getShm();
  //computedPos.set(shm->target_x, shm->target_y, shm->target_z);
  sh_memory->desired_x = sh_memory->target_x;
  sh_memory->desired_y = sh_memory->target_y;
  sh_memory->desired_z = sh_memory->target_z;
  sh_memory->desired_vel_x = 0;
  sh_memory->desired_vel_y = 0;
  sh_memory->desired_vel_z = 0;
  compute_pd_forces();
}





void Robot::writeLog(){
  // Not so sure if I really want to use this very much.
  // After all, we can record trajectories and have them within Python
  // instantly using start_capture(), stop_capture().
  if (sh_memory->write_log) {
    FILE* fp = fopen("log.csv","a");
    fprintf(fp, "%f %f %f %f %f %f %li %li\n",sh_memory->x, sh_memory->y, sh_memory->z, fx, fy, fz, sh_memory->controller, sh_memory->quit);
    fclose(fp);
  }
}






void Robot::mainLoop() {

  printDebug("Entering main loop");
  sh_memory->active = 1;
  
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
    
    // Read position and velocity from the sensors
    readSensors();

    // Just to be sure, set the forces to zero (actual desired
    // forces will be written in the controllers).
    fx = 0; fy = 0; fz = 0;

    // Execute the controller that is currently selected,
    // which will compute the forces fx,fy,fz that we want to apply
    switch (sh_memory->controller) {
    case 0:   /* null field (no forces) */
      ControllerNull(); break;
    case 1:   /*movement to point*/
      ControllerMoveToPoint(); break;
    case 2:   /*hold at point*/
      ControllerHoldAtPoint(); break;
    case 3:   /*viscous force*/
      ControllerViscousForceField(); break;
    }

    // Apply the forces to the robot
    setForce();

    // Update the shared memory
    updateSHM();      // update the SHM
    recordPosition(); // record the current position if we are currently recording
    writeLog();       // write the log if we are currently supposed to output a log

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
  sh_memory->active = 0;
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
  
  signal(SIGINT,sigint_handler); // install a Control+C handler
  
  // Start the debug log
  robot.openDebug();
  robot.printDebug("Starting robot");
  
  // Initialise the shared memory
  robot.printDebug("Opening shared memory");
  robot.open_sharedmem();
  // Set some default values
  shm_t * sh_memory = robot.getShm();
  sh_memory->active     = 0; // setting up
  sh_memory->controller = 0;
  sh_memory->write_log  = 0;
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



