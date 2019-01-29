#include "Robot.h"
#include <time.h>
#include <sys/time.h>
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








// Apply the forces that we computed
void Robot::setForce(setforce_t* desired_f) {
  double fx = desired_f->x;
  double fy = desired_f->y;
  double fz = desired_f->z;
  if (dhdSetForce (fx, fy, fz) < 0) {
    printDebug ("error: cannot set force (%s)", dhdErrorGetLastStr());
    sh_live_memory->quit = 1; // bail out
  }
}


// Remove any forces
void Robot::zeroForce() {
  setforce_t *zerof = new setforce_t;
  setForce(zerof); // apply
}



void Robot::updateSHM(setforce_t *desired_f) {
  // Put the forces that were actually applied into the shared memory
  sh_live_memory->fx = desired_f->x;
  sh_live_memory->fy = desired_f->y;
  sh_live_memory->fz = desired_f->z;
}






void Robot::writeLog(){
  // Not so sure if I really want to use this very much.
  // After all, we can record trajectories and have them within Python
  // instantly using start_capture(), stop_capture().
  if (sh_live_memory->write_log) {
    FILE* fp = fopen("log.csv","a");
    fprintf(fp, "%f %f %f %f %f %f %li %li %li %li\n",sh_live_memory->x, sh_live_memory->y, sh_live_memory->z, sh_live_memory->fx,sh_live_memory->fy,sh_live_memory->fz,
	   sh_live_memory->active, sh_live_memory->controller, sh_instr_memory->record_flag, sh_live_memory->move_done);
    fclose(fp);
  }
}



void Robot::mainLoop() {

  printDebug("Entering main loop");
  sh_live_memory->active = 1;
  // We are about to enter the main loop. Now let's compute the time
  // we want to do the next iteration of the loop.
  double t0 = get_wall_time();
  //clock_t t0 = clock() ;
  double next_iteration_t = t0 + sh_live_memory->main_loop_time;

  while (keep_going()) {

    // Get the current time (for loop time computation)
    double t_loop_begin = get_wall_time();

    // Keep track of how often we went through this loop
    sh_live_memory->loop_iterator+=1;

    if (sh_instr_memory-> instruction_no == sh_live_memory-> instruction_no +1){
      if (sh_instr_memory ->new_trajectory != sh_live_memory ->new_trajectory) {
          new_trajectory(sh_instr_memory, sh_live_memory);
      }
      new_instruction(sh_instr_memory, sh_live_memory);

    }

    // Read position and velocity from the sensors
    readSensors();

    setforce_t *desired_force = forceFromController(sh_live_memory);

    // Apply the forces to the robot
    setForce(desired_force);

    // Update the shared memory
    updateSHM(desired_force);      // update the SHM
    recordPosition(); // record the current position if we are currently recording
    writeLog();       // write the log if we are currently supposed to output a log

    // Get the current time
    double t1 = get_wall_time();

    // Compute how long this loop took
    sh_live_memory->loop_time = t1-t_loop_begin;
    sh_live_memory->clock = t1; // update the clock
    flush_sharedmem(); // Flush shared memory

    // Wait until the scheduled next iteration time
    while (t1 < next_iteration_t) {
      t1 = get_wall_time();
    }

    // Compute the time we want to do the NEXT iteration of the loop
    next_iteration_t += sh_live_memory->main_loop_time;
    while (next_iteration_t<t1) {
      // If our current iteration really took too long, we should
      // drop a frame.
      printDebug ("Frame dropped: now (t=%d) next iteration time (t=%d) already passed",t1,next_iteration_t);
      next_iteration_t += sh_live_memory->main_loop_time;
      sh_live_memory->dropped_iterations +=1;
    }
    sh_live_memory->iteration_time = t1-t_loop_begin; // Compute how long the whole iteration took, approximately

  }
  sh_live_memory->active = 0;
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
  //shm_instr_t * sh_instr_memory = robot.getShmi();
  shm_live_t * sh_live_memory = robot.getShml();

  sh_live_memory->active     = 0; // setting up
  sh_live_memory->controller = 0;
  sh_live_memory->write_log  = 1;
  sh_live_memory->loop_time  = 0;
  sh_live_memory->instruction_no = 0;
  sh_live_memory->new_trajectory = 0;
  sh_live_memory->gap_iterator = 0;

  // Parameters for protection
  sh_live_memory->distance_max_hold = 0.03;
  sh_live_memory->distance_max = 0.3;

  sh_live_memory-> x_lim = 0.5;
  sh_live_memory-> y_lim = 0.5;
  sh_live_memory-> z_lim = 0.5;

  // Update the main loop time if so instructed in the shared memory
  sh_live_memory->main_loop_time = MAIN_LOOP_TIME_S; // the default loop time (can be changed during run-time)
  robot.printDebug("Operating loop time %f s",sh_live_memory->main_loop_time);

  // Launch the robot
  robot.openDevice();

  // Launch the comedi interface (if enabled)
  robot.openComedi();
  //robot.readComedi();
  
  // Launch the main loop
  robot.mainLoop();

  // This is after we've finished (for example when the user quits the robot)
  robot.close_sharedmem();
  robot.closeDevice();

  // Close the debug file
  robot.printDebug("Stopping robot");
  robot.closeDebug();

}





int main(int argc, char const *argv[]) {


  /*pthread_t          handle;
  pthread_create (&handle, NULL, launch, NULL);
  struct sched_param sp;
  memset (&sp, 0, sizeof(struct sched_param));
  sp.sched_priority = 10;
  pthread_setschedparam (handle, SCHED_RR, &sp);*/

 launch();


  return 0;
}
