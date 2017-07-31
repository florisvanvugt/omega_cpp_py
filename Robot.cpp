#include "Robot.h"

Robot::Robot()
{
    BasicRobot();
    computedPos.zero();
    computedVel.zero();
    computedForce.zero();
}

Robot::~Robot()
{
    //dtor
}


//Calcul de x(t)
void Robot::computePos (double t){
    cVector3d depth;
    cVector3d result;
    shm_t *sh_memory = getShm();
    double T = sh_memory->movement_duration;
    cVector3d targetPos;
    cVector3d startPos;
    targetPos.set(sh_memory->target_x, sh_memory->target_y, sh_memory->target_z);
    startPos.set(sh_memory->start_x, sh_memory->start_y, sh_memory->start_z);
    targetPos.subr(startPos,depth);
    double ratesTime = 10*pow(t,3)-15*pow(t,4)+6*pow(t,5);
    result = cMul(ratesTime,depth);
    computedPos = cAdd(result,startPos);
}

//Calcul de v(t)
void Robot::computeVel (double t) {
    cVector3d depth;
    cVector3d result;
    shm_t *sh_memory = getShm();
    double T = sh_memory->movement_duration;
    cVector3d targetPos;
    cVector3d startPos;
    targetPos.set(sh_memory->target_x, sh_memory->target_y, sh_memory->target_z);
    startPos.set(sh_memory->start_x, sh_memory->start_y, sh_memory->start_z);
    targetPos.subr(startPos,depth);
    double ratesTime = (30/(T))*pow(t,2)-(60/(T))*pow(t,3)+(30/(T))*pow(t,4);
    computedVel = cMul(-ratesTime,depth);
}

void Robot::computeInteractions (double t) {

    cVector3d localPos (0, 0, 0);
    cVector3d localVel (0, 0, 0);

    computePos(t);
    computeVel(t);

    computeInteractions ();
}

//Compute of torque and force to apply
void Robot::computeInteractions ()
{
    cVector3d localForce  (0, 0, 0);
    cVector3d localVel (0, 0, 0);
    cVector3d depth;
    cVector3d actualPos;
    cVector3d velocity;
    shm_t * shm = getShm();
    actualPos.set(shm->x,shm->y,shm->z);
    velocity.set(shm->vel_x,shm->vel_y,shm->vel_z);

    // compute position of device in locale coordinates of cube
    actualPos.subr(computedPos,depth);

    // compute reaction force
    localForce.x = -k.x*depth.x;
    localForce.y = -k.y*depth.y;
    localForce.z = -k.z*depth.z;

    //velocity  = cMul(-khi, velocity);
    //localForce.addr(localVel, localForce);

    // convert results in global coordinates
    computedForce.x = localForce.x;
    computedForce.y = localForce.y;
    computedForce.z = localForce.z;
}

void Robot::setForce() {
    shm_t * sh_memory = getShm();
    if (dhdSetForce (computedForce.x, computedForce.y, computedForce.z) < 0) {
        printf ("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        sh_memory->quit = 0;
    }
}

void Robot::forceToSHM() {
  shm_t * shm = getShm();
  shm->fx = computedForce.x;
  shm->fy = computedForce.y;
  shm->fz = computedForce.z;
}

void Robot::computedPositionToSHM() {
  shm_t * shm = getShm();
  shm->minjerk_x = computedPos.x;
  shm->minjerk_y = computedPos.y;
  shm->minjerk_z = computedPos.z;
}

void Robot::ControllerNull() {
  computedForce.zero();
}


void Robot::ControllerMoveToPoint() {
  shm_t * sh_memory = getShm();

  //(sh_memory->mds)=8000.;
  double T = sh_memory->movement_duration;
  //(sh_memory->mds)=(sh_memory->movement_duration)/.00025;
  (sh_memory->mds)=(sh_memory->movement_duration)/MAIN_LOOP_TIME_S;
  //sh_memory->main_loop_time = MAIN_LOOP_TIME_S;

  //double MOVEMENT_DURATION_SAMPLES = T/sh_memory->main_loop_time;
  //(sh_memory->mds)=MOVEMENT_DURATION_SAMPLES;

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

    (sh_memory->t)=movt;
    computeInteractions (movt);
    (sh_memory->movet)++;
  }
}

void Robot::ControllerHoldAtPoint() {
  shm_t * shm = getShm();

  computedPos.set(shm->target_x, shm->target_y, shm->target_z);
  computeInteractions();
}

void Robot::writeLog(){
  FILE* fp = fopen("log.csv","a");
  shm_t* shm = getShm();
  fprintf(fp, "%f %f %f %f %f %f %li %li\n",shm->x, shm->y, shm->z, computedForce.x, computedForce.y, computedForce.z, shm->controller, shm->quit);
  //fprintf(fp, "%lu\n", sizeof(int));
  //fprintf(fp, "%lu\n", sizeof(double));
  //fprintf(fp, "%lu\n", sizeof(shm_t));
  fclose(fp);
}





int main(int argc, char const *argv[]) {

  Robot robot;
  robot.open_sharedmem();
  robot.openDevice();
  //robot.mainLoop();

  //printf("Movement duration will be %f",MOVEMENT_DURATION_SAMPLES);

  shm_t * sh_memory = robot.getShm();
  sh_memory->controller = 0;
  sh_memory->loop_time = 0;

  //void * pp =;
  //printf("x %p y %p angle %p\n", &(sh_memory->x), &(sh_memory->y), &(sh_memory->angle));
  //void *pA = (unsigned int) &(sh_memory->otherquit);
  //void *pB = &(sh_memory->x);

  //printf("y-x %li\n", pA - pB );
  //clock_t t = clock();

  // Set the time of the next loop iteration
  //clock_t t0 = clock() ;
  
  while (robot.keep_going()) {

    // Get the current time (for loop time computation)
    clock_t t0 = clock();

    // Keep track of how often we went through this loop
    sh_memory->loop_iterator+=1;
    
    // Update SHM position et velocity
    robot.getPosition(); // positionToSHM()
    //robot.getVelocity(); // velocityToSHM()

    // Execute a particular controller
    // Lire dans le SHM quel controlleur?
    // Exec cette controlleur

    switch (sh_memory->controller) {
      case 0/* value */:
        robot.ControllerNull(); break;
      case 1/*movement to point*/:
        robot.ControllerMoveToPoint(); break;
      case 2/*hold at point*/:
        robot.ControllerHoldAtPoint(); break;
    }

    robot.setForce();
    robot.forceToSHM();
    robot.computedPositionToSHM();
    robot.writeLog();
    //usleep(25000);

    // Get the current time
    //time_t t;
    //time(&t);
    clock_t t = clock();
    clock_t t1 = clock();

    // Compute how long this loop took

    //double seconds = difftime(t,t0);
    sh_memory->loop_time = ((double)(t-t0))/CLOCKS_PER_SEC;
    if (sh_memory->loop_time > MAIN_LOOP_TIME_S) {
      sh_memory->dropped_iterations +=1;
    }
    //printf("%f\n", MAIN_LOOP_ITERATION_TIME);
    while ((((double)(t1-t0))/CLOCKS_PER_SEC) < MAIN_LOOP_TIME_S) {
      t1 = clock();
      //printf("loop_time : %f\n", sh_memory->loop_time*1000000);
    }
    sh_memory->iteration_time = ((double)(t1-t0))/CLOCKS_PER_SEC;

    //usleep(MAIN_LOOP_ITERATION_TIME-(sh_memory->loop_time)*1000000);
  }

  robot.close_sharedmem();
  robot.closeDevice();
  return 0;
}
