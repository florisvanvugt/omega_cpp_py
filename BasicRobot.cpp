#include "BasicRobot.h"

BasicRobot::BasicRobot()
{
    //ctor
}

BasicRobot::~BasicRobot()
{
    //dtor
}



bool BasicRobot::keep_going() {
  // Tell us whether the robot needs to quit
  //return true;
  return (sh_memory)->quit==0;
}


//Prepare the shared memory
void BasicRobot::open_sharedmem() {
  int fd;

  //if (exists(fd))
  if( remove( SHM_FILEPATH ) != 0 )
    perror( "Error deleting shared memory file. Maybe it didn't exist, in which case it's okay" );
  
  //Open the memory
  fd = open(SHM_FILEPATH, O_RDWR | O_CREAT , (mode_t)0666);
  if (fd == -1) {
    perror("Error opening file for writing");
    exit(EXIT_FAILURE);
  }


  int result;
  //Put the cursor at the last byte of the shared memory
  result = lseek(fd, SHM_FILESIZE-1, SEEK_SET);
  if (result == -1) {
    close(fd);
    perror("Error calling lseek() to 'stretch' the file");
    exit(EXIT_FAILURE);
  }
  
  //Put this byte to 1, which will 'stretch' the memory file to ensure it has the correct size
  result = write(fd, "", 1);
  if (result != 1) {
    close(fd);
    perror("Error writing last byte of the file");
    exit(EXIT_FAILURE);
  }
  
  
  //Allocate the memory for the shared memory
  sh_memory = (shm_t *)mmap(0, SHM_FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  
  //Verify if the allocation worked
  if (sh_memory == MAP_FAILED) {
    close(fd);
    perror("Error mmapping the file");
    exit(EXIT_FAILURE);
  }
  
  //sh_memory->x = 0;
}


// Flushes the shared memory to ensure it is available to other processes
void BasicRobot::flush_sharedmem() {
  /* Apparently there is no guarantee that changes to the mmapped object
     are really visible to other processes with whom we share data.
     But I read conflicting reports about this.
     e.g. https://stackoverflow.com/questions/5902629/mmap-msync-and-linux-process-termination
     I decide to go ahead anyway.
  */
  msync(0,SHM_FILESIZE,MS_SYNC);
}


//Close the shared memory
void BasicRobot::close_sharedmem() {
  if (munmap(sh_memory, SHM_FILESIZE) == -1) {
      perror("Error un-mmapping the file");
  }

  if( remove( SHM_FILEPATH ) != 0 )
    perror( "Error deleting shared memory file" );
}

shm_t *BasicRobot::getShm(){
  return sh_memory;
}

//Open the robot
void BasicRobot::openDevice() {
  // open the first available device
  if (dhdOpen () < 0) {
      printf ("Error: Cannot open device (%s)\n", dhdErrorGetLastStr());
      sh_memory->quit = 1;
  }
  // identify device
}

//Get the position of the robot et put it in the shared memory
void BasicRobot::getPosition(){
  double x,y,z;
   if (dhdGetPosition(&(x),&(y),&(z)) < 0) {
     printf ("Error: Cannot read the position (%s)\n", dhdErrorGetLastStr());
     sh_memory->quit = 1;
   }

   sh_memory->x = x;
   sh_memory->y = y;
   sh_memory->z = z;

   if (sh_memory->record_flag == 1) {
     if (sh_memory->record_iterator<(sizeof(sh_memory->record_x)/sizeof(sh_memory->record_x[0]))) {
     (sh_memory->record_x)[sh_memory->record_iterator] = x;
     (sh_memory->record_y)[sh_memory->record_iterator] = y;
     (sh_memory->record_z)[sh_memory->record_iterator] = z;
     sh_memory->record_iterator = sh_memory->record_iterator+1;
   }
   }
}




//Get the velocity of the robot et put it in the shared memory
void BasicRobot::getVelocity(){
   if (dhdGetLinearVelocity(&(sh_memory->vel_x),&(sh_memory->vel_y),&(sh_memory->vel_z)) < 0) {
     printf ("Error: Cannot read the position (%s)\n", dhdErrorGetLastStr());
     sh_memory->quit = 1;
   }
}

//Close the device
void BasicRobot::closeDevice() {
  dhdClose();
}
