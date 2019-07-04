#include "BasicRobot.h"
#include <fstream>
#include <cstdarg>
#include <string>



BasicRobot::BasicRobot()
{
}


BasicRobot::~BasicRobot()
{
}



bool BasicRobot::keep_going() {
  // Tell us whether the robot needs to quit
  //return true;
  if (sh_live_memory->quit==1) {
    printDebug ("Quit signal received, exiting.");
    return 0;
  }
  return 1;
}


//This function is called by open_sharedmen, it avoids to write twice the same lines for instr and live memory
void BasicRobot::create_file_of_size(const char *fp, int fs, int *fd) {
    if( remove( fp ) != 0 ){
      printDebug( "Error deleting shared memory file. Maybe it didn't exist, in which case it's okay" );
    }
    //Open the memory
    *fd = open(fp, O_RDWR | O_CREAT , (mode_t)0666);
    if (*fd == -1) {
      printDebug("Error opening file for writing");
      exit(EXIT_FAILURE);
    }
    int result;
    //Put the cursor at the last byte of the shared memory
    result = lseek(*fd, fs-1, SEEK_SET);
    if (result == -1) {
      close(*fd);
      printDebug("Error calling lseek() to 'stretch' the file");
      exit(EXIT_FAILURE);
    }
    //Put this byte to 1, which will 'stretch' the memory file to ensure it has the correct size
    result = write(*fd, "", 1);
    if (result != 1) {
      close(*fd);
      printDebug("Error writing last byte of the file");
      exit(EXIT_FAILURE);
    }
}


//Prepare the shared memory
void BasicRobot::open_sharedmem() {
  int fd;
  //Prepare the shared instr memory
  if (SHM_INSTR_FILESIZE!=sizeof(shm_instr_t)) {
    printDebug("Error: the shared memory is not the correct size. Expected size %i vs actual size %i. This could be due to memory padding.",SHM_INSTR_FILESIZE,sizeof(shm_instr_t));
    exit(EXIT_FAILURE);
  }

  create_file_of_size(SHM_INSTR_FILEPATH, SHM_INSTR_FILESIZE, &fd);
  //Allocate the memory for the shared memory
  sh_instr_memory = (shm_instr_t *)mmap(0, SHM_INSTR_FILESIZE, PROT_READ, MAP_SHARED, fd, 0);
  //Verify if the allocation worked
  if (sh_instr_memory == MAP_FAILED) {
    close(fd);
    printDebug("Error mmapping the file");
    exit(EXIT_FAILURE);
  }
  //Prepare the shared live memory
  if (SHM_LIVE_FILESIZE!=sizeof(shm_live_t)) {
    printDebug("Error: the shared memory is not the correct size. Expected size %i vs actual size %i. This could be due to memory padding.",SHM_LIVE_FILESIZE,sizeof(shm_live_t));
    exit(EXIT_FAILURE);
  }
  int fe;
  create_file_of_size(SHM_LIVE_FILEPATH, SHM_LIVE_FILESIZE, &fe);
  //Allocate the memory for the shared memory
  sh_live_memory = (shm_live_t *)mmap(0, SHM_LIVE_FILESIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fe, 0);
  //Verify if the allocation worked
  if (sh_live_memory == MAP_FAILED) {
    close(fe);
    printDebug("Error mmapping the file");
    exit(EXIT_FAILURE);
  }
}

// Flushes the shared memory to ensure it is available to other processes
void BasicRobot::flush_sharedmem() {
  /* Apparently there is no guarantee that changes to the mmapped object
     are really visible to other processes with whom we share data.
     But I read conflicting reports about this.
     e.g. https://stackoverflow.com/questions/5902629/mmap-msync-and-linux-process-termination
     I decide to go ahead anyway.
  */
  msync(0,SHM_INSTR_FILESIZE,MS_SYNC);
  msync(0,SHM_LIVE_FILESIZE, MS_SYNC);
}

//Close the shared memory
void BasicRobot::close_sharedmem() {
  if (munmap(sh_instr_memory, SHM_INSTR_FILESIZE) == -1 || munmap(sh_live_memory, SHM_LIVE_FILESIZE) == -1) {
    printDebug("Error un-mmapping the shared memory file");
  }
  if( remove( SHM_INSTR_FILEPATH ) != 0 || remove (SHM_LIVE_FILEPATH ) != 0)
    printDebug( "Error deleting shared memory file" );
}


shm_instr_t *BasicRobot::getShmi(){
  return sh_instr_memory;
}

shm_live_t *BasicRobot::getShml(){
  return sh_live_memory;
}


//Open the robot
void BasicRobot::openDevice() {
  // open the first available device
  if (dhdOpen () < 0) {
      printDebug ("Error: Cannot open device (%s)\n", dhdErrorGetLastStr());
      printf ("Error: Cannot open device (%s) -- this can be because the robot is switched off (which has a fairly obvious fix) OR because another robot process is active (if so, try killing it with make kill)\n", dhdErrorGetLastStr());
      sh_live_memory->quit = 1;
  }

  // identify device
  printDebug ("'%d' device detected", dhdGetSystemName());
  // set velocity estimation mode
  //dhdConfigLinearVelocity(20,DHD_VELOCITY_WINDOWING,-1);

  // set gravity compensation
  dhdSetGravityCompensation(DHD_ON);
  dhdSetStandardGravity (15);
  // use 15 for the heavy ball that someone made for us (don't ask)
  // use 12 for the "cylindrical handle" or ball with the support clamp (whatever that means, people will be wondering for centuries from now...)

  // release brakes!
  dhdSetBrakes(DHD_OFF);


}



//Get the position of the robot et put it in the shared memory
void BasicRobot::readSensors(){
  double x,y,z;

   if (dhdGetPosition(&x,&y,&z) < 0) {
     printDebug ("Error: Cannot read the position (%s)", dhdErrorGetLastStr());
     sh_live_memory->quit = 1;
   }

   sh_live_memory->x = x;
   sh_live_memory->y = y;
   sh_live_memory->z = z;


   double vx,vy,vz;
   if (dhdGetLinearVelocity(&vx,&vy,&vz) < 0) {
     printDebug ("Error: Cannot read the position (%s)", dhdErrorGetLastStr());
     sh_live_memory->quit = 1;
   }
   sh_live_memory->vel_x = vx;
   sh_live_memory->vel_y = vy;
   sh_live_memory->vel_z = vz;

}


// Record the current position if so requested
void BasicRobot::recordPosition() {
   if (sh_instr_memory->record_flag == 1) {
     // Test whether we have still space in our buffer, if not, do not record anything
     if (sh_live_memory->record_iterator<(unsigned)(sizeof(sh_live_memory->record_x)/sizeof(sh_live_memory->record_x[0]))) {
       (sh_live_memory->record_x)[sh_live_memory->record_iterator] = (sh_live_memory->x);
       (sh_live_memory->record_y)[sh_live_memory->record_iterator] = (sh_live_memory->y);
       (sh_live_memory->record_z)[sh_live_memory->record_iterator] = (sh_live_memory->z);


       // Now record the FT samples in the same way
       int i;
       int retval;
       int chan; // the channel we are currently reading
       lsampl_t data;
       comedi_range *rangetype;
       lsampl_t maxdata;
       double volts;
	
       for (chan=0; chan<COMEDI_NCHANNELS; chan++)
	 {
	   retval      = comedi_data_read(comedidev, COMEDI_SUBDEVICE, chan, COMEDI_RANGE, COMEDI_AREF, &data);
	   if(retval < 0) { comedi_perror("comedi_data_read"); }
	   rangetype   = comedi_get_range(comedidev,COMEDI_SUBDEVICE,chan,COMEDI_RANGE);
	   maxdata     = comedi_get_maxdata(comedidev,COMEDI_SUBDEVICE,chan);
	   volts       = comedi_to_phys(data,rangetype,maxdata);

	   // Where does this go in the record_ft?
	   i = sh_live_memory->record_iterator*COMEDI_NCHANNELS + chan;
	   (sh_live_memory->record_ft)[i] = volts;
	   
	 }
       
       sh_live_memory->record_iterator = sh_live_memory->record_iterator+1;
       
     }
   }
}




//Close the device
void BasicRobot::closeDevice() {
  dhdClose();
}
//Debug log functionality

// Formatted printing to an output stream
char* printformat(string fmt, size_t size, ...)
{
  const char* formt = fmt.c_str();
  static char buffer[500] = ""; // note the maximum length of the format
  va_list argptr;
  va_start(argptr,size); // formt
  vsprintf(buffer,formt, argptr);
  va_end(argptr);
  return buffer;
}


// Open the debug log file
void BasicRobot::openDebug() {
  debugfp = fopen (DEBUG_FILENAME,"a");
  fprintf(debugfp,"\n\n");
}





// Print a line to the debug file
void BasicRobot::printDebug(const char *fmt, ...) {
  //debugfp << msg;

  fprintf(debugfp,"[Robot %i] ",::getpid());
  time_t timer; struct tm* tm_info;
  char tbuffer[24];
  time(&timer);
  tm_info = localtime(&timer);
  strftime(tbuffer, sizeof(tbuffer), "%Y-%m-%d %H:%M:%S ", tm_info);
  fwrite(tbuffer,sizeof(char),sizeof(tbuffer),debugfp);

  va_list arglist;
  va_start( arglist, fmt );
  vfprintf( debugfp, fmt, arglist );
  va_end( arglist );

  fprintf(debugfp,"\n");
  fflush(debugfp);
}



// Close the debug log
void BasicRobot::closeDebug() {
  fflush(debugfp);
  fclose(debugfp);
}





/*
 *
 *
 * Stuff that relates to reading Force-Torque sensors through Comedi
 *
 *
 */


void BasicRobot::openComedi() {
  
  comedidev = comedi_open(COMEDI_DEV);
  if(comedidev == NULL) {
    comedi_perror("comedi_open");
  }
  
}


void BasicRobot::readComedi() {
  int retval;
  lsampl_t data;
  int chan = 0;		        /* change this to your channel */
  retval = comedi_data_read(comedidev, COMEDI_SUBDEVICE, chan, COMEDI_RANGE, COMEDI_AREF, &data);
  if(retval < 0) {
    comedi_perror("comedi_data_read");
  }
  printf("%d\n", data);
  
}



void BasicRobot::closeComedi() {
}



