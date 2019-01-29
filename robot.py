from sharedmem import *
import subprocess
import os
import time
import sys
from math import *

# Default stiffness and damping to use for the movement controller
STIFFNESS = 2000
DAMPING = 20


# Default viscosity used for a viscous controller
VISCOSITY = 20


# The directory where the robot scripts are located
robot_dir = os.path.dirname(os.path.realpath(__file__))
#robot_dir = "."


FT_NCHANNELS = 16 # how many channels worth of data we are capturing (this has to match with COMEDI_NCHANNELS in the robot C++ code)


def init():
    """ Initialises the robot and the shared memory """
    init_specifications(robot_dir)
    init_shared_memory()
    wshm('controller',0)
    wshm('instruction_no',0)
    print("Waiting for robot to become active...")
    while rshm('active')!=1:
        pass
    print("ready.")


def launch(basedir=None):
    """ Launches the robot process."""
    # Launch the C process controlling the robot (which will allocate the shared memory that we will then connect to)

    if basedir!=None:
        global robot_dir
        robot_dir = basedir

    #if not os.path.isfile('./run_robot.sh'):
    #   print("The script ./run_robot.sh should exist - your installation seems to be broken.")
    #   sys.exit(-1)
    #if not os.path.isfile('./Robot'):
    #   print("The robot executable ./Robot should exist - maybe try 'make' to compile the robot code?")
    #   sys.exit(-1)

    out = subprocess.Popen('sudo ./run_robot.sh',cwd=robot_dir,shell=True)
    outp= out.communicate()[0]
    res = out.returncode
    if res!=0:
        print("## ERROR LOADING ROBOT")
    if outp:
        print("Robot loading: ",outp)

    return res


def load(): ## DEPRECATED
    """ Connects to the robot using the shared memory -- DEPRECATED"""
    pass


def return_home():
    """ Return to a home position, so that when we release the robot
    it doesn't fall too far. """
    print("Returning robot handle to home position...")
    move_to(.07,.005,.006,3.)
    while not move_is_done():
        pass
    print("... returned home.")


def unload():
    """ Terminates the robot process. """
    # Ideally tell the robot process to terminate itself
    wshm('quit',1)
    wait_for_the_new_instruction(10)





def safe_unload():
    """ Unloads the robot, but first returns to the home position
    so that when we remove forces, it doesn't drop all of a sudden."""
    return_home()
    unload()



def release():
    """ Release the robot (null field)."""
    wshm('controller',0)
    wait_for_the_new_instruction(7)


    
def wait_for_the_new_instruction( prefix ) :
    """
    So here we let C++ know that there is a new instruction set,
    and we (Python) wait until C++ has received it and put it in the live memory,
    so that we know that it is running the new instruction.

    Arguments
    prefix : the identifier of the group of variables that we want to push into the instruction memory (defined in variable_groups.yaml)
    """
    wshm ('prefix',prefix)
    inst = rshm('instruction_no')
    wshm('instruction_no',inst+1) # writing to instruction shm
    
    # Now wait until the robot has actually caught on on these new instructions and starts running them
    while rshm('instruction_no')!=inst+1: # reading from the live shm
        pass 

    

def move_is_done():
    """ Tells us whether a movement that we started is completed."""
    return rshm('move_done')==1





def move_to(x,y,z,t):
    """ Move the robot to position (x,y,z).
    Note that the function returns immediately, even when the movement
    is still ongoing. """

    wshm('stiffness',STIFFNESS)
    wshm('damping',DAMPING)
    sx,sy,sz=rshm('x'),rshm('y'),rshm('z')

    if (rshm('movement_cancelled')==1) :
        wshm('move_done',1)

    elif ((abs(x)>=rshm('x_lim')) or (abs(y)>=rshm('y_lim')) or (abs(z)>=rshm('z_lim'))) :
        print ("CAUTION : one of the target of move_to is out of the border. Movement cancelled")
        wshm('movement_cancelled', 1)
        wshm('move_done',1)

    elif (sqrt((x-sx)**2+(y-sy)**2+(z-sz)**2)/t>=rshm('distance_max')) :
        print ("CAUTION : the velocity of move_to will be too large. Movement cancelled")
        wshm('movement_cancelled', 1)
        wshm('move_done',1)

    else :
        wshm_vector(('target_x','target_y','target_z','movement_duration','start_x','start_y','start_z','move_done','move_iterator','controller'), (x,y,z,t,sx,sy,sz,0,0,1))

    wait_for_the_new_instruction(0);


def load_a_trajectory (x,y,z):
    M = max(len(x),len(y),len(z))
    wshm ('trajectory_size', M )
    if (rshm('movement_cancelled')==1) :
        pass
    elif ((max(map (abs,x))>=rshm('x_lim')) or (max(map (abs,y))>=rshm('y_lim')) or (max(map (abs,z))>=rshm('z_lim'))) :
        wshm('movement_cancelled', 1)
        print ("CAUTION : one of the target of the play is out of the border. Movement cancelled")
    elif (sqrt ((x[-1]-x[0])**2 + (y[-1]-y[0])**2 + (z[-1]-z[0])**2)/M >= rshm('distance_max')*rshm('main_loop_time')):
        wshm('movement_cancelled', 1)
        print ("CAUTION : the velocity of the play will be too important. Movement cancelled")
    else :
        wshm('trajectory_x',x)
        wshm('trajectory_y',y)
        wshm('trajectory_z',z)
        wshm('new_trajectory',1)
    wait_for_the_new_instruction(3)

def play_movement () :
    " Play the movement which is in trajectory in the shared memory"
    wshm('stiffness',STIFFNESS)
    wshm('damping',DAMPING)
    if (rshm('movement_cancelled')==1) :
        wshm('move_done',1)
        wshm('controller',0)
    else :
        wshm('move_iterator',0)
        wshm('controller',4)
        wshm('move_done',0)
    wait_for_the_new_instruction(4)



def hold_at(x=None,y=None,z=None):
    """
    Hold the robot at position (x,y,z)

    Arguments
    x,y,z : position to be held at; if they are None then the current position will be read.

    Caution
    You have to make sure the robot handle is really at (x,y,z) before calling this, otherwise
    it will snap to that location.
    """
    if x==None: x = rshm('x')
    if y==None: y = rshm('y')
    if z==None: z = rshm('z')

    wshm('stiffness',STIFFNESS)
    wshm('damping',DAMPING)

    if (sqrt ((x-rshm('x'))**2+(y-rshm('y'))**2+(z-rshm('z'))**2)<rshm('distance_max_hold')):
        wshm('target_x',x)
        wshm('target_y',y)
        wshm('target_z',z)
        wshm('controller',2)
    else :
        print (" CAUTION : The position where you want to stay is too far from the current position")
        wshm ('movement_cancelled', 1)

    wait_for_the_new_instruction(1)





def hold(x=None,y=None,z=None):
    hold_at(x,y,z)




def stay():
    """Hold the robot at the current position."""
    hold_at()



def stay_at(x,y,z):
    """Hold the robot at position (x,y,z)."""
    hold_at(x,y,z)

    
def three_d_to_two_d(x=None):
    """
    Essentially, turn the robot into a planar controller.
    """
    if x==None: x = rshm('x')
    wshm('stiffness',STIFFNESS)
    wshm('damping',DAMPING)
    wshm('target_x', x)
    wshm('controller',6)
    wait_for_the_new_instruction(2);


def viscous_force(viscosity=VISCOSITY):
    """Hold the robot at the current position."""
    wshm('viscosity',viscosity)
    wshm('controller',3)
    wait_for_the_new_instruction(5)

def active_to_null ():
    if (rshm('movement_cancelled')==1) :
        wshm('move_done',1)
    else :
        wshm('move_done',0)
        wshm('active_to_null',1000)
        wshm('move_iterator',0)
        wshm('controller',5)

    wait_for_the_new_instruction(6)


def start_capture():
    wshm('record_flag',1)
    wshm('record_iterator',0)
    wait_for_the_new_instruction(8)

    
# if end is true it will stop_capture and if it's false it will send the data without stop recording
def stop_capture(end):
    if (end) :
        wshm('record_flag',0)
        
    trajx = rshm('record_x')
    trajy = rshm('record_y')
    trajz = rshm('record_z')
    trajft = rshm('record_ft')
    iterator = rshm('record_iterator')

    wshm ('prefix',9)
    inst = rshm('instruction_no')
    wshm('instruction_no',inst+1)

    ft = trajft[:FT_NCHANNELS*iterator]
    
    return {"traj":zip(trajx[:iterator],trajy[:iterator],trajz[:iterator]),
            "ft":ft}
    #return trajectory that was captured

