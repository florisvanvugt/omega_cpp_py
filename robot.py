from sharedmem import *
import subprocess
import os
import time



# Stiffness to use for the movement controller
STIFFNESS = 2000
DAMPING = 20


def init():
    """ Initialises the robot and the shared memory """
    init_specifications()
    init_shared_memory()
    wshm('controller',0)
    print("Waiting for robot to become active...")
    while rshm('active')!=1:
        pass
    print("ready.")


def launch():
    """ Launches the robot process."""
    # Launch the C process controlling the robot (which will allocate the shared memory that we will then connect to)
    res = subprocess.call(['./run_robot.sh'])
    if (res != 0):
        print("Error in calling main!")

        
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



def safe_unload():
    """ Unloads the robot, but first returns to the home position
    so that when we remove forces, it doesn't drop all of a sudden."""
    return_home()
    unload()

    

def move_is_done():
    """ Tells us whether a movement that we started is completed."""
    return rshm('move_done')==1



def move_to(x,y,z,t):
    """ Move the robot to position (x,y,z).
    Note that the function returns immediately, even when the movement
    is still ongoing. """

    wshm('controller',0) # do not do anything while we set this up
    wshm('stiffness',STIFFNESS)
    wshm('damping',DAMPING)
    wshm('target_x',x)
    wshm('target_y',y)
    wshm('target_z',z)
    wshm('movement_duration',t)

    sx,sy,sz=rshm('x'),rshm('y'),rshm('z')
    wshm('start_x',sx)
    wshm('start_y',sy)
    wshm('start_z',sz)

    wshm('move_done',0)
    wshm('move_iterator',0)
    wshm('controller',1) # switch on the controller



    
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
    wshm('target_x',x)
    wshm('target_y',y)
    wshm('target_z',z)
    wshm('controller',2)


def hold(x=None,y=None,z=None):
    hold_at(x,y,z)
    
    


def stay():
    """Hold the robot at the current position."""
    hold_at()
    

    
def stay_at(x,y,z):
    """Hold the robot at position (x,y,z)."""
    hold_at(x,y,z)
    

    


def start_capture():
    wshm('record_flag',1)



def stop_capture():
    wshm('record_flag',0)
    trajx = rshm('record_x')
    trajy = rshm('record_y')
    trajz = rshm('record_z')
    iterator = rshm('record_iterator')

    return zip(trajx[:iterator],trajy[:iterator],trajz[:iterator])
    #return ... # trajectory that was captured
