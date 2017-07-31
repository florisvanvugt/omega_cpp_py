from sharedmem import *
import subprocess
import os
import time




def init():
    """ Initialises the robot and the shared memory """
    init_specifications()
    init_shared_memory()
    wshm('controller',0)


def launch():
    """ Launches the robot process."""
    # Launch the C process controlling the robot (which will allocate the shared memory that we will then connect to)
    res = subprocess.call(['./run_robot.sh'])
    if (res != 0):
        print("Error in calling main!")

        
def load():
    """ Connects to the robot using the shared memory."""
    pass


def unload():
    """ Terminates the robot process. """
    # Ideally tell the robot process to terminate itself
    wshm('quit',1)


def move_is_done():
    """ Tells us whether a movement that we started is completed."""
    return rshm('move_done')==1
    #    wshm('flag',0)
    #    return True
    #else:
    #return False


def move_to(x,y,z,t):
    """ Move the robot to position (x,y,z).
    Note that the function returns immediately, even when the movement
    is still ongoing. """

    wshm('target_x',x)
    wshm('target_y',y)
    wshm('target_z',z)

    wshm('movement_duration',t)

    sx,sy,sz=rshm('x'),rshm('y'),rshm('z')

    wshm('start_x',sx)
    wshm('start_y',sy)
    wshm('start_z',sz)

    wshm('move_done',0)
    wshm('movet',0)
    wshm('controller',1)


def hold_at():
    """Hold the robot at position (x,y,z)"""
    wshm('controller',2)




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
