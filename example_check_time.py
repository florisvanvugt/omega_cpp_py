import robot
import time
from sharedmem import *

robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory


ts = []
ts.append( time.time() )


# Send a command to the robot


robot.move_to(0, 0.05, 0, 1)


# t[1] = time for writing an instruction
ts.append( time.time())

while (rshm('move_done')!=1):
    pass


#t[-1] = time for python to detect the new value in the share live memory
ts.append( time.time())

dts = [ t-ts[0] for t in ts ]

print(dts)
robot.unload()
