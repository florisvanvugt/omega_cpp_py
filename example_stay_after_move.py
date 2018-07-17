import robot
from sharedmem import *
from math import *

robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory


print("Move your arm to start")

while (sqrt(pow(rshm('vel_x'),2)+ pow(rshm('vel_y'),2)+ pow(rshm('vel_z'),2))<0.1):
    pass


while(sqrt(pow(rshm('vel_x'),2)+ pow(rshm('vel_y'),2)+ pow(rshm('vel_z'),2))>0.01):
    pass

robot.stay()

raw_input("Press <ENTER> to stop robot.")

robot.unload() #Stop the C++ script
