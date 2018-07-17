import numpy as np
import robot
from sharedmem import *



robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory


raw_input("Press <ENTER> to put the robot in place")

# Define the trajectory
n=np.linspace(-0.05, 0.05, 1000)
o=[0]
robot.load_a_trajectory(o,n,10*n*n)


# Move the robot to the initial position
robot.move_to(0,n[0],10*n[0]*n[0],1)
while (robot.move_is_done() == False):
    pass

# Play the trajectory
robot.start_capture()
robot.play_movement ()
while (robot.move_is_done() == False):
    pass
traj=robot.stop_capture(True)

robot.unload() #Stop the C++ script

robot.plot_records(True) # Plot the movement that has been recorded during the trajectory
