import robot
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from sharedmem import rshm

robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory





raw_input("Press <ENTER> to put the robot in place")

robot.move_to(0,-0.05,0,1)
while (robot.move_is_done() == False):
    pass  #wait until the end of the movement

print("let's go")
robot.start_capture()

robot.move_to(0,0.05,0,1) # move to point (x=-0.01,y=0.2,z=0.04.) in t=2 seconds.
while (robot.move_is_done() == False):
    pass  #wait until the end of the movement


robot.active_to_null()
while (robot.move_is_done() == False):
    pass  #wait until the end of the movement

raw_input("Press <ENTER> to stop robot.")

traj = robot.stop_capture(True)

x,y,z = zip(*traj)

print "The gap between two controller is ", rshm ('gap_iterator')/2 , "ms\n"
robot.unload() #Stop the C++ script


plt.figure(1)
ax = plt.axes(projection='3d')
ax.plot3D (x,y,z)
ax.set_xlim(-0.05,+0.05)
ax.set_ylim(-0.05,+0.05)
ax.set_zlim(-0.05,+0.05)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()
