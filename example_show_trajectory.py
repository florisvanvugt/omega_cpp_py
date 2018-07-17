
import robot
import time
from sharedmem import *

robot.launch()
robot.init()

STIFFNESS = 2000
DAMPING = 20

T = 5. # seconds


raw_input("Press <ENTER> to start capture")
robot.start_capture()


time.sleep(T)

traj = robot.stop_capture(True)
robot.unload()



import matplotlib.pyplot as plt

plt.plot(traj)
(x,y,z)=traj[-1]
plt.text(len(traj),x,'x')
plt.text(len(traj),y,'y')
plt.text(len(traj),z,'z')
plt.show()

raw_input("Press <ENTER> to stop robot.")
