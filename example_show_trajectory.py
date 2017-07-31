
import robot
import time

robot.launch()
robot.init()


T = 1. # seconds

raw_input("Press <ENTER> to start capture")

robot.start_capture()

time.sleep(T)

traj = robot.stop_capture()
robot.unload()



import matplotlib.pyplot as plt

plt.plot(traj)
(x,y,z)=traj[-1]
plt.text(len(traj),x,'x')
plt.text(len(traj),y,'y')
plt.text(len(traj),z,'z')
plt.show()

raw_input("Press <ENTER> to stop robot.")
