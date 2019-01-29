
import robot
import time
from sharedmem import *

import matplotlib.pyplot as plt

robot.launch()
robot.init()

STIFFNESS = 2000
DAMPING = 20

T = 5. # seconds


raw_input("Press <ENTER> to start capture")
robot.start_capture()


time.sleep(T)

capt = robot.stop_capture(True)
traj = capt["traj"]
ft   = capt["ft"]

#ft = np.array(ft,shape=(len(traj),robot.FT_NCHANNELS))
ft = np.array(ft).reshape(len(traj),robot.FT_NCHANNELS)

plt.plot(ft)
plt.show()


robot.unload()



plt.plot(traj)
(x,y,z)=traj[-1]
plt.text(len(traj),x,'x')
plt.text(len(traj),y,'y')
plt.text(len(traj),z,'z')
plt.show()

raw_input("Press <ENTER> to stop robot.")
