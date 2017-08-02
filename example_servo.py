
import robot
import time
import json
import matplotlib.pyplot as plt


robot.launch()
robot.init()

robot.DAMPING = 0
robot.STIFFNESS = 2000
startx,starty,startz = .06,.03,.015
endx,endy,endz       = .002,.085,.05
T = 2 # seconds

raw_input("Press <ENTER> to begin")
print("Moving to start")
robot.move_to(startx,starty,startz,2)
while not robot.move_is_done():
    pass
robot.hold(startx,starty,startz)
print("Holding")
time.sleep(1)

robot.move_to(endx,endy,endz,T)
robot.start_capture()
print("Moving to end")
while not robot.move_is_done():
    pass
traj = robot.stop_capture()
print("Holding")
robot.hold(endx,endy,endz)

time.sleep(1)
stiff,damp = robot.rshm('stiffness'),robot.rshm('damping')

print("Unloading")
robot.unload()



with open('servo.json.txt', 'w') as outfile:
    json.dump({"start":(startx,starty,startz),
               "end":(endx,endy,endz),
               "stiffness":stiff,"damping":damp,
               "trajectory":traj}, outfile)


    

plt.plot(traj)
(x,y,z)=traj[-1]
plt.text(len(traj),x,'x')
plt.text(len(traj),y,'y')
plt.text(len(traj),z,'z')
plt.show()



