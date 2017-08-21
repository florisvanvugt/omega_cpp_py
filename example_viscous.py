import robot
import time


robot.launch()
robot.init()

raw_input("Switch on the robot force...")

raw_input("Press <ENTER> to begin viscous force")
robot.viscous_force(50)

vr = ['x','y','z','vel_x','vel_y','vel_z','fx','fy','fz','viscosity','controller']
print(" ".join(vr))
for _ in range(20):
    print(" ".join([str(robot.rshm(v)) for v in vr]))
    time.sleep(.5)


raw_input("Press <ENTER> to begin")
robot.release()

raw_input("Press <ENTER> to unload")
robot.unload()
