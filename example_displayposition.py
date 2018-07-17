import robot
import time


robot.launch()
robot.init()

i = 0
while((i<50) and (robot.rshm('quit') != 1)):
    x = robot.rshm('x')
    y = robot.rshm('y')
    z = robot.rshm('z')
    print("x=%f  &  y=%f  &  z=%f"%(x, y, z))
    time.sleep(.1)
    i += 1

print("Unloading...")
robot.unload()
time.sleep(1)

