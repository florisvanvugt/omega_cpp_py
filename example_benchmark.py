
import robot
import time

#robot.launch()
robot.init()


T = 5. # seconds
print("Loop test...")
it0 = robot.rshm('loop_iterator')
time.sleep(T)
it1 = robot.rshm('loop_iterator')
print("This took %i samples i.e. loop time %f us"%(it1-it0, T/(it1-it0)*1000000))
print(robot.rshm('dropped_iterations'))


raw_input("Press the robot force button now and press <ENTER>")

#robot.start_capture()
#Initialize the position of the robot
robot.move_to(0,0,0,5)

t0 = time.time()

while(robot.move_is_done() == False):
    pass

t = time.time()
print("This took %f seconds"%(t-t0))

#traj = robot.stop_capture()
robot.hold_at()

#print('iterator=%i',robot.rshm('record_iterator'))

raw_input("Press a key to continue <ENTER>")

#robot.start_capture()

robot.move_to(0.02,0.02,0.02,1)

while(robot.move_is_done() == False):
    pass
#robot.stop_capture()
robot.hold_at()

print('iterator=%i',robot.rshm('record_iterator'))

raw_input("Press a key to continue <ENTER>")

#robot.start_capture()

robot.move_to(0,0,0,1)

while(robot.move_is_done() == False):
    pass
#robot.stop_capture()
robot.hold_at()

print('iterator=%i',robot.rshm('record_iterator'))

raw_input("Press a key to continue <ENTER>")

#robot.start_capture()

robot.move_to(0.02,0.02,0.02,1)

while(robot.move_is_done() == False):
    pass
#traj = robot.stop_capture()


robot.hold_at()

robot.unload()

#print(traj)
