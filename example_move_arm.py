import robot
import time



robot.launch()
robot.init()
robot.load()

raw_input("Press the robot force button now and press <ENTER>")

#time.sleep(4)

#vs   = ["x","y","z","fx","fy","fz","start_x","start_y","start_z","target_x","target_y","target_z", "minjerk_x", "minjerk_y", "minjerk_z", "controller","movet","t","mds","movement_duration","move_done", "main_loop_time"]
#print(" ".join(vs))

#for _ in range(50):
#    res  = [ str(robot.rshm(v)) for v in vs ]
#    print(" ".join(res))
#    time.sleep(.05)

print("Move starts!")
robot.move_to(0.02,0.02,0.02,2.)

while(robot.move_is_done() == False):
    pass
#    res  = [ str(robot.rshm(v)) for v in vs ]
#    print(" ".join(res))
#    time.sleep(.05)

robot.hold_at()
robot.return_home()
raw_input("Press <ENTER> when you want to stop the program")





robot.unload()
time.sleep(1)
