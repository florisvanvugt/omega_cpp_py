import robot
import time

DISPLAY_TIME = .025

robot.launch()
robot.init()


vs   = ["x","y","z","fx","fy","fz",
        "start_x","start_y","start_z","target_x","target_y","target_z",
        "desired_x","desired_y","desired_z","stiffness",
        "controller","movet","t","mds","movement_duration","move_done", "main_loop_time"]
print(" ".join(vs))

for _ in range(20):
    print(" ".join([ str(robot.rshm(v)) for v in vs ]))
    time.sleep(DISPLAY_TIME)

raw_input("Move robot to desired location and robot force button and press <ENTER>")

robot.stay()


for _ in range(100):
    print(" ".join([ str(robot.rshm(v)) for v in vs ]))
    time.sleep(DISPLAY_TIME)

raw_input("Press <ENTER> to start moving...")




print("Move starts!")
robot.move_to(0.02,0.02,0.02,2.)

while(robot.move_is_done() == False):
    print(" ".join([ str(robot.rshm(v)) for v in vs ]))
    time.sleep(DISPLAY_TIME)

robot.stay()
raw_input("Press <ENTER> when you want to stop the program")

robot.return_home()
raw_input("Press <ENTER> when you want to stop the program")





robot.unload()
time.sleep(1)
