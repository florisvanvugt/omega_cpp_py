import robot
import time


robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory


## Communicate some basic parameters to the robot
conf = {}
# The arc defines the path to be followed and will be drawn on the screen using lines.
conf["ARC_RADIUS"] = .005 # the radius of the arc in robot coordinates (m)
conf["ARC_BASE_X"] = 0 # the x position of the center of the arc (robot Y dimension)
conf["ARC_BASE_Y"] = 0 # the y position of the targets (in robot coordinates) (robot Z dimension)

conf["X_PLANE"]=0  # Defines the plane within which we restrict the robot


startx = conf["ARC_BASE_X"]+2*conf["ARC_RADIUS"]
starty = conf["ARC_BASE_Y"]

raw_input("Press <ENTER> to put the robot in place.")

robot.move_to(conf["X_PLANE"],startx,starty,3)
while (robot.move_is_done() == False):
    pass  #wait until the end of the movement


def channel_trial(x_plane,arc_base_x,arc_base_y,arc_radius):
    " Play the movement which is in trajectory in the shared memory"
    robot.wshm('arc_x_plane', x_plane)
    robot.wshm('arc_base_y',  arc_base_x)
    robot.wshm('arc_base_z',  arc_base_y)
    robot.wshm('arc_radius',  arc_radius)
    robot.wshm('controller',  7)
    robot.wait_for_the_new_instruction(11) # update the variable group corresponding to this controller

raw_input("Press <ENTER> to enable the arc force channel.")

channel_trial(conf["X_PLANE"],
              conf["ARC_BASE_X"],
              conf["ARC_BASE_Y"],
              conf["ARC_RADIUS"])

t0=time.time()

log = open('quicklog.csv','w')
showvars = 'x y z desired_y desired_z arc_base_y arc_base_z arc_radius arc_dist'.split()
h = " ".join(showvars)
log.write(h+"\n")
print(h)
while time.time()-t0<10:
    vals=robot.rshm(showvars)
    v = " ".join([ "%.03f"%f for f in vals])
    print(v)
    log.write(v+"\n")
    time.sleep(.1)

log.close()
pass # do something to enable the force channel

raw_input("Press <ENTER> to unload the robot.")
robot.unload()
