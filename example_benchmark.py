
import robot
import time

robot.launch()
robot.init()


if True:

    T = 5. # seconds
    print("Running loop test for %f seconds"%T)
    t0 = time.time()
    it0      = robot.rshm('loop_iterator')
    robott0  = robot.rshm('clock')
    t1 = time.time()
    while t1<t0+T:
        it1      = robot.rshm('loop_iterator')
        robott1  = robot.rshm('clock')
        t1       = time.time()
    print("Passed %i samples i.e. loop time %f ms and %i dropped loops"%(it1-it0,
                                                                         T/(it1-it0)*1000,
                                                                         robot.rshm('dropped_iterations')))
    print("Python thinks this took %f s, the robot thinks this took %f s"%(t1-t0,robott1-robott0))
    print("The robot thinks it has a loop time of %f ms"%(1000*(robott1-robott0)/(it1-it0)))
    print("Desired loop time %f ms"%(1000*robot.rshm('main_loop_time')))




if False:

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


