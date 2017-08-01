
import robot
import time

robot.launch()
robot.init()

VERBOSE = True

time.sleep(5) # let the robot wake up gently

if True:

    for T in [5,10,20,30,40,50,60]:
        for mlp in [.001,.0025,.003,.005,.01,.02,.1,.5]:
            robot.wshm('main_loop_time',mlp)
            time.sleep(.1) # let things settle down

            if VERBOSE:
                print("")
                print("Running loop test for %f seconds"%T)
            t0 = time.time()
            it0      = robot.rshm('loop_iterator')
            robott0  = robot.rshm('clock')
            t1 = time.time()
            while t1<t0+T:
                it1      = robot.rshm('loop_iterator')
                robott1  = robot.rshm('clock')
                t1       = time.time()
            dropped = robot.rshm('dropped_iterations')
            if VERBOSE:
                print("Passed %i samples i.e. loop time %f ms and %i dropped loops"%(it1-it0,
                                                                                     T/(it1-it0)*1000,
                                                                                     dropped))
                print("Python thinks this took %f s, the robot thinks this took %f s"%(t1-t0,robott1-robott0))
                print("The robot thinks it has a loop time of %f ms"%(1000*(robott1-robott0)/(it1-it0)))
                print("Desired loop time %f ms"%(1000*robot.rshm('main_loop_time')))
                print("Clocks per second %f"%robot.rshm('clocks_per_sec'))
                print("");
            print("%f %i %i %f %f %f %f %i %f"%(T,it0,it1,t0,t1,robott0,robott1,dropped,mlp))




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


