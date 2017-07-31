import random
import time
import robot
import os
import numpy as np
import math
import pygame
from pygame.locals import *

def angle_to_position(distance, angle):
    """Compute coordonates to move the robot with this angle and this distance"""
    coord = np.zeros((3,1),dtype='f')
    normedAngle = -angle+90
    actualx = robot.rshm('x')
    actualy = robot.rshm('y')
    actualz = robot.rshm('z')

    coord[0] = -math.sin(math.radians(normedAngle))*distance + actualx;
    coord[1] = math.cos(math.radians(normedAngle))*distance + actualy;
    coord[2] = actualz

    return coord

#Give a report about to the subject about his response
def subject_report(angle, response):
    #Initialisation
    pygame.init()
    Yes = pygame.mixer.Sound("sounds/EN_yes.wav")
    No = pygame.mixer.Sound("sounds/EN_no.wav")

    #Try all the possible cases
    if (angle < 0 and response == 'l'):
        print("Right")
        Yes.play()
    elif (angle > 0 and response == 'r'):
        print("Right")
        Yes.play()
    else:
        print("Wrong")
        No.play()


#create .txt to keep the subject's responses at the Extreme Test in
def create_txt_extreme(first_name, last_name, day):
    name = os.getcwd() + "/Results/"+ str(day) + "/" + first_name + "_" + last_name + "_extreme.txt"
    f = open(name,"w")
    f.write("Subject's Responses, angle : \n")
    return f


#create .txt to keep the subject's responses at the Perceptual Test in
def create_txt_perceptual(first_name, last_name, day, block):
    name = os.getcwd() + "/Results/"+ str(day) + "/" + first_name + "_" + last_name + "_somatic_" + str(block) + ".txt"
    f = open(name,"w")
    f.write("Subject's Responses, angle: \n")
    return f




#Ask the subject ID
subject_fstn = raw_input("Enter subject first name: ")
subject_lstn = raw_input("Enter subject last name: ")

#choose the test you want to use
typeTest = raw_input("What test do you want ? (Extreme/Perceptual)  ")
while ( typeTest != 'Extreme' and typeTest != 'Perceptual' ) :
    typeTest = raw_input("What test do you want ? (Extreme/Perceptual)  ")

#Ask the day of the trial
while True:
    try :
        day = int(raw_input("Enter day : "))
        break
    except OSError:
        print("The day must be an integer.")

#Creation of a path to keep all the data in
PATH = os.getcwd() + "/Results/"

try :
    os.mkdir(PATH)
except OSError:
    print("Path already exists")

#Creation of a path to keep the data of one day in
PATH = PATH + str(day) + '/'

try :
    os.mkdir(PATH)
except OSError:
    pass

robot.launch()
robot.init()
robot.load()

raw_input("Press the robot force button now and press <ENTER>")




"""Extreme Test"""

if (typeTest == 'Extreme'):
    print("\nWelcome to the Extreme Test\n")

    ftxt = create_txt_extreme(subject_fstn, subject_lstn, day)

    done = 0
    #Continue while the user wants to
    while done == 0:

        #Ask the angle of the trial
        while True:
            try :
                angle = int(raw_input("Enter angle: "))
                break
            except ValueError:
                print("The angle must be an integer.")

        print("\nThe test is going to start.\n")
        time.sleep(2)

        #Initialize the position of the robot
        robot.move_to(0,0,0,1)

        while(robot.move_is_done() == False):
            pass

        #Hold the robot at this position until the first trial starts
        robot.hold_at()

        #Compute a table of angles randomly
        tabAngles = [angle,-angle]*2
        random.shuffle(tabAngles)

        for ang in tabAngles:
            time.sleep(2)

            #Write the angle in the shm
            robot.wshm('angle', ang)

            #compute the coordonates fo the point corresponding to the distance and the angle
            coord = angle_to_position(0.05, ang)

            #Move the robot to this position
            robot.move_to(coord[0], coord[1], coord[2],1)

            while(robot.move_is_done() == False):
                pass

            #Hold the robot at the position until the user says Right or Left
            robot.hold_at()
            response = raw_input("Press <r> for Right or <l> for Left and <ENTER> then : ")
            while (response != 'r' and response != 'l'):
                response = raw_input("Press <r> for Right or <l> for Left and <ENTER> then : ")

            #Write the subject's response in the .txt document
            if response == 'r':
                ftxt.write("Right, " + str(ang) + "\n")
            else:
                ftxt.write("Left, " + str(ang) + "\n")
            ftxt.flush()

            #Reinitialize the position of the robot
            robot.move_to(0,0,0,1)

            while(robot.move_is_done() == False):
                pass

            robot.hold_at()

        #Ask the user if he wants to do the Extreme test with another angle
        othertrial = raw_input("Do you want to try an other angle ? [y/n]  ")
        while (othertrial != 'y' and othertrial != 'n'):
            othertrial = raw_input("Do you want to try an other angle ? [y/n]  ")

        if (othertrial == 'n'):
            done = 1








"""Perceptual Test"""

if (typeTest == 'Perceptual'):
    print("\nWelcome to the Perceptual Test. Each day, this test lasts for 4 blocks.\n")

    #Ask the block number
    while True:
        try :
            block = int(raw_input("Enter block number: "))
            break
        except ValueError:
            print("The angle must be an integer.")

    ftxt = create_txt_perceptual(subject_fstn, subject_lstn, day, block)

    #Ask the extreme angle of the trial
    while True:
        try :
            extreme_angle = int(raw_input("Enter extreme angle: "))
            break
        except ValueError:
            print("The angle must be an integer.")

    print("\nThe test is going to start.\n")
    time.sleep(2)

    #Initialize the position of the robot
    robot.move_to(0,0,0,1)

    tabAngles = [0.25,0.5,0.75,1]
    tabAngles = [i*extreme_angle for i in tabAngles]
    tabAngles.extend([i*(-1) for i in tabAngles])
    tabAngles = tabAngles
    random.shuffle(tabAngles)

    while(robot.move_is_done() == False):
        pass

    robot.hold_at()

    for ang in tabAngles:
        time.sleep(2)

        #Write the angle in the shm
        robot.wshm('angle', ang)

        #compute the coordonates fo the point corresponding to the distance and the angle
        coord = angle_to_position(0.05, ang)

        #Move the robot to this position
        robot.move_to(coord[0], coord[1], coord[2],1)

        while(robot.move_is_done() == False):
            pass

        #Hold the robot at the position until the user says Right or Left
        robot.hold_at()
        response = raw_input("Press <r> for Right or <l> for Left and <ENTER> then : ")
        while (response != 'r' and response != 'l'):
            response = raw_input("Press <r> for Right or <l> for Left and <ENTER> then : ")

        #Write the subject's response in the .txt document
        if response == 'r':
            ftxt.write("Right, " + str(ang) + "\n")
        else:
            ftxt.write("Left, " + str(ang) + "\n")
        ftxt.flush()

        #Give a good or bad report about the subject's response
        subject_report(ang,response)

        #Reinitialize the position of the robot
        robot.move_to(0,0,0,1)

        while(robot.move_is_done() == False):
            pass

        robot.hold_at()

    #Hold the robot at this position until the first trial starts
    robot.hold_at()





#Stop the program softly for the subject comfort
robot.hold_at()
raw_input("Press <ENTER> when you want to stop the program")

ftxt.close()
robot.unload()
time.sleep(1)
