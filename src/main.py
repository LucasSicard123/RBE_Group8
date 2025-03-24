

        
# ---------------------------------------------------------------------------- #
#                                                                              #
# Module:       main.py                                                      #
# Author:       Charles Doh                                                      #
# Created:      3/21/2025, 8:32:54 AM                                        #
# Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import time

# Brain should be defined by default
brain = Brain()

wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 5 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference

#Instantiations
brain.screen.print("Hello V5, Team 8")

left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)

def turnDegrees(n_degrees): 
    degreesToRotatePerRobotRotation = (wheelTrack)/(wheelDiameter) #Degrees to turn per robot rotational degree.
    toTurn = degreesToRotatePerRobotRotation * n_degrees
    left_motor.spin_for(REVERSE, toTurn, DEGREES, 30, RPM, False)
    right_motor.spin_for(FORWARD, toTurn, DEGREES, 30, RPM, True)

def moveInches(n_inches): 
    degreesToTravel = n_inches * (degreesPerInch) # 1 : 1
    degreesToRotate = degreesToTravel * 5 #5 : 1, now it is gear ratio compliant
    left_motor.spin_for(FORWARD, degreesToRotate, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, degreesToRotate, DEGREES, 100, RPM, True)

def polygon(n_sides, n_length):
    degreesPerTurn = 360/n_sides
    for i in n_sides : 
        moveInches(n_length)
        turnDegrees(360/n_sides)

#Function calls begin again here
polygon(4,5)
