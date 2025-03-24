# ---------------------------------------------------------------------------- #
#                                                                              #
# Module:       main.py                                                        #
# Author:       Lucas, Charles, Max                                            #
# Created:      3/21/2025, 8:32:54 AM                                          #
# Description:  V5 project                                                     #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import time
# from drivetrainController import drivetrainController

# Brain should be defined by default
brain = Brain()

wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 5 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference

#Instantiations
brain.screen.print("Hello V5, Team 8")
# controller = drivetrainController(Controller(ControllerType.PRIMARY), brain) #experimental as hell 
left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)

#!Turns Robot among rotational center n degrees COUNTER CLOCKWISE
def turnDegrees(n_degrees): 
    degreesToRotatePerRobotRotation = wheelTrack / wheelDiameter #Degrees to turn per robot rotational degree.
    toTurn = degreesToRotatePerRobotRotation * n_degrees * gearRatio
    left_motor.spin_for(REVERSE, toTurn, DEGREES, 30, RPM, False)
    right_motor.spin_for(FORWARD, toTurn, DEGREES, 30, RPM, True)

#!Moves n(integer) Inches forwards @ speed_rpm. 
def moveInches(n_inches, speed_rpm): 
    degreesToTravel = n_inches * degreesPerInch # 1 : 1
    degreesToRotate = degreesToTravel * gearRatio #5 : 1, now it is gear ratio compliant
    #terribly done parallel-command group
    left_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, False) 
    right_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, True)

#!Polygon function : To turn in a equilateral polygon of n_sides, with n_length per side. Breaks down
def polygon(n_sides, n_length):
    if n_sides <= 2 :
        brain.screen.print("Invalid Number of sides") 
        return

    degreesPerTurn = 360/n_sides
    for i in range(n_sides) : 
        moveInches(n_length, 50)
        turnDegrees(degreesPerTurn)

#! Predefined maze solution, assuming correct orientation of the beginning 
def solveMaze():
    moveInches(27,50) #Assuming the front wheel is by the starting line. This should end up a little before the wall to the left(18in one)
    turnDegrees(90) #Counter Clockwise
    moveInches(27,50)
    # moveInches(16,50) #TODO: Check this
    turnDegrees(-88) #Hopefully this works with clockwise movement 
    moveInches(15, 50) #Might be too tight
    turnDegrees(-88) #Clockwise
    moveInches(9, 50) #Might be a little short 
    return

#Function calls begin again here
# polygon(6,5)
solveMaze()
