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
controller = Controller(ControllerType.PRIMARY)

wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 5 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference

#Instantiations
brain.screen.print("Hello V5, Team 8")
left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)
rangeFinderFront = Sonar(brain.three_wire_port.g)
rangeFinderSide = Sonar(brain.three_wire_port.a)
#DriveTraincontroller = drivetrainController(controller, brain, left_motor, right_motor) #experimental as hell 

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

def movePercent(n_percent, speed_rpm): 
    left_motor.spin(FORWARD, n_percent, PERCENT)
    right_motor.spin(FORWARD, n_percent, PERCENT)

#Helper function, this will be used often to validate input
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def rangeCheck(value, min_Value, max_value): 
    if value >= min_Value and value <= max_value:
        return True
    return False

#Drive function, WIP
def drive(speed_rpm, n_direction): 
    # n_direction = clamp(n_direction, -1, 1) #Ensure that we dont get ridiculous turn values
    left_motor.set_velocity(speed_rpm - (n_direction),RPM )
    right_motor.set_velocity(speed_rpm + (n_direction), RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)
    # sleep(10) #10 millisecond cooldown
    # if controller.buttonA.pressed: 
    #     left_motor.stop(BrakeType.BRAKE)
    #     right_motor.stop(BrakeType.BRAKE)
    return

#lab.2 function 
def wallFollowInches(setDistanceFromwall): 
    kP = 10
    kI = 0
    kD = 0
    FeedForward = 0
    distanceFromWall_side = rangeFinderSide.distance(DistanceUnits.IN) #Inches
    distanceFromWall_front = rangeFinderFront.distance(DistanceUnits.IN) #Inches
    error_front = distanceFromWall_front - 8
    error_directional =  distanceFromWall_side - setDistanceFromwall #Actual - setpoint = negative(goes backward) when bot is too close, otherwise it goes towards the wall

    if distanceFromWall_front > 8 : 
        drive(75,-kP * error_directional) #this looks sus to me
    else : 
        drive(0,0) #stop first
        turnDegrees(90) # command based completion
    

#Function calls begin again here
# polygon(6,5)
# solveMaze()
wait(2000) # Wait time for the Sonar to catch up, and actually gives read values
wallFollowInches(11) # 11 Inches from the wall
# brain.screen.set_cursor(1,1)
# while True : 
#     brain.screen.print(rangeFinderSide.distance(DistanceUnits.IN))
#     brain.screen.set_cursor(1,1)
#     wait(1000)
#     brain.screen.clear_screen()


