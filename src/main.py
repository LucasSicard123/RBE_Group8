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
# gyro = Gyro(brain.three_wire_port.c) #For later (Gyroscope)
# inertial = Inertial(brain.three_wire_port.d) #For later (Inertial Sensor)

# DriveTraincontroller = drivetrainController(controller, brain, left_motor, right_motor) #experimental as hell 

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

def stopMotors():
    left_motor.stop(BrakeType.BRAKE)
    right_motor.stop(BrakeType.BRAKE)

#Helper function, this will be used often to validate input
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

#! Inclusive bounds. 
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
    return

#lab.2 function 
def wallFollowInches(setDistanceFromwall): 
    kP = 2
    kI = 0
    kD = 0
    FeedForward = 0
    tolerance = 0.2
    distanceFromWall_side = rangeFinderSide.distance(DistanceUnits.IN) #Inches
    distanceFromWall_front = rangeFinderFront.distance(DistanceUnits.IN) #Inches
    # error_front = distanceFromWall_front - 8 #Made obsolete(for now)
    error_directional =   setDistanceFromwall - distanceFromWall_side #Actual - setpoint = negative(goes backward) when bot is too close, otherwise it goes towards the wall
    notComplete = True
    #Code for part 1: Dead reckoning (Requires a lot of 'if' conditions, and it pisses me off)
    turnCount = 0
    


    #Code for part 2: Inertial sensors

    #Use #turnwithGyroInPlace(90)


    #Dead-reckoning version as of now
    while notComplete:
        if not(rangeCheck(distanceFromWall_front, 8-tolerance, 8+tolerance)) and turnCount < 1: 
            error_directional =  distanceFromWall_side - setDistanceFromwall #update
            error_directional = clamp(error_directional, -11, 50) #Set a limit on how far the wall can even be, otherwise we deal with ridiculous turn values

            # drive(75, kP * error_directional) #this looks sus to me
            brain.screen.print(error_directional)
            print(str(error_directional) + "\n")
            wait(100)
            brain.screen.set_cursor(1,1)
            brain.screen.clear_screen()
        else : 
            if turnCount == 1 and not(rangeCheck(distanceFromWall_front, 0 - tolerance, 0+tolerance)): #TODO: add proper values, measure distance from center point of the two trees to the closest front wall
                error_directional =  distanceFromWall_side - setDistanceFromwall #update
                error_directional = clamp(error_directional, -11, 50) #Same clamp as earlier
                drive(75, kP * error_directional)
                wait(10) 
            else :
                drive(0,0) #stop first
                wait(10)
                turnDegrees(90) # command based completion
                turnCount += 1
                if turnCount >= 2: 
                    stopMotors()
                    notComplete = False

#! should be used with precision, setPoint is in Degrees(radians requires too many PI calls, and I am too lazy)
# def turnWithGyroInPlace(setPoint):
#     setPoint %= 360
#     reachedSetpoint = False

#     kP = 1/360 #To be determined more accurately, this is a guess
#     kI = 0
#     kD = 0
#     error = setPoint - inertial.orientation(OrientationType.YAW)

#     tolerance = 3
#     while not reachedSetpoint: 
#         left_motor.set_velocity((kP * error),RPM )
#         right_motor.set_velocity((kP * error), RPM)
#         left_motor.spin(FORWARD)
#         right_motor.spin(FORWARD)
#         reachedSetpoint = rangeCheck(error, -3, 3)

#     stopMotors()
#     return 

################################################################################
#Function calls begin here
# polygon(6,5)
# solveMaze()
controller.buttonA.pressed(stopMotors) #Predefine a easy to press E-stop just in case. 
wait(2000) # Wait time for the Sonar to catch up, and actually gives read values
wallFollowInches(11) # 11 Inches from the wall


# brain.screen.set_cursor(1,1)
# while True : 
#     brain.screen.print(rangeFinderSide.distance(DistanceUnits.IN))
#     brain.screen.set_cursor(1,1)
#     wait(1000)
#     brain.screen.clear_screen()


