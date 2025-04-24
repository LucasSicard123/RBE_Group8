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
from drivetrainController import drivetrainController
from Auxilary import Auxilary

# Brain should be defined by default
brain = Brain()
controller = Controller(ControllerType.PRIMARY)


wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 5 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference
# ai_vision_12__Orange = Colordesc(1, 240, 70, 64, 10, 0.2)
ai_vision_12__Green = Colordesc(1, 9, 129, 55, 15, 0.4)
runArm = False

#Instantiations
brain.screen.print("Hello V5, Team 8")
left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)


#TODO: Change later when the robot is assembled. 
left_lift_motor = Motor(Ports.PORT3, 18_1, True)
right_lift_motor = Motor(Ports.PORT4, 18_1, False)


# rangeFinderFront = Sonar(brain.three_wire_port.g)
# rangeFinderSide = Sonar(brain.three_wire_port.a)
left_line_sensor = Line(brain.three_wire_port.e)
right_line_sensor = Line(brain.three_wire_port.f)
brain_inertial = Inertial(Ports.PORT19)
# arm_motor = Motor(Ports.PORT20, 18_1, False)
# _button = Bumper(brain.three_wire_port.c)
ai_vision_12 = AiVision(Ports.PORT20, ai_vision_12__Green)

drivetrain = drivetrainController(controller, brain, left_motor, right_motor, brain_inertial)

# left_motor.position(RotationUnits.DEG)
# left_motor.reset_position()

#!Turns Robot among rotational center n degrees COUNTER CLOCKWISE
def turnDegrees(n_degrees): 
    degreesToRotatePerRobotRotation = wheelTrack / wheelDiameter #Degrees to turn per robot rotational degree.
    toTurn = degreesToRotatePerRobotRotation * n_degrees * gearRatio
    left_motor.spin_for(REVERSE, toTurn, DEGREES, 60, RPM, False)
    right_motor.spin_for(FORWARD, toTurn, DEGREES, 60, RPM, True)

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

#Drive function
def drive(speed_rpm, n_direction): 
    # n_direction = clamp(n_direction, -1, 1) #Ensure that we dont get ridiculous turn values
    left_motor.set_velocity(speed_rpm - (n_direction * speed_rpm),RPM )
    right_motor.set_velocity(speed_rpm + (n_direction * speed_rpm), RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)
    return

# def linefollow(speed, runtime, distance):
#     starttime = time.time()
#     run = True
#     while run:
#         if time.time() - starttime >= runtime:
#             wait(1000) 
#             if rangeFinderFront.distance(DistanceUnits.IN) <= distance:
#                 run = False
#         lv = left_line_sensor.reflectivity()
#         rv = right_line_sensor.reflectivity()
#         # brain.screen.print("left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
#         # brain.screen.set_cursor(1,1)
#         # wait(100)
#         # brain.screen.clear_screen()
#         if speed < 0: 
#             temp = lv 
#             lv = rv
#             rv = temp
#         kp = 1.7
#         if lv < 50 and rv < 50:
#             drive(speed,0)
#             brain.screen.clear_screen()
#             brain.screen.print("going forward","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
#         elif rv < 50:
#             drive(speed,(rv*kp))
#             brain.screen.clear_screen()
#             brain.screen.print("turing left","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
#         elif lv < 50:
#             drive(speed,-(lv*kp))
#             brain.screen.clear_screen()
#             brain.screen.print("turing right","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
#     return True
    

# def imuStraight():
#     while True: 
#         drive(brain_inertial.)
    # return
def imuturn(n_degrees):
    n_degrees = brain_inertial.rotation() + n_degrees
    while True:
        # brain.screen.set_cursor(1,1)
        # brain.screen.print(brain_inertial.heading())
        # wait(100)
        if (n_degrees-9) <= brain_inertial.rotation() <= (n_degrees+9):
            brain.screen.set_cursor(2,1)
            brain.screen.print("done")
            left_motor.spin(REVERSE, 0, PERCENT)
            right_motor.spin(FORWARD, 0, PERCENT)
            return
        elif brain_inertial.rotation() > n_degrees:
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing left")
            left_motor.spin(REVERSE, 35, PERCENT)
            right_motor.spin(FORWARD, 35, PERCENT)
        elif brain_inertial.rotation() < n_degrees:
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing right")
            left_motor.spin(FORWARD, 35, PERCENT)
            right_motor.spin(REVERSE, 35, PERCENT)

def DetectObject():
    # takes a snapshot and searches for SIG_3_RED_BALL
    # you’ll want to use the signature that you defined above
    objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
    # print the coordinates of the center of the object
    if (objects):
        print('x:', ai_vision_12.largest_object().centerX, ' y:',
        ai_vision_12.largest_object().centerY, ' width:',
        ai_vision_12.largest_object().width)
        brain.screen.print_at('x: ', ai_vision_12.largest_object().centerX, x = 50, y =40)
        brain.screen.print_at(' y:', ai_vision_12.largest_object().centerY, x = 150,y = 40)
        brain.screen.print_at(' width:', ai_vision_12.largest_object().width, x = 250,y = 40)
        wait(90)
        brain.screen.clear_screen()

def turntoObj():
    objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
    kP = 0.3
    # print the coordinates of the center of the object
    while True:
        wait(90)
        objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
        # error = 160 - ai_vision_12.largest_object().centerX
        # print(error)
        if (objects):
            error = 160 - ai_vision_12.largest_object().centerX
            print((objects) == True)
            # if rangeCheck(error, -20, 20):
            #     stopMotors()
            # elif error > 0:
            #         # brain.screen.set_cursor(2,1)
            #         # brain.screen.print("turing left")
            #     left_motor.spin(REVERSE, kP * error, PERCENT)
            #     right_motor.spin(FORWARD, kP * error, PERCENT)
            # elif error < 0:
            #         # brain.screen.set_cursor(2,1)
            #         # brain.screen.print("turing right")
            #     # left_motor.spin(FORWARD, kP * error, PERCENT)
            #     # right_motor.spin(REVERSE, kP * error, PERCENT)
            #     left_motor.spin(REVERSE, kP * error, PERCENT)
            #     right_motor.spin(FORWARD, kP * error, PERCENT)

def drivetoObj():
    objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
    kP = 8.76
    kPdrive = 0.4
    brain.screen.clear_screen()
    widthReq = 120
    tolerance = 2
    # print the coordinates of the center of the object
    while True:
        wait(10)
        objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
        # error = 160 - ai_vision_12.largest_object().centerX
        # print(error)
        if (objects[0].exists):
            width = ai_vision_12.largest_object().width
            brain.screen.set_cursor(1,1)
            brain.screen.print(width)
            dist = widthReq - width
            turn = (ai_vision_12.largest_object().centerX - 160) / 160
            if Auxilary.rangeCheck(width, widthReq - tolerance, widthReq + tolerance,):
                if Auxilary.rangeCheck(turn, -0.1, 0.1): 
                    left_motor.spin(FORWARD, kP * turn, PERCENT)
                    right_motor.spin(REVERSE, kP * turn, PERCENT)
                else:
                    stopMotors()
            elif dist < 0:
                drive(dist * kPdrive, turn)
            elif dist > 0:
                drive(dist * kPdrive, -turn)
            left_motor.stop()
            right_motor.stop()
 
def followObj():
    kP = 0.3
    # print the coordinates of the center of the object
    while True:
        wait(10)
        objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
        if (objects):
            error = 160 - ai_vision_12.largest_object().centerX
            width = ai_vision_12.largest_object().width
            print(error)
            if Auxilary.rangeCheck(error, -20, 20):
                stopMotors()
            elif error > 0 and (objects) == True:
                left_motor.spin(REVERSE, 10, PERCENT)
                right_motor.spin(FORWARD, 10, PERCENT)
            elif error < 0 and (objects) == True:
                left_motor.spin(REVERSE, 10, PERCENT)
                right_motor.spin(FORWARD, 10, PERCENT)
            if Auxilary.rangeCheck(width, 50, 20):
                stopMotors()
            elif width > 25 and (objects) == True:
                    # brain.screen.set_cursor(2,1)
                    # brain.screen.print("turing left")
                left_motor.spin(REVERSE, 10, PERCENT)
                right_motor.spin(REVERSE, 10, PERCENT)
            elif width < 45 and (objects) == True:
                    # brain.screen.set_cursor(2,1)
                    # brain.screen.print("turing right")
                # left_motor.spin(FORWARD, kP * error, PERCENT)
                # right_motor.spin(REVERSE, kP * error, PERCENT)
                left_motor.spin(FORWARD, 10, PERCENT)
                right_motor.spin(FORWARD, 10, PERCENT)      
            

################################################################################
#Function calls begin here
# polygon(6,5)
# solveMaze()
# controller.buttonA.pressed(stopMotors) #Predefine a easy to press E-stop just in case. 
# controller.buttonB.pressed(arm_motor.stop(BrakeType.COAST))
brain_inertial.reset_rotation()
wait(100)


# drivetoObj()
# Wait time for the Sonar to catch up, and actually gives read values
# arm_motor.reset_position() #Zero the motor
# #alt
# arm_move(90)


# wallFollowInches(11) # 11 Inches from the wall
#wallFollowInches(8)
# wallFollowInches_imu(6)
# wallFollowInches_line(6)
# linefollow()
# imuturn(-90)


# _button.pressed(printstr)

#-204
#449
# brain.screen.set_cursor(1,1)
# while True : 
#     brain.screen.print(arm_motor.position())
#     brain.screen.set_cursor(1,1)
#     wait(1000)
#     brain.screen.clear_screen()