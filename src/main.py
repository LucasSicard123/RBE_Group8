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
# ai_vision_12__Orange = Colordesc(1, 240, 70, 64, 10, 0.2)
ai_vision_12__Green = Colordesc(1, 9, 129, 55, 15, 0.4)
runArm = False

#Instantiations
brain.screen.print("Hello V5, Team 8")
left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)
rangeFinderFront = Sonar(brain.three_wire_port.g)
rangeFinderSide = Sonar(brain.three_wire_port.a)
left_line_sensor = Line(brain.three_wire_port.e)
right_line_sensor = Line(brain.three_wire_port.f)
brain_inertial = Inertial(Ports.PORT19)
arm_motor = Motor(Ports.PORT20, 18_1, False)
_button = Bumper(brain.three_wire_port.c)
ai_vision_12 = AiVision(Ports.PORT20, ai_vision_12__Green)


#########################################
#Vision Block
#########################################
# def DetectObject():
# # takes a snapshot and searches for SIG_3_RED_BALL
# # you’ll want to use the signature that you defined above
#     objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
# # print the coordinates of the center of the object
#     if (objects):
#         print('x:', ai_vision_12.largest_object().centerX, ' y:',
#         ai_vision_12.largest_object().centerY, ' width:',
#         ai_vision_12.largest_object().width)
#         brain.screen.print_at('x: ', ai_vision_12.largest_object().centerX, x = 50, y =
#         40)
#         brain.screen.print_at(' y:', ai_vision_12.largest_object().centerY, x = 150,
#         y = 40)
#         brain.screen.print_at(' width:', ai_vision_12.largest_object().width, x = 250,
#         y = 40)
        
#         wait(90)
#         brain.screen.clear_screen()



############################################

def invert():
    global runArm
    runArm = not runArm

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

#Helper function, this will be used often to validate input
def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

#! Inclusive bounds. 
def rangeCheck(value, min_Value, max_value):
    return value >= min_Value and value <= max_value

#Drive function
def drive(speed_rpm, n_direction): 
    # n_direction = clamp(n_direction, -1, 1) #Ensure that we dont get ridiculous turn values
    left_motor.set_velocity(speed_rpm - (n_direction * speed_rpm),RPM )
    right_motor.set_velocity(speed_rpm + (n_direction * speed_rpm), RPM)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)
    return

def wallFollowInches(setDistanceFromwall): 
    kP = 10.0 # must be float
    kI = 0
    kD = 0
    FeedForward = 0
    tolerance = 0.2
    distanceFromWall_side = rangeFinderSide.distance(DistanceUnits.IN) #Inches
    # error_front = distanceFromWall_front - 8 #Made obsolete(for now)
    error_directional = setDistanceFromwall - distanceFromWall_side #Actual - setpoint = negative(goes backward) when bot is too close, otherwise it goes towards the wall
    notComplete = True
    #Code for part 1: Dead reckoning (Requires a lot of 'if' conditions, and it pisses me off)
    turnCount = 0
    timeDiff = 0
    time.time()
    timeStart = 0
    #Code for part 2: Inertial sensors

    #Use #turnwithGyroInPlace(90)
    #Dead-reckoning version as of now
    while notComplete:
        if rangeFinderFront.distance(DistanceUnits.IN) >= 7 and turnCount == 0: 
            error_directional =  rangeFinderSide.distance(DistanceUnits.IN) - setDistanceFromwall #update
            error_directional = clamp(error_directional, -18, 18) #Set a limit on how far the wall can even be, otherwise we deal with ridiculous turn values
            drive(200, -kP * error_directional) #this looks sus to me
            timeStart = time.time()
            print(-kP * error_directional)
        else : 
            # drive(0, 0)
            if (turnCount == 1) and (timeDiff <= 14): #TODO: add proper values, measure distance from center point of the two trees to the closest front wall
                error_directional =  rangeFinderSide.distance(DistanceUnits.IN) - setDistanceFromwall #update
                error_directional = clamp(error_directional, -18, 18) #Same clamp as earlier
                drive(200, -kP * error_directional)
                timeDiff = time.time() - timeStart #Currenttime - TimeStart
                print(-kP * error_directional)
            else :
                if turnCount >= 2: 
                    stopMotors()
                    notComplete = False
                else :
                    drive(0,0) #stop first
                    turnDegrees(95) # command based completion
                    if turnCount == 1:
                        moveInches(10, 150)
                    turnCount += 1

def wallFollowInches_imu(setDistanceFromwall): 
    kP = 13 # must be float
    kI = 0
    kD = 0
    FeedForward = 0
    tolerance = 0.2
    distanceFromWall_side = rangeFinderSide.distance(DistanceUnits.IN) #Inches
    error_directional = setDistanceFromwall - distanceFromWall_side #Actual - setpoint = negative(goes backward) when bot is too close, otherwise it goes towards the wall
    notComplete = True
    turnCount = 0
    timeDiff = 0
    time.time()
    timeStart = 0
    while notComplete:
        if rangeFinderFront.distance(DistanceUnits.IN) >= 7 and turnCount == 0: 
            error_directional = setDistanceFromwall - rangeFinderSide.distance(DistanceUnits.IN)  #update
            error_directional = clamp(error_directional, -18, 18) #Set a limit on how far the wall can even be, otherwise we deal with ridiculous turn values
            drive(200, kP * error_directional) #this looks sus to me
            timeStart = time.time()
            print(kP * error_directional)
        else : 
            # drive(0, 0)
            if (turnCount == 1) and (timeDiff <= 13): #TODO: add proper values, measure distance from center point of the two trees to the closest front wall
                error_directional = setDistanceFromwall - rangeFinderSide.distance(DistanceUnits.IN)  #update
                error_directional = clamp(error_directional, -18, 18) #Same clamp as earlier
                drive(200, kP * error_directional)
                timeDiff = time.time() - timeStart #Currenttime - TimeStart
                print(kP * error_directional)
            else :
                if turnCount >= 2: 
                    stopMotors()
                    notComplete = False
                else :
                    drive(0,0) #stop first
                    imuturn(-90) # command based completion
                    if turnCount == 1:
                        moveInches(10, 150)
                    turnCount += 1


def wallFollowInches_line(setDistanceFromwall): 
    kP = 16.0 # must be float
    kI = 0
    kD = 0
    FeedForward = 0
    tolerance = 0.2
    distanceFromWall_side = rangeFinderSide.distance(DistanceUnits.IN) #Inches
    error_directional = setDistanceFromwall - distanceFromWall_side #Actual - setpoint = negative(goes backward) when bot is too close, otherwise it goes towards the wall
    notComplete = True
    turnCount = 0
    while notComplete:
        brain.screen.set_cursor(1,1)
        brain.screen.print(rangeFinderFront.distance(DistanceUnits.IN))
        brain.screen.set_cursor(2,1)
        brain.screen.print(rangeFinderSide.distance(DistanceUnits.IN))
        if turnCount == 0 and rangeFinderFront.distance(DistanceUnits.IN) >= 5: 
            error_directional = setDistanceFromwall - rangeFinderSide.distance(DistanceUnits.IN)  #update
            error_directional = clamp(error_directional, -18, 18) #Set a limit on how far the wall can even be, otherwise we deal with ridiculous turn values
            drive(150, kP * error_directional) #this looks sus to me
        elif rangeFinderFront.distance(DistanceUnits.IN) < 5 and turnCount == 0: 
            drive(0,0)
            imuturn(-90)
            wait(2000)
            turnCount += 1
            linefollow(100, 18, 40) # command based
            # wait(10000)
        elif turnCount == 1: 
            drive(0,0)
            imuturn(-90)
            turnCount += 1
        elif turnCount >= 2:
            drive(0,0) #stop first
            moveInches(15, 100)
            stopMotors() # command based completion
            notComplete = False #Completed task

def linefollow(speed, runtime, distance):
    starttime = time.time()
    run = True
    while run:
        if time.time() - starttime >= runtime:
            wait(1000) 
            if rangeFinderFront.distance(DistanceUnits.IN) <= distance:
                run = False
        lv = left_line_sensor.reflectivity()
        rv = right_line_sensor.reflectivity()
        # brain.screen.print("left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
        # brain.screen.set_cursor(1,1)
        # wait(100)
        # brain.screen.clear_screen()
        if speed < 0: 
            temp = lv 
            lv = rv
            rv = temp
        kp = 1.7
        if lv < 50 and rv < 50:
            drive(speed,0)
            brain.screen.clear_screen()
            brain.screen.print("going forward","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
        elif rv < 50:
            drive(speed,(rv*kp))
            brain.screen.clear_screen()
            brain.screen.print("turing left","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
        elif lv < 50:
            drive(speed,-(lv*kp))
            brain.screen.clear_screen()
            brain.screen.print("turing right","left:",left_line_sensor.reflectivity(),"right:",right_line_sensor.reflectivity())
    return True
    

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

def arm_move(n_degrees):
    kP = 1.0 #0.8
    error = n_degrees - arm_motor.position(RotationUnits.DEG) #setpoint - actual = positive, in the ccw for the arm. 
    destination = n_degrees

    while not runArm:
        if _button.pressing():
            invert() #Inverts runArm boolean
            brain.screen.set_cursor(2,1)
            brain.screen.print("PRESSED!!!! ", runArm)
            wait(1000) #Prevent repeated calls for invert()
            brain.screen.clear_screen()

    while True:
        if runArm and rangeCheck(arm_motor.position(RotationUnits.DEG), -100, 200):  
            error = destination - arm_motor.position(RotationUnits.DEG) #update
            # error = clamp(error, -100, 100) Clamp might have an issue
            brain.screen.set_cursor(1,1)
            brain.screen.print("Motor Torque = ", arm_motor.torque())
            brain.screen.set_cursor(3,1)
            brain.screen.print("Motor Current  = ",arm_motor.current())
            brain.screen.set_cursor(5,1)
            brain.screen.print("Motor Temperature = ", arm_motor.temperature())
            brain.screen.set_cursor(7,1)
            brain.screen.print("arm_motor.position = ", arm_motor.position(RotationUnits.DEG))
            #arm_motor.spin(FORWARD, kP * error, DPS) #Could also use RPM, but thats just semantics
            # brain.screen.clear_screen()
            if abs(error) <= 1:
                # arm_motor.spin(FORWARD, 0, DPS)
                arm_motor.stop(BrakeType.HOLD) #This is the predefined PID controlled positional hold. We should not technically be using this
                break
            else:
                arm_motor.spin(FORWARD, kP*error, DPS)

            if _button.pressing():
                invert() #Inverts runArm boolean
                brain.screen.set_cursor(2,1)
                brain.screen.print("PRESSED!!!! ", runArm)
                wait(1000) #Prevent repeated calls for invert()
                brain.screen.clear_screen()
        else:
            destination = 0
            invert()

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
            if rangeCheck(width, widthReq - tolerance, widthReq + tolerance,):
                if rangeCheck(turn, -0.1, 0.1): 
                    left_motor.spin(FORWARD, kP * turn, PERCENT)
                    right_motor.spin(REVERSE, kP * turn, PERCENT)
                # else:
                stopMotors()
            elif dist < 0:
                drive(dist * kPdrive, turn)
            elif dist > 0:
                drive(dist * kPdrive, -turn)

                
                # drive(dist * someSpeed, turn)
            # elif width > 35:
            #         # brain.screen.set_cursor(2,1)
            #         # brain.screen.print("turing left")
            #     left_motor.spin(REVERSE, 10, PERCENT)
            #     right_motor.spin(REVERSE, 10, PERCENT)
            # elif width < 25:
            #         # brain.screen.set_cursor(2,1)
            #         # brain.screen.print("turing right")
            #     # left_motor.spin(FORWARD, kP * error, PERCENT)
            #     # right_motor.spin(REVERSE, kP * error, PERCENT)
            #     left_motor.spin(FORWARD, 10, PERCENT)
            #     right_motor.spin(FORWARD, 10, PERCENT)
        else:
            left_motor.stop()
            right_motor.stop()
 
def followObj():
    kP = 0.3
    # print the coordinates of the center of the object
    while True:
        wait(90)
        objects = ai_vision_12.take_snapshot(ai_vision_12__Green)
        # error = 160 - ai_vision_12.largest_object().centerX
        # print(error)
        if (objects):
            error = 160 - ai_vision_12.largest_object().centerX
            width = ai_vision_12.largest_object().width
            print(error)
            if rangeCheck(error, -20, 20):
                stopMotors()
            elif error > 0 and (objects) == True:
                    # brain.screen.set_cursor(2,1)
                    # brain.screen.print("turing left")
                left_motor.spin(REVERSE, 10, PERCENT)
                right_motor.spin(FORWARD, 10, PERCENT)
            elif error < 0 and (objects) == True:
                    # brain.screen.set_cursor(2,1)
                    # brain.screen.print("turing right")
                # left_motor.spin(FORWARD, kP * error, PERCENT)
                # right_motor.spin(REVERSE, kP * error, PERCENT)
                left_motor.spin(REVERSE, 10, PERCENT)
                right_motor.spin(FORWARD, 10, PERCENT)
            if rangeCheck(width, 50, 20):
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
drivetoObj()
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