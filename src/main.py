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

import math 

class Auxilary:
    def __init__(self):
        return
    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))
    @staticmethod
    def rangeCheck(value, min_Value, max_value):
        return value >= min_Value and value <= max_value
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, process_variable, dt):
        # Calculate error
        error = self.setpoint - process_variable
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt #Delta-Error / dt (given)
        D_out = self.Kd * derivative
        
        # Compute total output
        output = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        
        return output
# Brain should be defined by default
brain = Brain()
controller = Controller(ControllerType.PRIMARY)

wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 30/12 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference
ai_vision_12__Green = Colordesc(3, 38, 248, 130, 10, 0.59) #Field
# ai_vision_12__Green = Colordesc(1, 44, 152, 82, 10, 0.4)
# ai_vision_12__Green = Colordesc(1, 104, 255, 227, 15, 0.4) #Table desk
# ai_vision_12__Orange = Colordesc(1, 239, 82, 76, 6, 0.2)
ai_vision_12__Orange = Colordesc(2, 247, 155, 136, 10, 0.35) #Field

# ai_vision_12__Yellow = Colordesc(2, 176, 164, 104, 12, 0.69) #Test val
ai_vision_12__Yellow = Colordesc(1, 249, 198, 118, 10, 0.49) #Field val

#Instantiations
brain.screen.print("Hello V5, Team 8")
left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT2, 18_1, False)

#TODO: Change later when the robot is assembled. 
right_lift_motor = Motor(Ports.PORT3, 18_1, False)

arm_motor = Motor(Ports.PORT8, 18_1, False) # This might change to a servo later. 

rangeFinderFront = Sonar(brain.three_wire_port.b)
brain_inertial = Inertial(Ports.PORT21)
ai_vision_12 = AiVision(Ports.PORT9, ai_vision_12__Green, ai_vision_12__Orange, ai_vision_12__Yellow)



def moveRPM(n_RPM, n_percentDirection): 
    left_motor.spin(FORWARD, Auxilary.clamp(n_RPM - n_percentDirection * 2 * n_RPM, -100, 100), RPM)
    right_motor.spin(FORWARD, Auxilary.clamp(n_RPM + n_percentDirection * 2 * n_RPM, -100, 100), RPM)
    return


#Command based method, that runs through first before letting any other command or function get called. 
def imuTurnFieldRelative(n_degrees):
    print("Curr rot : ", brain_inertial.heading())
    while True:
        if (n_degrees-2.5) <= (brain_inertial.heading()) <= (n_degrees+2.5):
            left_motor.spin(REVERSE, 0, PERCENT)
            right_motor.spin(FORWARD, 0, PERCENT)
            return
        elif n_degrees > 0: #brain_inertial.rotation() > n_degrees (before)
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing left")
            left_motor.spin(REVERSE, 15, PERCENT)
            right_motor.spin(FORWARD, 15, PERCENT)
        elif n_degrees < 0:
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing right")
            left_motor.spin(FORWARD, 15, PERCENT)
            right_motor.spin(REVERSE, 15, PERCENT)
        elif n_degrees == 0: 
            if -1 <= brain_inertial.heading() <= 1: 
                left_motor.spin(REVERSE, 0, PERCENT)
                right_motor.spin(FORWARD, 0, PERCENT)
                return
            elif 180 - brain_inertial.heading() > 0: #brain_inertial.rotation() > n_degrees (before)
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing left")
                left_motor.spin(REVERSE, 15, PERCENT)
                right_motor.spin(FORWARD, 15, PERCENT)
            elif 180 - brain_inertial.heading() < 0:
            # brain.screen.set_cursor(2,1)
            # brain.screen.print("turing right")
                left_motor.spin(FORWARD, 15, PERCENT)
                right_motor.spin(REVERSE, 15, PERCENT)


def imuturn(n_degrees):
    n_degrees = (brain_inertial.rotation() + n_degrees) % 360
    # if(n_degrees < 0): 
    #     n_degrees = 360 - n_degrees
    while True:
        if (n_degrees-2.5) <= (brain_inertial.rotation() % 360) <= (n_degrees+2.5):
            left_motor.spin(REVERSE, 0, PERCENT)
            right_motor.spin(FORWARD, 0, PERCENT)
            return
        elif n_degrees > 0: #brain_inertial.rotation() > n_degrees
            # brain.screen.print("turing left")
            left_motor.spin(REVERSE, 15, PERCENT)
            right_motor.spin(FORWARD, 15, PERCENT)
        elif n_degrees <= 0:
            # brain.screen.print("turing right")
            left_motor.spin(FORWARD, 15, PERCENT)
            right_motor.spin(REVERSE, 15, PERCENT)


def straightDrivePeriodic(n_speedRPM, rotation): 
    distance = 0
    kP = 4
    error = rotation - (brain_inertial.heading())
    # error = Auxilary.clamp(error, -25, 25)
    if brain_inertial.heading() > 180 and rotation < 180: 
        error = rotation - (360 - brain_inertial.heading())
        print("Exceeded expect value range : ", error)
    elif rotation < 180 and brain_inertial.heading() < 180: 
        error = brain_inertial.heading() - rotation
    # print("Error: ", error)
    # print("Heading : ", brain_inertial.heading())

    error = Auxilary.clamp(error, -8, 8)


    output = kP * error
    if(Auxilary.rangeCheck(output, -1.8, 1.8)):
        output = 0
    
    # if(rotation < 180): 
    # output *= -1

    if(rotation >= 180): 
        output *= -1

    # if(output > 0): 
    #     print("Turning left")
    # if(output < 0): 
    #     print("Turning right")
    
    # error = brain_inertial.rotation(RotationUnits.DEG) - rotation
    left_motor.spin(FORWARD, n_speedRPM - (output), RPM) 
    right_motor.spin(FORWARD, n_speedRPM + (output), RPM)

def driveUntilWall(n_inches, rotation): 
    notFinished = True
    # imuTurnFieldRelative(0)
    drive(0,0)
    while notFinished:
        distance = rangeFinderFront.distance(DistanceUnits.IN)
        if(distance <= n_inches): 
            imuTurnFieldRelative(rotation)
            notFinished = False
            stopMotors()
        else: 
            straightDrivePeriodic(60, rotation)

#!Moves n(integer) Inches forwards @ speed_rpm. 
def moveInches(n_inches, speed_rpm): 
    direction = DirectionType.FORWARD
    if(n_inches < 0): 
        direction = DirectionType.REVERSE
    degreesToTravel = n_inches * degreesPerInch # 1 : 1
    degreesToRotate = degreesToTravel * gearRatio #5 : 1, now it is gear ratio compliant
    #terribly done parallel-command group

    left_motor.spin_for(direction, degreesToRotate, DEGREES, speed_rpm, RPM, False) 
    right_motor.spin_for(direction, degreesToRotate, DEGREES, speed_rpm, RPM, True)


def moveInchesDirection(n_inches, speed_rpm, n_direction): 
    degreesToTravel = n_inches * degreesPerInch # 1 : 1
    degreesToRotate = degreesToTravel * gearRatio #5 : 1, now it is gear ratio compliant
    #terribly done parallel-command group
    left_motor.spin_for(FORWARD, degreesToRotate - (1  * n_direction), DEGREES, speed_rpm, RPM, False) 
    right_motor.spin_for(FORWARD, degreesToRotate + (1  * n_direction), DEGREES , speed_rpm, RPM, True)

def climbRamp(): 
    kP = 1/25
    # print(brain_inertial.orientation(OrientationType.PITCH))
    yawError = brain_inertial.heading() - 0
    wait(1000) #TODO: Remove this if deemed unnecessary, I added this for debugging purposes. 
    while not(Auxilary.rangeCheck(brain_inertial.orientation(OrientationType.PITCH), -1, 1)): 
        #  print(brain_inertial.orientation(OrientationType.PITCH))
        if Auxilary.rangeCheck(brain_inertial.orientation(OrientationType.PITCH), -1, 1):
            stopMotors()
            print("reached top")
            break
        else: 
            yawError = brain_inertial.heading() - 0
            # print(yawError)
            #Check inversions, it all hinges on whether or not the gyroscope is Clockwise positive or CCW positive. 
            # left_motor.spin(FORWARD, 25 - (yawError * kP), RPM)
            # right_motor.spin(FORWARD, 25 + (yawError * kP), RPM)
            # print("Left Motor : ", left_motor.current(CurrentUnits.AMP), "\n Right Motor : ", right_motor.current(CurrentUnits.AMP))
            straightDrivePeriodic(50, 0) #If the climb starts peaking, refer to this. 
    stopMotors()
    print("reached top")


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
    print("Left Motor : ", left_motor.current(CurrentUnits.AMP), "\n Right Motor : ", right_motor.current(CurrentUnits.AMP))

    return
    
def DetectObject():
    # takes a snapshot and searches for SIG_3_RED_BALL
    # you’ll want to use the signature that you defined above
    objects = ai_vision_12.take_snapshot(ai_vision_12__Yellow)
    # print the coordinates of the center of the object
    if (objects):
        print('x:', ai_vision_12.largest_object().centerX, ' y:',
        ai_vision_12.largest_object().centerY, ' width:',
        ai_vision_12.largest_object().width, 'height: ', ai_vision_12.largest_object().height)
        print("Angle", ai_vision_12.largest_object().angle)
        brain.screen.print_at('x: ', ai_vision_12.largest_object().centerX, x = 50, y =40)
        brain.screen.print_at(' y:', ai_vision_12.largest_object().centerY, x = 150,y = 40)
        brain.screen.print_at(' width:', ai_vision_12.largest_object().width, x = 250,y = 40)
        wait(90)
        brain.screen.clear_screen()


def movePercent(n_percent, n_percentDirection): 
    print(n_percent - 2 * n_percentDirection)
    if(n_percent - 2 * n_percentDirection == 0): 
        return
    left_motor.spin(FORWARD, n_percent - (2 * n_percentDirection), RPM)
    right_motor.spin(FORWARD, n_percent + (2 * n_percentDirection), RPM)
    return

def closeClaw():
    arm_motor.spin_to_position(-270, DEGREES, 25, RPM, False)
    notFinished = True
    prevPosition = arm_motor.position(RotationUnits.DEG)
    wait(100)
    while notFinished: 
        # print("Close claw : ", arm_motor.current(CurrentUnits.AMP))
        if(arm_motor.velocity(VelocityUnits.RPM) == 0):
            wait(1000)
            arm_motor.stop(BrakeType.HOLD)
            # print("furnished")
            notFinished = False
            continue
    return

def openClaw():
    arm_motor.spin_to_position(100, DEGREES, 25, RPM, False)
    notFinished = True
    wait(100)
    while notFinished: 
        # print("Open claw", arm_motor.current(CurrentUnits.AMP))
        if(arm_motor.velocity(VelocityUnits.RPM) == 0):
            wait(1000)
            arm_motor.stop(BrakeType.HOLD)
            # print("furnished")
            notFinished = False
            continue
    return 

def lift(height):
    pitchDiameter = 0.5 #In inches
    pitchCircumference = pitchDiameter * math.pi
    PID = PIDController(8,0.01,0.001,0)
    kP = 10
    finished = False
    distance = 0
    if(height > 14): 
        print("Invalid target height")
        return
    currHeight = math.sin(math.acos((9.5 - 0)/(9.5))) * 9.5 * 2
    # self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
    tolerance = 0.1 #In inches
    # print(currHeight)
    if(right_lift_motor.position(RotationUnits.REV) >= 2.47): 
        right_lift_motor.stop(BrakeType.BRAKE)
        print("Scissor limit reached")

        return #Limit reached.
    while not finished: 
        distance = right_lift_motor.position(RotationUnits.DEG)/360 * (pitchCircumference)
        if(distance < 0): 
            distance = 0
        currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
        # currHeight = height
        error = height - currHeight 
        PID.setpoint = height
        pidOutput = PID.compute(currHeight, 0.01) #Vex motors calculate every 10ms
        if Auxilary.rangeCheck(currHeight, height - tolerance, height + tolerance): 
            right_lift_motor.stop(BrakeType.BRAKE)
            print("Finished : " + str(currHeight))
            finished = True
        else: 
            # right_lift_motor.spin(FORWARD, kP * error, PERCENT)
            right_lift_motor.spin(FORWARD, pidOutput, PERCENT)
def liftInches(heightToAdd):
    pitchDiameter = 0.5 #In inches
    pitchCircumference = pitchDiameter * math.pi
    PID = PIDController(8,0.01,0.001,0)
    kP = 10
    finished = False
    distance = 0
    distance = right_lift_motor.position(RotationUnits.DEG)/360 * (pitchCircumference)
    if(distance < 0): 
        distance = 0
    currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
    height = currHeight + heightToAdd
    print(height)
    # self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
    tolerance = 0.1 #In inches
    # print(currHeight)
    if(right_lift_motor.position(RotationUnits.REV) >= 2.47): 
        right_lift_motor.stop(BrakeType.BRAKE)
        print("Scissor limit reached")

        return #Limit reached.
    while not finished: 
        distance = right_lift_motor.position(RotationUnits.DEG)/360 * (pitchCircumference) + 0.01
        if(distance < 0): 
            distance = 0
        currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
        error = height - currHeight
        PID.setpoint = height
        pidOutput = PID.compute(currHeight, 0.01) #Vex motors calculate every 10ms, this is simply for 
        # print(pidOutput)
        if Auxilary.rangeCheck(error, -tolerance, tolerance): 
            right_lift_motor.stop(BrakeType.HOLD)
            print("Finished" + str(currHeight))
            # print(currHeight)
            finished = True
        else: 
            a = 1
            # right_lift_motor.spin(FORWARD, -kP * error, PERCENT)
            right_lift_motor.spin(FORWARD, pidOutput, PERCENT)

def liftPeriodic(Color:Colordesc):
    pitchDiameter = 0.5 #In inches
    pitchCircumference = pitchDiameter * math.pi
    PID = PIDController(0.15,0.01,0.001,0)
    kP = 0.7
    objects = ai_vision_12.take_snapshot(Color)
    # print the coordinates of the center of the object
    if (objects):
        tolerance = 0.1 #In inches
        if(right_lift_motor.position(RotationUnits.REV) >= 2.47): 
            right_lift_motor.stop(BrakeType.BRAKE)
            print("Scissor limit reached")
            return #Limit reached. 
        error =  ai_vision_12.largest_object().centerY - 125
        PID.setpoint = 0
        output = -kP * error
        if  Auxilary.rangeCheck(right_lift_motor.position(RotationUnits.REV), -100000, 0.1)  and (output) < 0: 
            right_lift_motor.stop(BrakeType.BRAKE)
            print(output)
            print("Limit reached")
            return False
        if Auxilary.rangeCheck(error, -tolerance, tolerance):
            right_lift_motor.stop(BrakeType.HOLD)
            return True
        else: 
            right_lift_motor.spin(FORWARD, output, RPM)
    else: 
        right_lift_motor.stop(BrakeType.HOLD)

# def approachFruit(color: Colordesc):
#     finished = False
#     kPHorizontal = 0.22 # 1/5th of the actual error to be accomodated at a time.
#     currHeight = 1 # For keeping track of scissor lift. Should be set to 0 before calling.
#     height = 0
#     while not finished:
#         wait(10)
#         objects = ai_vision_12.take_snapshot(color)
#         wait(100)
#         if (objects):
#             height = ai_vision_12.largest_object().height
#             if (height < 130):
#                 horizontalError = (180 - ai_vision_12.largest_object().centerX) / 180
#                 moveRPM(40, horizontalError * kPHorizontal)
#                 height = ai_vision_12.largest_object().height
#                 verticalError = (180 - ai_vision_12.largest_object().centerY) / 180
#                 print("Vert Error: ", verticalError)
#                 lift(currHeight + verticalError * 1.8) 
#                 currHeight = currHeight + verticalError * 1.8
#                 print("Current Height: ", currHeight)
#             else:
#                 stopMotors()
#                 finished = True
#     lift(currHeight+1)

def Autodrive():
        Notfinished = True
        innerLoop = True
        i = 1
        moveInches(8, 40) #Climb onto the start of the ramp. 
        climbRamp() #Climb the ramp and then stop
        moveInches(5, 40) #Move off the ramp
        driveUntilWall(3, 0) #driveUntilWall(InchesToWall:DOUBLE, Heading_in_degrees:DOUBLE)
        while Notfinished:
            currentColor = ai_vision_12__Green #Default value. 
            if(i == 1):
                print("On Green")
                currentColor = ai_vision_12__Green
            if(i == 2):
                print("On Yellow")
                currentColor = ai_vision_12__Yellow
            if(i == 3):
                print("On Orange")
                currentColor = ai_vision_12__Orange
            # print("Starting loop")
            imuturn(-90.25)
            if(i == 1):
                # moveInches(50, 50)
                moveInches(44, 50)
            elif(i == 2):
                moveInches(41.5, 50)
            elif(i == 3): 
                driveUntilWall(10, 270)

            imuTurnFieldRelative(270)
            wait(2000)
            imuturn(-90.25)
            # moveInches(, 50)
            # print("Starting loop")
            wait(1000) # wait 1 second, make sure we are balanced 
            print("Aligned with fruit tree line")
            # wait(3000)
            print("Current rotation : ", brain_inertial.rotation())

            
            # #Accumulate distance

            left_motor.reset_position()
            right_motor.reset_position()

            imuturn(-25)

            # ####################################################
            #Fruit grabbing sequence. 
            findfruit(currentColor)
            openClaw()
            Notcompleted = True
            # if isShortFruit(): widthQ =  
            while not(fruitReached(currentColor, 230, 300)) and Notcompleted: 
                horizontalError = (160 - ai_vision_12.largest_object().centerX) / 160
                moveRPM(12.5, horizontalError * 0.8)
                liftPeriodic(currentColor)
                DetectObject()
            right_lift_motor.stop(BrakeType.HOLD)
            wait(100)
            Notcompleted = False
            liftInches(1.5)
            moveInches(1.3, 15)
            stopMotors()
            print("Grabbing fruit 1")
            right_lift_motor.stop(BrakeType.BRAKE)
            closeClaw()
            lift(2)
            #End of fruit grabbing sequence
            # #####################################################

            
            # #Considerable delay here because of the calculations.
            # # # moveInches(10, 20)
            leftRev = left_motor.position(RotationUnits.DEG)
            rightRev = right_motor.position(RotationUnits.DEG)
            print(leftRev)
            print(rightRev)
            left_motor.spin_for(REVERSE, leftRev, DEGREES, 15, RPM,  False)
            right_motor.spin_for(REVERSE, rightRev, DEGREES, 15, RPM, True)

            # #End of reversal sequence. 
            

            # # #First Deposit
            # # imuTurnFieldRelative(180)
            driveUntilWall(3, 180)
            # #The hill
            # if(i != 3): 
            #     moveInches(30, 50)
            #     imuTurnFieldRelative(180)
            #     driveUntilWall(3, 180)
            # # imuTurnFieldRelative(180)
            openClaw()
            
            # moveInches(-10, 50)

            # #Second grab 
            # imuTurnFieldRelative(0)
            # imuTurnFieldRelative(90)
            # moveInches(, 50)

            # #Second backtrack start


            # left_motor.reset_position()
            # right_motor.reset_position()

            # ####################################################
            # #Fruit grabbing sequence 2. 
            # findfruit(currentColor)
            # openClaw()
            # Notcompleted = True
            # height = 230
            # while not(fruitReached(currentColor, height, 300)) and Notcompleted: 
            #     horizontalError = (160 - ai_vision_12.largest_object().centerX) / 160
            #     moveRPM(10, horizontalError * 0.8)
            #     liftPeriodic(currentColor)
            #     DetectObject()
            #     # if(fruitReached())
            # Notcompleted = False
            # right_lift_motor.stop(BrakeType.HOLD)
            # wait(100)
            # liftInches(1.5)
            # moveInches(1.4, 15)
            # stopMotors()
            # print("Grabbing Fruit 2")
            # right_lift_motor.stop(BrakeType.BRAKE)
            # closeClaw()
            # lift(2)
            # #End of fruit grabbing sequence 2
            # #####################################################

            # leftRev = left_motor.position(RotationUnits.DEG)
            # rightRev = right_motor.position(RotationUnits.DEG)
            # print(leftRev)
            # print(rightRev)
            # left_motor.spin_for(REVERSE, leftRev, DEGREES, 15, RPM,  False)
            # right_motor.spin_for(REVERSE, rightRev, DEGREES, 15, RPM, True)


            # # #Second backtrack end

            # # #Go back to basket and release
            # imuTurnFieldRelative(180)
            # driveUntilWall(3, 180)
            # openClaw()

            # #Move onto next tree.  
            
            imuTurnFieldRelative(0)
            driveUntilWall(3, 0)
            #Climb hill
            # if(i != 3): 
            #     moveInches(30, 50)
            #     imuTurnFieldRelative(0)
            #     driveUntilWall(3, 0)
            # imuturn(90)
            # moveInches(20, 25)
            i += 1
            if(i >= 4): 
                Notfinished = False #First cycle. You may change this later to allow for repeats. 
                break
        return



def fruitReached(color:Colordesc, heightReq, widthReq ):
    objects = ai_vision_12.take_snapshot(color, 8) #I put a maximum of 8 objects
    
    # print the coordinates of the center of the object
    lastheight = 0
    queryObj = ai_vision_12.largest_object() #default
    if (objects): #Condition : it has at least ONE object (truthy)
        #Next: Filter for largest height, ignoring largest area. 
        try:
            for obj in objects: 
                if(obj.height > lastheight): 
                    queryObj = obj
                    lastheight = obj.height
                    # print("a : ", lastheight)    
        except: 
            print("Problem occured, defaulting to largest object")

        if(queryObj.height >= heightReq):
            print("Height requiement met")
            return True
        if((queryObj.width >= widthReq)):
            print("Width requirement met")
            return True
        else:
            return False
    else:
        return False #No objects found, condition unmet.

def findfruit(color):
    objects = ai_vision_12.take_snapshot(color)
    height = 0 #No fruits below 5 inches, no point in starting to low so we start at 5 inches
    notDone = True
    while notDone: 
        objects = ai_vision_12.take_snapshot(color)
        Xerror = 0
        if(not(objects)): 
            height += 1
            lift(height)
            print("No objects, lifting...")
        if(objects): 
            queryObj = ai_vision_12.largest_object() #Default case
            print("Found objects, lifting to find closer")
            lastheight = 0
            #Filter for object with largest height, if it fails by any means, it shall default to the instantiation above
            try:
                for obj in objects: 
                    if(obj.height > lastheight): 
                        queryObj = obj
                        lastheight = obj.height
                        print(lastheight)    
            except: 
                print("Problem occured, defaulting to largest object")
            if(queryObj.height > 50): 
                Xerror = (160 - queryObj.centerX)/160
            if (queryObj.height > 78): 
                print("Found fruit")
                notDone = False #End function
                break #End function 2(redundant but is better)
            height += 3
            lift(height)
        #Reset height and move forward
        if(height > 12): 
            height = 0
            moveInchesDirection(3, 20, Xerror)
    print("Finished finding fruit")
    right_lift_motor.stop(BrakeType.BRAKE)




     

            
#Command based, needs completion before resuming with the stack 
def findTorqueToMovement(Motor1):
    finished = False
    lasttorque = 0
    lastPosition = Motor1.position(RotationUnits.DEG)
    speed = 0; 
    Motor1.spin_for(FORWARD, 60, DEGREES, False) #Ignore completing this command, and move on to the loop.
    while not(finished): 
        lasttorque = Motor1.torque(TorqueUnits.INLB)

        if Motor1.position(RotationUnits.DEG) - lastPosition >= 6:
            print("Last torque : " + str(lasttorque))
            print("Most current : " + str(Motor1.current(CurrentUnits.AMP)))
            Motor1.stop(BrakeType.BRAKE)
            # brain.screen.print_at(1,1,lasttorque)
            finished = True
        # Motor1.spin(FORWARD, speed, RPM)
        # speed+=1

# ai_vision_12.largest_object().


################################################################################
#Function calls begin here
# brain_inertial.reset_rotation()
# brain_inertial.reset_heading()
brain_inertial.calibrate()
right_lift_motor.reset_position()
arm_motor.reset_position()
brain_inertial.reset_rotation()
brain_inertial.reset_heading()
wait(3000)
print("calibration finished")
brain.screen.print("Calibration finished")
wait(10000)
print("Starting autodrive")
brain.screen.clear_screen()
brain.screen.print("Starting autodrive")
# imuTurnFieldRelative(180)
Autodrive()
# while True:
#     DetectObject()

# lift(12)
# imuTurnFieldRelative(90)
wait(2000)
# climbRamp()
print("Commands finished")
brain.program_stop()
# Autodrive()