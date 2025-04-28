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
# from drivetrainController import DrivetrainController
# from Auxilary import Auxilary, PIDController
# from Mechanisms import Lift
# from Mechanisms import Arm

import math 

#TODO: Math seems find but might be wrong. tbd
#Should work in theory, just not tested yet. 
def moveRPM(self, n_RPM, n_percentDirection): 
    self.left_motor.spin(FORWARD, Auxilary.clamp(n_RPM - n_percentDirection * 2 * n_RPM, -100, 100), RPM)
    self.right_motor.spin(FORWARD, Auxilary.clamp(n_RPM + n_percentDirection * 2 * n_RPM, -100, 100), RPM)
    return


#Command based method, that runs through first before letting any other command or function get called. 
def imuTurnFieldRelative(self, n_heading):
    heading = self.inertial.heading()
    toTurn = True
    tolerance = 3 #TODO: Change for later, after testing. 
    while toTurn: 
        heading = self.inertial.heading()
        if Auxilary.rangeCheck(heading, n_heading - 3, n_heading + 3): 
            self.stopMotors()
            toTurn = False #End function
        if heading < n_heading: 
            self.moveRPM(35, 1)
        if heading > n_heading: 
            self.moveRPM(35, -1)

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

color_Green = Colordesc(1, 9, 129, 55, 15, 0.4) # Green
color_Orange =Colordesc(1, 240, 70, 64, 10, 0.2) # Orange


wheelTrack = 11 #in inches
wheelDiameter = 4 #in inches
gearRatio = 5 #5 : 1 // 60 : 12
wheelCircumference = 3.14 * wheelDiameter
degreesPerInch = 360.0 / wheelCircumference
ai_vision_12__Green = Colordesc(1, 9, 129, 55, 15, 0.4)
ai_vision_12__orange = Colordesc(2, 244, 147, 92, 4, 0.2)
ai_vision_12__yellow = Colordesc(1, 243, 215, 102, 10, 0.2)

runArm = False

#Instantiations
brain.screen.print("Hello V5, Team 8")
left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)


#TODO: Change later when the robot is assembled. 
left_lift_motor = Motor(Ports.PORT3, 18_1, False)
# right_lift_motor = Motor(Ports.PORT4, 18_1, False)

# arm_motor = Motor(Ports.PORT5, 18_1, True) # This might change to a servo later. 

# rangeFinderFront = Sonar(brain.three_wire_port.g)
# rangeFinderSide = Sonar(brain.three_wire_port.a)
# left_line_sensor = Line(brain.three_wire_port.e)
# right_line_sensor = Line(brain.three_wire_port.f)
brain_inertial = Inertial(Ports.PORT21)
# arm_motor = Motor(Ports.PORT20, 18_1, False)
# _button = Bumper(brain.three_wire_port.c)
ai_vision_12 = AiVision(Ports.PORT10, ai_vision_12__Green)

left_lift_motor.reset_position()
# right_lift_motor.reset_position()


#!Moves n(integer) Inches forwards @ speed_rpm. 
def moveInches(n_inches, speed_rpm): 
    degreesToTravel = n_inches * degreesPerInch # 1 : 1
    degreesToRotate = degreesToTravel * gearRatio #5 : 1, now it is gear ratio compliant
    #terribly done parallel-command group
    left_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, False) 
    right_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, True)

def climbRamp(): 
        kP = 1/25
        print(brain_inertial.orientation(OrientationType.PITCH))
        while True: 
            #  print(brain_inertial.orientation(OrientationType.PITCH))
            if Auxilary.rangeCheck(brain_inertial.orientation(OrientationType.PITCH), -1, 1):
                stopMotors()
                print("reached top")
                break
            else: 
                yawError = brain_inertial.heading() - 0
                print(yawError)
                #Check inversions, it all hinges on whether or not the gyroscope is Clockwise positive or CCW positive. 
                left_motor.spin(FORWARD, 25 - (yawError * kP), RPM)
                right_motor.spin(FORWARD, 25 + (yawError * kP), RPM)


# def movePercent(n_percent, speed_rpm): 
#     left_motor.spin(FORWARD, n_percent, PERCENT)
#     right_motor.spin(FORWARD, n_percent, PERCENT)

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


def movePercent(n_percent, n_percentDirection): 
    print(n_percent - 2 * n_percentDirection)
    if(n_percent - 2 * n_percentDirection == 0): 
        return
    left_motor.spin(FORWARD, n_percent - (2 * n_percentDirection), RPM)
    right_motor.spin(FORWARD, n_percent + (2 * n_percentDirection), RPM)
    return

def driveTeleOp(): 
    toDrive = True
    while toDrive: 
        #Normalized values [-100,100]
        drive_r = -float(controller.axis1.position()) #rotational 
        drive_f = -float(controller.axis3.position()) #speed control
        if Auxilary.rangeCheck(drive_r, -5.5, 5.5):
            drive_r = 0
        if Auxilary.rangeCheck(drive_f, -5.5, 5.5): 
            drive_f = 0
        movePercent(drive_f, drive_r)
        # if controller.buttonA.pressing :
        #     print("pressed")
        #     stopMotors()
        #     # toDrive = False

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

def lift(height):
        pitchDiameter = 0.5 #In inches
        pitchCircumference = pitchDiameter * math.pi
        PID = PIDController(8,0.01,0.001,0)
        kP = 10
        finished = False
        distance = 0
        currHeight = math.sin(math.acos((9.5 - 0)/(9.5))) * 9.5 * 2
        # self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
        tolerance = 0.1 #In inches
        # print(currHeight)
        if(left_lift_motor.position(RotationUnits.REV) >= 2.47): 
            left_lift_motor.stop(BrakeType.BRAKE)
            print("Scissor limit reached")

            return #Limit reached.
        while not finished: 
            distance = left_lift_motor.position(RotationUnits.DEG)/360 * (pitchCircumference)
            # print(left_lift_motor.position(RotationUnits.DEG))
            currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
            # currHeight = height
            error = height - currHeight #When positive, motors should move clockwise(forwards)[it is inverted for one of the two motors]
            #Otherwise, the current height is greater than setpoint height, hence the error is negative, in this case the pinions should move backwards.
            #TODO: Make sure this happens and resolve any inversion issues.
            PID.setpoint = height
            # print("current height : " + str(currHeight))
            # print("error : " + str(error))

            pidOutput = PID.compute(currHeight, 0.01) #Vex motors calculate every 10ms, this is simply for 
            # lasttime = time.time()
            # print(pidOutput)
            if Auxilary.rangeCheck(currHeight, height - tolerance, height + tolerance): 
                left_lift_motor.stop(BrakeType.BRAKE)
                print("Finished" + str(currHeight))
                # print(currHeight)
                finished = True
            else: 
                a = 1
                # left_lift_motor.spin(FORWARD, kP * error, PERCENT)
                left_lift_motor.spin(FORWARD, pidOutput, PERCENT)
                # self.right_motor.spin(FORWARD, pidOutput, PERCENT)

def approachFruit(drivetrain, lift, speed_RPM):
    finished = False
    kPHorizontal = 0.2 # 1/5th of the actual error to be accomodated at a time.
    # color = colorStack.get()
    color= 0
    while not finished: 
        objects = ai_vision_12.take_snapshot(color)
        if(objects):
             height = ai_vision_12.largest_object().angle
             horizontalError = (160 - ai_vision_12.largest_object.centerX)/160
             drivetrain.moveRPM(40, horizontalError * kPHorizontal)
             

            #  print(height)
            #  lift.matchHeightPeriodic(height)



def Autodrive():
        finished = False
        while not finished:
            moveInches(10, 80)
            climbRamp() #Climb the ramp and then stop
            wait(1000) # wait 1 second. 
            imuTurnFieldRelative(90, 0)
            moveInches(15, 80)
            imuTurnFieldRelative(0, 0)
            moveInches(2, 40)

            print("Ready to spot fruits")



            finished = True
        return


            
#Command based, needs completion before resuming with the stack 
def findTorqueToMovement(Motor1):
    finished = False
    lasttorque = 0
    lastPosition = Motor1.position(RotationUnits.DEG)
    speed = 0; 
    Motor1.spin_for(FORWARD, 60, DEGREES, False) #Ignore completing this command, and move on to the loop.
    while not(finished): 
        lasttorque = Motor1.torque(TorqueUnits.INLB)
        if Motor1.position(RotationUnits.DEG) - lastPosition >= 3:
            print("Last torque : " + str(lasttorque))
            Motor1.stop(BrakeType.BRAKE)
            # brain.screen.print_at(1,1,lasttorque)
            finished = True
        # Motor1.spin(FORWARD, speed, RPM)
        # speed+=1



################################################################################
#Function calls begin here
# brain_inertial.reset_rotation()
# brain_inertial.reset_heading()
brain_inertial.calibrate()
left_lift_motor.reset_position()

wait(2500)
print("calibration finished")


# findTorqueToMovement(left_lift_motor)
# left_lift_motor.stop(BrakeType.COAST)
# lift(12) #lift two inches
# driveTeleOp()
wait(2000)
# climbRamp()
print("Commands finished")
Autodrive()