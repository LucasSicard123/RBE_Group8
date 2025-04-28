from vex import *
import math
from Auxilary import Auxilary
from Auxilary import PIDController

class Lift: 
    pitchDiameter = 0.5 #In inches
    pitchCircumference = pitchDiameter * math.pi
    def __init__(self, left_lift_motor):
        self.left_motor = left_lift_motor
        self.PID = PIDController(1, 0, 0, 0) #Initialize with a setpoint of 0
    #Independent command that works like a command with a completion condition.
    def lift(self, height):
        kP = 1
        finished = False
        distance = 0
        currHeight = math.sin(math.acos((9.5 - 0)/(9.5))) * 9.5 * 2
        # self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
        tolerance = 0.1 #In inches
        print(currHeight)
        if(self.left_motor.position(RotationUnits.REV) >= 2.47): 
            self.left_motor.stop(BrakeType.BRAKE)

            return #Limit reached.

        while not finished: 
            distance = (self.left_motor.position(RotationUnits.DEG))/360 * (self.pitchCircumference)
            currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
            error = height - currHeight #When positive, motors should move clockwise(forwards)[it is inverted for one of the two motors]
            #Otherwise, the current height is greater than setpoint height, hence the error is negative, in this case the pinions should move backwards.
            #TODO: Make sure this happens and resolve any inversion issues.
            self.PID.setpoint = height
            pidOutput = self.PID.compute(currHeight, 0.1) #Vex motors calculate every 10ms, this is simply for 
            print(pidOutput)
            if Auxilary.rangeCheck(currHeight, height - tolerance, height + tolerance): 
                self.left_motor.stop(BrakeType.BRAKE)
                finished = True
            else: 
                self.left_motor.spin(FORWARD, kP * error, PERCENT)
                # self.left_motor.spin(FORWARD, pidOutput, PERCENT)
                # self.right_motor.spin(FORWARD, pidOutput, PERCENT)

    #A periodic version to be called during the main loop stack. 
    def matchHeightPeriodic(self, height):
        kP = 1
        finished = False
        distance = 0
        # self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
        tolerance = 0.1 #In inches 
        distance = (self.left_motor.position(RotationUnits.DEG))/360 * (self.pitchCircumference) #TODO: Iffy
        if(self.left_motor.position(RotationUnits.REV) >= 2.47): 
            self.left_motor.stop(BrakeType.BRAKE)

            return #Limit reached.

        currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
        error = height - currHeight #When positive, motors should move clockwise(forwards)[it is inverted for one of the two motors]
        #Otherwise, the current height is greater than setpoint height, hence the error is negative, in this case the pinions should move backwards.
         #TODO: Make sure this happens and resolve any inversion issues.
        self.PID.setpoint = height
        pidOutput = self.PID.compute(currHeight, 0.1) #Vex motors calculate every 10ms, this is simply for 
        print(pidOutput)
        if Auxilary.rangeCheck(currHeight, height - tolerance, height + tolerance): 
            self.left_motor.stop(BrakeType.BRAKE)
            finished = True
        else: 
            # self.left_motor.spin(FORWARD, kP * error, PERCENT)
            self.left_motor.spin(FORWARD, pidOutput, PERCENT)
            # self.right_motor.spin(FORWARD, kP * error, PERCENT)

class Arm:
    highLimit = 80#TODO: Change, most definitely is needed this will be the upper bound of the servo/motor that drive the claw
    lowLimit = 0 #TODO: Change, might not be needed
    state = True #assumes a closed claw. 
    def __init__(self, pinchMotor):
        self.pinchMotor = pinchMotor
    
    def actuate(self):
        if self.state: 
            #Goal is to open up and stay that way
            self.pinchMotor.spin_to_position(self.highLimit)
        else: 
            self.pinchMotor.spin_to_position(self.lowLimit)
        return  
    
    def getState(self): 
        return self.state


    


             

