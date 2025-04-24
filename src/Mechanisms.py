from vex import *
import math
from Auxilary import Auxilary

class Mechanisms: 
    def __init__(self, left_lift_motor, right_lift_motor):
        self.left_motor = left_lift_motor
        self.right_motor = right_lift_motor


    def lift(self, height):
        kP = 1
        finished = False
        distance = 0
        pitchDiameter = 0.5 #In inches
        currHeight = math.sin(math.acos((9.5 - 0)/(9.5))) * 9.5 * 2
        pitchCircumference = pitchDiameter * math.pi
        self.left_motor.reset_position() #Do this somewhere else, BEFORE all the lift(self, height) calls. 
        tolerance = 0.5 #In inches

        while not finished: 
            distance = (self.left_motor.position(RotationUnits.DEG))/360 * (pitchCircumference)
            currHeight = math.sin(math.acos((9.5 - distance)/(9.5))) * 9.5 * 2
            error = height - currHeight #When positive, motors should move clockwise(forwards)[it is inverted for one of the two motors]
            #Otherwise, the current height is greater than setpoint height, hence the error is negative, in this case the pinions should move backwards.
            #TODO: Make sure this happens and resolve any inversion issues.
            if Auxilary.rangeCheck(currHeight, height - tolerance, height + tolerance): 
                self.left_motor.stop(BrakeType.BRAKE)
                self.right_motor.stop(BrakeType.BRAKE)
            else: 
                self.left_motor.spin(FORWARD, kP * error, PERCENT)
                self.right_motor.spin(FORWARD, kP * error, PERCENT)

    


             

