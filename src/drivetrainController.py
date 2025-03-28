from vex import *
import time


class drivetrainController:
    def __init__(self, VEXcontroller, brain, left_motor, right_motor):
        self.VEXcontroller = VEXcontroller
        self.brain = brain
        self.left_motor = left_motor
        self.right_motor = right_motor
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    def movePercent(self, n_percent, n_percentDirection): 
        self.left_motor.spin(FORWARD, self.clamp(n_percent - n_percentDirection, -1, 1), PERCENT)
        self.right_motor.spin(FORWARD, self.clamp(n_percent + n_percentDirection, -1, 1), PERCENT)
        return
    def drive(self): 
        toDrive = True
        while toDrive: 
            #Normalized values [-1,1]
            drive_r = self.VEXcontroller.axis3.position #rotational 
            drive_f = self.VEXcontroller.axis2.position #speed control

            self.movePercent(drive_f, drive_r)
            if self.VEXcontroller.buttonA.pressing :
                self.movePercent(0,0) 
                toDrive = False