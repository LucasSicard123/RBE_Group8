from vex import *
import time


class drivetrainController:
    def __init__(self, VEXcontroller, brain, left_motor, right_motor):
        self.VEXcontroller = VEXcontroller
        self.brain = brain
        self.left_motor = left_motor
        self.right_motor = right_motor
    def drive(self): 
        toDrive = True
        while toDrive: 
            #Normalized values [-1,1]
            drive_r = self.VEXcontroller.axis3.position #rotational 
            drive_f = self.VEXcontroller.axis2.position #speed control
    def movePercent(self, n_percent, speed_rpm): 
        self.left_motor.spin(FORWARD, n_percent, PERCENT)
        self.right_motor.spin(FORWARD, n_percent, PERCENT)
        return