from vex import *
import time


class drivetrainController:
    def __init__(self, VEXcontroller, brain, left_motor, right_motor):
        self.VEXcontroller = VEXcontroller
        self.brain = brain
        self.left_motor = left_motor
        self.right_motor = right_motor
    def drive(self): 
        #self.left_motor.spin_with_voltage(self.VEXcontroller.get)
        #self.right_motor.spin_with_voltage()
        return