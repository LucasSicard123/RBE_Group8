from vex import *
import time
from Auxilary import Auxilary


class DrivetrainController:

    def __init__(self, VEXcontroller, brain, left_motor, right_motor, imu):
        self.VEXcontroller = VEXcontroller
        self.brain = brain
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.intertial = imu

    def movePercent(self, n_percent, n_percentDirection): 
        self.left_motor.spin(FORWARD, Auxilary.clamp((n_percent - n_percentDirection) * 100, -100, 100), PERCENT)
        self.right_motor.spin(FORWARD, Auxilary.clamp((n_percent + n_percentDirection) * 100, -100, 100), PERCENT)
        return
    
    def moveRPM(self, n_RPM, n_percentDirection): 
        self.left_motor.spin(FORWARD, Auxilary.clamp(n_RPM - n_percentDirection * 2 * n_RPM, -100, 100), RPM)
        self.right_motor.spin(FORWARD, Auxilary.clamp(n_RPM + n_percentDirection * 2 * n_RPM, -100, 100), RPM)
        return
    
    def imuTurnFieldRelative(self, n_heading):
        heading = self.intertial.heading()
        toTurn = True
        tolerance = 3 #TODO: Change for later, after testing. 
        while toTurn: 
            heading = self.intertial.heading()
            if Auxilary.rangeCheck(heading, n_heading - 3, n_heading + 3): 
                self.stopMotors()
            if heading < n_heading: 
                self.moveRPM(35, 1)
            if heading > n_heading: 
                self.moveRPM(35, -1)

    

    def stopMotors(self):
        self.left_motor.stop(BrakeType.BRAKE)
        self.right_motor.stop(BrakeType.BRAKE)

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

    def Autodrive(self):
        finished = False
        while not finished:
            finished = True
        return 
    



    