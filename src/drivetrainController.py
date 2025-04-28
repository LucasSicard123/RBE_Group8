from queue import LifoQueue
from vex import *
import time
from Auxilary import Auxilary


class DrivetrainController:

    # colorStack.put(Colordesc()) #yellow 

    wheelDiameter = 4 #in inches
    wheelCircumference = 3.14 * wheelDiameter
    degreesPerInch = 360.0 / wheelCircumference

    def __init__(self, VEXcontroller, brain, left_motor, right_motor, imu):
        self.VEXcontroller = VEXcontroller
        self.brain = brain
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.inertial = imu
    #TODO: Math seems find but might be wrong. tbd
    def movePercent(self, n_percent, n_percentDirection): 
        self.left_motor.spin(FORWARD, Auxilary.clamp((n_percent - 2 * n_percentDirection) * 100, -100, 100), PERCENT)
        self.right_motor.spin(FORWARD, Auxilary.clamp((n_percent + 2 * n_percentDirection) * 100, -100, 100), PERCENT)
        return
    #Should work in theory, just not tested yet. 
    def moveRPM(self, n_RPM, n_percentDirection): 
        self.left_motor.spin(FORWARD, Auxilary.clamp(n_RPM - n_percentDirection * 2 * n_RPM, -100, 100), RPM)
        self.right_motor.spin(FORWARD, Auxilary.clamp(n_RPM + n_percentDirection * 2 * n_RPM, -100, 100), RPM)
        return
    
    #Command based function, will have to finish before anything else happens to the motors. 
    def moveInches(self, n_inches, speed_rpm): 
        degreesToTravel = n_inches * self.degreesPerInch # 1 : 1
        degreesToRotate = degreesToTravel #1 : 1, now it is gear ratio compliant
        #terribly done parallel-command group
        self.left_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, False) 
        self.right_motor.spin_for(FORWARD, degreesToRotate, DEGREES, speed_rpm, RPM, True)

    
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

    

    def stopMotors(self):
        self.left_motor.stop(BrakeType.BRAKE)
        self.right_motor.stop(BrakeType.BRAKE)

    def driveTeleOp(self): 
        toDrive = True
        while toDrive: 
            #Normalized values [-1,1]
            drive_r = self.VEXcontroller.axis3.position #rotational 
            drive_f = self.VEXcontroller.axis2.position #speed control

            self.movePercent(drive_f, drive_r)
            if self.VEXcontroller.buttonA.pressing :
                self.movePercent(0,0) 
                toDrive = False

    def climbRamp(self): 
        finished = False
        kP = 1
        while not finished: 
            if Auxilary.rangeCheck(self.inertial.orientation(OrientationType.PITCH), -1, 1):
                self.stopMotors()
                finished = True
                continue
            else: 
                yawError = self.inertial.heading - 0
                #Check inversions, it all hinges on whether or not the gyroscope is Clockwise positive or CCW positive. 
                self.left_motor.spin(FORWARD, 25 - yawError * kP, RPM)
                self.right_motor.spin(FORWARD, 25 + yawError * kP, RPM)

        return
    

    




    