# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Lucas                                                        #
# 	Created:      3/19/2025, 8:32:54 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import time

# Brain should be defined by default
brain = Brain()

brain.screen.print("Hello V5")

left_motor = Motor(Ports.PORT2, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)

# Move left wheels 1 full rotation
left_motor.spin(FORWARD, 30, RPM)
time.sleep(10)
left_motor.stop(BRAKE)

# Move right wheels 1 full rotation
right_motor.spin(FORWARD, 30, RPM)
time.sleep(10)
right_motor.stop(BRAKE)

# Move both wheels 2 full rotations
left_motor.spin(FORWARD, 30, RPM)
right_motor.spin(FORWARD, 30, RPM)
time.sleep(20)
left_motor.stop(BRAKE)
right_motor.stop(BRAKE)

# Done!
brain.screen.print("How did I do?")
