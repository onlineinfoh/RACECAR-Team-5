
########################################################################################
# Imports
########################################################################################

import sys

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
global speed
global angle
global speed_offset
global angle_offset

########################################################################################
# Functions
########################################################################################

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global speed
    global angle
    global speed_offset
    global angle_offset

    speed = 0.0 # The initial speed is at 1.0
    angle = 0.0 # The initial turning angle away from the center is at 0.0


    # This tells the car to begin at a standstill
    rc.drive.stop()


def update():
    global speed
    global angle
    global speed_offset
    global angle_offset
    
    if rc.controller.was_pressed(rc.controller.Button.A):
        

    (x, y) = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    speed=x

    (a, b) = rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    angle=a

    # Send the speed and angle values to the RACECAR
    rc.drive.set_speed_angle(speed, angle)

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update)
    rc.go()
