import os
import sys
import time
import cv2 as cv # type: ignore
import numpy as np # type: ignore

sys.path.insert(1, "../../library")
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

# Initialize car
rc = racecar_core.create_racecar()

# Global variables
contour_center = None
contour_area = 0

# Called once on start
def start():
    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5) # sets update slow frequency to 2 Hz

# Called every frame
def update():


    # MANUAL DRIVE
    # speed determined by left joystick
    x, y = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    speed=x

    # angle determined by right joystick
    a,b=rc.controller.get_joystick(rc.controller.Joystick.RIGHT)
    angle=a

    # sets speed and angle to determined values
    rc.drive.set_speed_angle(speed, angle)


    # Capture and save an image when A button is pressed
    if rc.controller.was_pressed(rc.controller.Button.A): # runs if A button is pressed
        os.makedirs('data', exist_ok=True) # creates data directory if it doesn't exist already
        img = rc.camera.get_color_image() # gets image from camera
        filename = f"data/image_{int(time.time())}.jpeg" # makes a unique filename using timestamp
        cv.imwrite(filename, img) # saves image to file
        print(f"Saved image to {filename}") # confirms save

# Called once per second
def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()