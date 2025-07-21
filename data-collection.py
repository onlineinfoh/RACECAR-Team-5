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
    rc.set_update_slow_time(0.5)

# Called every frame
def update():
    # Manual override for testing
    speed = 0
    if rc.controller.get_trigger(rc.controller.Trigger.RIGHT) > 0:
        speed = 1
    elif rc.controller.get_trigger(rc.controller.Trigger.LEFT) > 0:
        speed = -1

    x, _ = rc.controller.get_joystick(rc.controller.Joystick.LEFT)
    angle = 1 if x > 0 else -1 if x < 0 else 0

    rc.drive.set_speed_angle(speed, angle)

    # Capture and save an image when A button is pressed
    if rc.controller.was_pressed(rc.controller.Button.A):
        os.makedirs('data', exist_ok=True)
        img = rc.camera.get_color_image()
        filename = f"data/image_{int(time.time())}.jpeg"
        cv.imwrite(filename, img)
        print(f"Saved image to {filename}")

# Called once per second
def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()