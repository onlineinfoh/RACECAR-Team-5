# Imports
import sys
import cv2 as cv # type: ignore
import numpy as np # type: ignore
sys.path.insert(1, "../../library")
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

# Initialize racecar instance
rc = racecar_core.create_racecar()

# Initialize threshold values
MIN_CONTOUR_AREA = 5


CROP_FLOOR = (4 * rc.camera.get_height()//5, 0), (rc.camera.get_height(),rc.camera.get_width()) # Crop size
ORANGE = ((7, 77, 156),(12, 218, 255)) # HSV values
GREEN = ((67,57,77),(104,220,230)) # HSV values

# Setup a priority list for the colors
COLOR_PRIORITY = (GREEN, ORANGE)

# Initial speed and angles
speed = 0.0
angle = 0.0  

# Important variables for contour detection and line following
contour_center = None 
contour_area = 0
prev_error = 0

# Ran 60 times per second
def update_contour():
    global contour_center
    global contour_area
    global prev_error

    # Retrieve the image
    image = rc.camera.get_color_image()

    if image is None:
        contour_center = None
        contour_area = 0
    else:
        # Crop the image
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Compute the contour center and the contour area
        contours = rc_utils.find_contours(image, COLOR_PRIORITY[0][0], COLOR_PRIORITY[0][1])
        if len(contours) == 0:
            contours = rc_utils.find_contours(image, COLOR_PRIORITY[1][0], COLOR_PRIORITY[1][1])
        if len(contours) > 0:
            largest_contour = rc_utils.get_largest_contour(contours)
            contour_center = rc_utils.get_contour_center(largest_contour)
            if largest_contour is not None:
                contour_area = rc_utils.get_contour_area(largest_contour)
        
        return contours, contour_center

# set starting values of speed and angle
def start():
    global speed
    global angle

    speed = 0
    angle = 0

    rc.drive.set_speed_angle(speed, angle)

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

# Update is run 60 times per second
def update():
    global speed
    global angle
    global prev_error
    
    #updates contour_center
    update_contour()
    
    if contour_center is not None:
        
        # sets error and change in error with PD control 
        setpoint = rc.camera.get_width() // 2
        val_now = contour_center[1]
        error = setpoint - val_now
        dt = rc.get_delta_time()
        derror = error-prev_error

        #sets proporitional control values   
        kp= -0.01
        kd = -5.0
     
        #calculates proportional angle
        angle = kp * error + kd * derror/dt
        
        #clamps the angle between -1 and 1
        angle = rc_utils.clamp(angle, -1, 1)
        
        # save a copy of the current computed error value
        prev_error = error
        
        #sets proportional speed
        speed = 1 - abs(angle) * 0.15
        print(speed)

        #clamps the speed between 0 and 1
        speed = rc_utils.clamp(speed, 0, 1)

    rc.drive.set_speed_angle(speed, angle)

# Run every second
def update_slow():
    pass

# Main function DO NOT CHANGE
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()