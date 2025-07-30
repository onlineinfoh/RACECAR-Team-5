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
# MIN_CONTOUR_AREA = 5


CROP_FLOOR = (4 * rc.camera.get_height()//5, 0), (rc.camera.get_height(),rc.camera.get_width()) # Crop size
ORANGE = ((7, 77, 156),(12, 218, 255)) # HSV values for orange
GREEN = ((67,57,77),(104,220,230)) # HSV values for green

# Setup a priority list for the colors
COLOR_PRIORITY = (GREEN, ORANGE)

# Initial speed and angles to rest
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
    image = rc.camera.get_color_image() # gets image!

    if image is None: # failsafe to set center and area if no image retrieved
        contour_center = None
        contour_area = 0
    else:
        # Crop the image
        image = rc_utils.crop(image, CROP_FLOOR[0], CROP_FLOOR[1])

        # Compute the contour center and the contour area
        # finds all contours in the image within the lower and upper HSV tuples of the 1st priority color
        contours = rc_utils.find_contours(image, COLOR_PRIORITY[0][0], COLOR_PRIORITY[0][1])
        if len(contours) == 0: # fallback for if no top priority contours found
            # if no top priority contours found, redo with second priority
            contours = rc_utils.find_contours(image, COLOR_PRIORITY[1][0], COLOR_PRIORITY[1][1])         # finds all contours in the image within the lower and upper HSV tuples of the 1st priority color

        if len(contours) > 0: # run if there are contours
            largest_contour = rc_utils.get_largest_contour(contours) # sets largest contour
            contour_center = rc_utils.get_contour_center(largest_contour) # sets contour center based on largest contour
            if largest_contour is not None: # if there is a largest contour
                contour_area = rc_utils.get_contour_area(largest_contour) # sets contour area
        
        return contours, contour_center

# set starting values of speed and angle
def start(): # runs at the beginning of script
    # accesses global variables
    global speed
    global angle

    speed = 0
    angle = 0

    rc.drive.set_speed_angle(speed, angle) # starts w speed and angle at 0

    # Set update_slow to refresh every half second
    rc.set_update_slow_time(0.5)

# Update is run 60 times per second
def update():
    # accesses global variables
    global speed
    global angle
    global prev_error
    
    # updates contour_center and contour_area
    update_contour()
    
    if contour_center is not None:
        
        # sets error and change in error with PD control 
        setpoint = rc.camera.get_width() // 2 # sets setpoint to center of camera frames x-axis
        val_now = contour_center[1] # defines PV as the current x coordinate of the contour
        error = setpoint - val_now # difference between setpoint and present value (PV)
        dt = rc.get_delta_time() # gets change in time since last time this function was run
        derror = error-prev_error # change in error

        #sets proportional control values   
        kp = -0.01 # coefficient used to base angle on current error
        kd = -5.0 # coefficient used to base angle on change in error over time
     
        #calculates proportional angle
        angle = kp * error + kd * derror/dt 
        
        #clamps the angle between -1 and 1
        angle = rc_utils.clamp(angle, -1, 1)
        
        # save a copy of the current computed error value
        prev_error = error # sets prev_error in order to find change in error in next frame
        
        #sets proportional speed
        kps = 0.15 # coefficient used to make speed dependent on angle. bigger turns -> slower speed
        speed = 1 - abs(angle) * kps
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