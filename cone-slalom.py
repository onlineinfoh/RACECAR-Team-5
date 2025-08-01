import sys
import cv2 as cv
import numpy as np
sys.path.insert(0, "../library")
import racecar_core
import racecar_utils as rc_utils

# Create global variables and initialization
rc = racecar_core.create_racecar()

# HSV ranges
ORANGE_MIN  = np.array([1, 170, 180])
ORANGE_MAX  = np.array([15, 255, 255])

GREEN_MIN = np.array([25, 78, 141])
GREEN_MAX = np.array([84, 217, 255])

# Dictionary of colors corresponding to HSV ranges 
color_ranges = {
    "orange": (ORANGE_MIN, ORANGE_MAX), 
    "green" : (GREEN_MIN, GREEN_MAX)
}

# color order!
colors = ["orange", "green"]

# initialize global variables
speed = 0
angle = 0
state = "approach cone"
contour_color = None
area = 0
image = None

# returns the color, contour_center, and area (in order) of the largest contour (aka cone)
def detect_cones(image, min_area=1000):
    # converts the image from BGR to HSV
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # re-initializes color, center, and area as None or 0
    detected_color = None
    contour_center = None
    best_area = 0

    # indexes through each color in color_ranges
    for color_name, (hsv_min, hsv_max) in color_ranges.items():
        mask = cv.inRange(hsv, hsv_min, hsv_max) # creates a mask of the current color

        contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) # finds all the contours in the mask

        # indexes through each contour in contours list
        for contour in contours:
            area = cv.contourArea(contour) 

            # if current contour's area is greater than the min_area 
            # (to avoid capturing random, small, noisy ones)
            # and if it is the biggest area so far
            # enter the loop and assign best_area, detected_color, and contour_center to the current, a.k.a biggest, contour
            if area > min_area and area > best_area:
                best_area       = area
                detected_color  = color_name
                contour_center  = rc_utils.get_contour_center(contour)  
    return detected_color, contour_center, best_area
    
# the difference is that only the middle strip of the image (i.e. cones in front) are being detected
def detect_front_cones(image, min_area = 700):
    cropped_image = rc_utils.crop(image, (0, 280), (rc.camera.get_height(), 360))
    hsv = cv.cvtColor(cropped_image, cv.COLOR_BGR2HSV)

    detected_color = None
    contour_center = None
    best_area = 0

    for color_name, (hsv_min, hsv_max) in color_ranges.items():
        mask = cv.inRange(hsv, hsv_min, hsv_max) # creates mask of current color
        contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE) # saves all contours of current color

        for contour in contours:
            area = cv.contourArea(contour) # returns area of current contour
            if area > min_area and area > best_area: # same logic as detect cones
                best_area       = area
                detected_color  = color_name
                contour_center  = rc_utils.get_contour_center(contour)
    return contour_center

# returns angle and distance of the closest point
def update_lidar():
    scan = rc.lidar.get_samples() # saves array of lidar samples
    angle, distance = rc_utils.get_lidar_closest_point(scan) # saves angle and distance of closest point
    return angle, distance

# returns angle and distance of the closest point to the right
def update_lidar_right():
    scan = rc.lidar.get_samples() # saves array of lidar samples
    angle , distance = rc_utils.get_lidar_closest_point(scan, (0, 180)) # saves angle and dist of closest point to the right
    return angle, distance
    
# returns angle and distance of the closest point to the left
def update_lidar_left():
    scan = rc.lidar.get_samples() # saves array of lidar samples
    angle , distance = rc_utils.get_lidar_closest_point(scan, (180, 360)) # saves angle and dist of closest point to the left
    return angle, distance

# returns angle and distance of the closest point to the front
def update_lidar_front():
    scan = rc.lidar.get_samples()
    angle, distance = rc_utils.get_lidar_closest_point(scan, (340,20)) # saves angle and dist of closest point to the front
    return angle, distance

def start():
    pass

def update():
    global angle, speed, state, contour_color, area, image

    # initializes closest angle and distance, and image from color camera
    angle_of_object, distance = update_lidar()
    image = rc.camera.get_color_image()

    # first state: approach cone until the contour area is large enough for the car to begin turning --> change states
    if state == "approach cone":
        contour_color, center, area = detect_cones(image)

        if area < 3800:
            speed = 0.6 # sets speed for if cone is further away
        elif area < 6000:
            speed = 0.6 # sets speed for if cone is closer
            angle = 0
        else:
            state = "turn"  # turns if cone is close enough
    # second state: turn around the current cone until there isn't enough space to turn --> change states
    if state == "turn":
        if contour_color == colors[1]:
            contour_color, center, area = detect_cones(image)

            # if the color of the largest contour has changed, start charging forward past the current cone
            if contour_color != colors[1]:
                contour_color = colors[1]
                state = "charge"
            # otherwise, turn around the current cone
            else:
                if center is not None :
                    speed = 0.6
                    angle = 1
                # if there is no contour (it's out of view), start charging forward past the current cone
                else:
                    speed = 0.6
                    state = "charge"

        # same logic as above, but w/ the other color of cone (so now it turns the opposite direction)
        if contour_color == colors[0]:
            contour_color, center, area = detect_cones(image)
            if contour_color != colors[0]:
                contour_color = colors[0]
                state = "charge"
                angle = 0
            else:
                if center is not None :
                    speed = 0.6
                    angle = -1
                else:
                    speed = 0.6
                    state = "charge"

    # third state: charge past the cone until the racecar passes the cone --> change states
    if state == "charge":
        if contour_color == colors[1]:

            # update angle_of_left
            angle_of_left, distance = update_lidar_left()
            
            # if the angle of the closest point to the left is less than 300, start turning towards the next cone
            if distance < 25:
                angle = 1
            elif angle_of_left < 300:
                state = "turn back"

            # otherwise, go forward
            else:
                speed = 0.6
                angle = 0
        # similar logic as above 
        if contour_color == colors[0]:
            angle_of_right, distance = update_lidar_right()
            if distance < 25:
                angle = -1
            elif angle_of_right> 60:
                state = "turn back"
            else:
                speed = 0.6
                angle = 0
                
    # fourth state: after passing a cone, turn towards the next cone
    if state == "turn back":

        # finds contour center of front cones
        center = detect_front_cones(image)

        # if there is no center found, start spproaching the next cone
        if center is not None:
            state = "approach cone"

        # otherwise, turn until a cone is found
        else:
            if contour_color == colors[1]:
                speed = 0.6
                angle = -1
            if contour_color == colors[0]:
                speed = 0.6
                angle = 1

    print(f"Sp: {speed}, Ang: {angle}, {state}, {contour_color}, cone angle: {angle_of_object}, dist: {distance}") # print values
    # rc.display.show_color_image(image) # Show the image captured by the camera

    # send speed and angle to the racecar
    rc.drive.set_speed_angle(speed, angle)
    
def update_slow():
    pass 

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()