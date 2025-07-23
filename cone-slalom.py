"""
MIT BWSI Autonomous RACECAR
MIT License

File Name: cone-slalom.py

Title: Cone Slalom

Author: Aidan Wong
"""

import sys
import cv2 as cv # type: ignore
import numpy as np # type: ignore

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, "../library")
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

rc = racecar_core.create_racecar()

# >> Constants
BLUE_MIN  = np.array([1, 170, 180])
BLUE_MAX  = np.array([15, 255, 255])

RED_MIN   = np.array([165,  50,  50])
RED_MAX   = np.array([179, 255, 255])

GREEN_MIN = np.array([25,78,141])
GREEN_MAX = np.array([84,217, 255])


color_ranges = {
    "blue": (BLUE_MIN, BLUE_MAX), 
    "red" : (RED_MIN,  RED_MAX),
    "green": (GREEN_MIN, GREEN_MAX)
}

# temporary
color_ranges = {
    "blue": (BLUE_MIN, BLUE_MAX), 
    "red" : (GREEN_MIN, GREEN_MAX)
}

# >> Variables
speed = 0
angle = 0
state = "approach cone"
contour_color = None
area = 0
image = None
past_color = None

def get_distance_to_cone():
    depth_image = rc.camera.get_depth_image()
    closest_pixel = rc_utils.get_closest_pixel(depth_image)
    distance = depth_image[closest_pixel[0]][closest_pixel[1]]
    return distance, closest_pixel

def detect_cones(image, min_area=1000):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    detected_color  = None
    contour_center  = None
    best_area       = 0

    for color_name, (hsv_min, hsv_max) in color_ranges.items():
        mask = cv.inRange(hsv, hsv_min, hsv_max)
        contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv.contourArea(contour)
            if area > min_area and area > best_area:
                best_area       = area
                detected_color  = color_name
                contour_center  = rc_utils.get_contour_center(contour)  # your helper
    return detected_color, contour_center, best_area
    
def detect_front_cones(image, min_area = 700):
    cropped_image = rc_utils.crop(image, (0, 280), (rc.camera.get_height(), 360))
    hsv = cv.cvtColor(cropped_image, cv.COLOR_BGR2HSV)

    detected_color  = None
    contour_center  = None
    best_area       = 0

    for color_name, (hsv_min, hsv_max) in color_ranges.items():
        mask = cv.inRange(hsv, hsv_min, hsv_max)
        contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv.contourArea(contour)
            if area > min_area and area > best_area:
                best_area       = area
                detected_color  = color_name
                contour_center  = rc_utils.get_contour_center(contour)  # your helper
    return contour_center
    
def update_lidar():
    scan = rc.lidar.get_samples()
    angle, distance = rc_utils.get_lidar_closest_point(scan)
    return angle, distance
    
def update_lidar_right():
    scan = rc.lidar.get_samples()
    angle , distance = rc_utils.get_lidar_closest_point(scan, (0, 180))
    return angle, distance
    
def update_lidar_left():
    scan = rc.lidar.get_samples()
    angle , distance = rc_utils.get_lidar_closest_point(scan, (180, 360))
    return angle, distance
    
def update_lidar_front():
    scan = rc.lidar.get_samples()
    angle, distance = rc_utils.get_lidar_closest_point(scan, (340,20))
    return angle, distance

def start():
    global speed, angle, state, contour_color
    speed = 0
    angle = 0
    state = "approach cone"
    contour_color = None # Remove 'pass' and write your source code for the start() function here

def update():
    global angle, speed, state, contour_color, area, image

    angle_of_object, distance = update_lidar()
    image = rc.camera.get_color_image()
    if state == "approach cone":
        
        contour_color, center, area = detect_cones(image)
        past_color = contour_color
        if area < 6000:
            speed = 0.6
        if area < 3800:
            speed = 0.6
            angle = 0   
        
        else:
            state = "see cone"        
            
    if state == "see cone":
        contour_color, center, area = detect_cones(image)
        print(contour_color) 
        state = "turn"
    if state == "turn":
        if contour_color == "red":
            contour_color, center, area = detect_cones(image)
            if contour_color != "red":
                contour_color = "red"
                state = "charge"
            else:
                if center is not None :
                    speed = 0.6
                    angle = 1
                else:
                    speed = 0.6
                    state = "charge"
        if contour_color == "blue":
            contour_color, center, area = detect_cones(image)
            if contour_color != "blue":
                contour_color = "blue"
                state = "charge"
                angle = 0
            else:
                if center is not None :
                    speed = 0.6
                    angle = -1
                else:
                    speed = 0.6
                    state = "charge"
    if state == "charge":
        if contour_color == "red":
            angle_of_left, _ = update_lidar_left()
            print(angle_of_left)
            if angle_of_left < 300:
                state = "turn back"
            else:
                speed = 0.6
                angle = 0
        if contour_color == "blue":
            angle_of_right, distance = update_lidar_right()
            if distance < 25:
                angle = -1
            elif angle_of_right> 60:
                state = "turn back"
            else:
                speed = 0.6
                angle = 0
        
    if state == "turn back":
        angle_of_front , distance_of_front = update_lidar_front()
        center = detect_front_cones(image)
        if center is not None:
            state = "approach cone"
        else:
            if contour_color == "red":
                speed = 0.6
                angle = -1
            if contour_color == "blue":
                speed = 0.6
                angle = 1

    # Show the image captured by the camera
    print(speed, angle,state, contour_color, angle_of_object, distance)
    
    # rc.display.show_color_image(image)
    rc.drive.set_speed_angle(speed, angle)
    
def update_slow():
    pass 

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()