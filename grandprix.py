"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-prereq-labs

File Name: grand_prix.py

Title: Grand Prix

Author: Krista Sebastian

Purpose: Write a script to enable fully autonomous behavior from the RACECAR. The
RACECAR will traverse the obstacle course autonomously without human intervention.
Once the start button is pressed, the RACECAR must drive through the course until it
reaches finish line.

Note: There is no template code in this document to follow except for the RACECAR script 
structure found in template.py. You are expected to use code written from previous labs
to complete this challenge. Good luck!

Expected Outcome: When the user runs the script, they must not be able to manually control
the RACECAR. The RACECAR must move forward on its own, traverse through the course, and then
stop on its own.
"""

########################################################################################
# Imports
########################################################################################

import sys
import cv2 as cv

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

# Declare any global variables here
state = ''
markers = []

#THRESHOLDS


#DETECTORS
left_detected = False
right_detected = False

#VALUES


#SETTERS
speed = 0
angle = 0

########################################################################################
# Functions
########################################################################################

class ARMarker:
    def __init__(self, marker_id, marker_corners, orientation, area):
        self.id = marker_id
        self.corners = marker_corners
        self.orientation = orientation
        self.area = area
        self.color = ''
        self.color_area = 0
    
    def find_color_border(self, image):
        crop_points = self.crop_points(image)
        image = image[crop_points[0][0]:crop_points[0][1], crop_points[1][0]:crop_points[1][1]]

        color_name, color_area = self.find_colors(image)
        self.color = color_name
        self.color_area = color_area
        
        rc.display.show_color_image(image)


    def crop_points(self, image):
        ORIENT = {'UP': 0, 'LEFT': 1, 'DOWN': 2, 'RIGHT': 3}
        current_orientation = self.orientation

        marker_left, marker_top = self.corners[ORIENT[current_orientation]]
        marker_right, marker_bottom = self.corners[(ORIENT[current_orientation] + 2)% 4]

        half_length = (marker_right - marker_left) // 2
        half_width = (marker_bottom - marker_top) // 2

        marker_top = max(0, marker_top - half_width)
        marker_left = max(0, marker_left - half_length)
        marker_bottom = min(image.shape[0], marker_bottom + half_width) + 1
        marker_right = min(image.shape[1], marker_right + half_length) + 1

        return ((int(marker_top), int(marker_bottom)), (int(marker_left), int(marker_right)))


def detect_AR_Tag(image):
    markers = []

    corners, ids, _ = detector.detectMarkers(image)
    
    for x in range(len(corners)):
        current_corners = corners[x][0]
        orientation = ""

        if current_corners[0][0] == current_corners[1][0]:
            if current_corners[0][1] < current_corners[1][1]:
                orientation = 'RIGHT'
            else:
                orientation = 'LEFT'
        else:
            if current_corners[0][0] > current_corners[1][0]:
                orientation = 'DOWN'
            else:
                orientation = 'UP'

        area = abs((current_corners[2][0] - current_corners[0][0]) * (current_corners[2][1] - current_corners[0][1]))

        current_marker = ARMarker(ids[x][0], current_corners, orientation, area)
        markers.append(current_marker)
        
    cv.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
    return markers, image

def find_orientation(corners):
    ax = corners[0][0]
    ay = corners[0][1]
    bx = corners[1][0]
    by = corners[1][1]

    if ax == bx:
        if ay>by:
            return 'LEFT'
        else:
            return 'RIGHT'
    else:
        if ax > bx:
            return 'DOWN'
        else:
            return 'UP'
        

def find_area(corners):
    return abs((corners[0][2][0] -corners[0][0][0]) * (corners[0][2][1] - corners[0][0][1]))



def clamp(value, min_val: float, max_val: float):
    """Clamps a value within a specified range."""
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    else:
        return value 

def update_aruco():
    global markers, state
    image = rc.camera.get_color_image()

    markers = rc_utils.get_ar_markers(image)
    rc_utils.draw_ar_markers(image, markers)

    for marker in markers:
        area = find_area(marker.get_corners_aruco_format())
        if marker.get_id() == 32 and area >= 1400:
            state = 'BETWEEN LINES'



def start():
    rc.drive.set_speed_angle(speed, angle)

 
def update():
    global speed, angle

    update_aruco()

    if state == '':
        pass
    else:
        pass


    rc.drive.set_speed_angle(speed, angle)

def update_slow():
    for marker in markers:
        print(f"ID: {marker.get_id()}")
        print(f"Corners: {marker.get_corners_aruco_format()}")
        print(f"Area: {find_area(marker.get_corners_aruco_format())}")
        print(f"Orientation: {marker.get_orientation()}")
        print('==========================')


########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
