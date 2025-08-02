import os
import sys
import time
import cv2 as cv # type: ignore
import numpy as np # type: ignore

sys.path.insert(1, "library")
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

from pycoral.adapters.common import input_size # type: ignore
from pycoral.adapters.detect import get_objects # type: ignore
from pycoral.utils.dataset import read_label_file # type: ignore
from pycoral.utils.edgetpu import make_interpreter # type: ignore
from pycoral.utils.edgetpu import run_inference # type: ignore

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'elevator_edgetpu.tflite'
label_name = 'elevator-objects.txt'
model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 2

interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()
labels = read_label_file(label_path)
inference_size = input_size(interpreter)

# Initialize car
rc = racecar_core.create_racecar()

# Global variables
contour_center = None
contour_area = 0

speed = 0
angle = 0 
prev_error=0
prev_angle=0

sign = ""

center = 160

area = 0

tag = {1:'stop',2:"go"}

state = 1

elv_count=0

import sys
import cv2 # type: ignore
import numpy as np # type: ignore

# Insert path to Racecar library
sys.path.insert(1, 'library')
import racecar_core # type: ignore
import racecar_utils as rc_utils # type: ignore

# Create the Racecar object
rc = racecar_core.create_racecar()

# AR tag detection setup
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

class ARMarker:
    def __init__(self, marker_id, corners, orientation, area):
        self.id = marker_id
        self.corners = corners
        self.orientation = orientation
        self.area = area


def find_orientation(corners):
    # Determine orientation based on the first two corners
    if corners[0][0] == corners[1][0]:
        return "LEFT" if corners[0][1] > corners[1][1] else "RIGHT"
    else:
        return "DOWN" if corners[0][0] > corners[1][0] else "UP"


def find_area(corners):
    # Compute area from opposing corners
    return abs((corners[2][0] - corners[0][0]) * (corners[2][1] - corners[0][1]))


def detect_ar_tags(image):
    markers = []
    corners_list, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return markers
    for idx, corner_array in enumerate(corners_list):
        corners = corner_array[0]
        orientation = find_orientation(corners)
        area = find_area(corners)
        markers.append(ARMarker(ids[idx][0], corners, orientation, area))
    return markers
                
# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]

    # indexes through each object
    for obj in objs:
        # if the model is more than 60% sure of the object being a car, enter the loop
        if obj.score > 0.6:
            
            # stores dimensions of bounding box for object
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            # assigns the percent certainty of the object being correct to "label"
            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
            
            # creates the image to be displayed (bounding box and corresponding object label)
            cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv.putText(cv2_im, label, (x0, y0+30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            break
    return cv2_im

# Called once on start
def start():
    rc.drive.set_speed_angle(0, 0)

# Called every frame
def update():

    global speed, angle  
    global prev_error, prev_angle
    global center,area,sign
    global elv_count
    if not state == 1:
        scan = rc.lidar.get_samples() # get lidar scan
        left = rc_utils.get_lidar_average_distance(scan, 310, 50)
        error = 115.0 - left
        dt=rc.get_delta_time()
        # proportional factor and derivative factor
        kd = 0.003 # 0.003
        kp = 0.0055 # 0.005
        kp = 0.013
        derror=error-prev_error
        
        angle = kp*error + derror/dt*kd
        prev_angle = angle

        kps = 0.6
        kps = 0.38
        
        cangle=rc_utils.clamp(angle, -1.0,1.0)
        speed = 1. - kps * np.abs(cangle)
        rc.drive.set_speed_angle(speed, cangle)
        prev_error=error
    else:
        rc.drive.set_speed_angle(0, 0)
        thresh = 3000
        if sign == "go":
            if area > thresh:
                rc.drive.set_speed_angle(0, 0)
            setpoint = 320
            error = 
        elif sign == "":
            pass
            # hard code
        else:
            rc.drive.set_speed_angle(0, 0)

        # elevator
        print(sign)
        

    ###############################################################################
    # ar tag
    image = rc.camera.get_color_image()
    if image is None:
        return
    markers = detect_ar_tags(image)
    for m in markers:
        state = m.id
        print(f"Marker ID: {m.id}, Size: {m.area}")
    
def update_slow():
    # global variables
    global center, sign, area
    frame = rc.camera.get_color_image()
    
    if frame is not None:
        # converts base image to also display object bounding boxes
        rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb_image = cv.resize(rgb_image, inference_size)
        
        # adds object bounding boxes to image
        run_inference(interpreter, rgb_image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        image = append_objs_to_img(frame, inference_size, objs, labels)

        rc.display.show_color_image(image)
        if len(objs) == 0:
            sign = ""
        for obj in objs:
            if obj.score > 0.6:
                sign = tag[obj.id+1]
                loc = (obj.bbox.xmin + obj.bbox.xmax)//2
                area = (obj.bbox.xmin - obj.bbox.xmax ) * (obj.bbox.ymin - obj.bbox.ymax )
                center = loc
                print(f"{tag[obj.id+1]}")
            else:
                sign = ""

if __name__ == "__main__":
    rc.set_update_slow_time(0.1)
    rc.set_start_update(start, update, update_slow)
    rc.go()