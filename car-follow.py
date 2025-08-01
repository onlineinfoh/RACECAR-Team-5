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
model_name = 'car-follower-v1_edgetpu.tflite'
label_name = 'objects.txt'
model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 9

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

tag = {1:'Car'}
                
# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]

    # indexes through each object
    for obj in objs:
        # if the model is more than 60% sure of the object being a car, enter the loop
        if obj.score > 0.6:
            
            print(f"{tag[obj.id+1]}")
            
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

    # calculate the error: present value - setpoint
    error = center - 160
    # Calculate the change in error
    derror=error-prev_error

    # get the change in time
    dt=rc.get_delta_time()

    # proportional factor and derivative factor
    # NOTE: currently, the derivative factor is unused
    kd = 0.9 * 0
    kp = 0.003
    
    # Combine the proportional term and the derivative term to compute the angle 
    angle = kp*error + derror/dt*kd
    # print(angle)

    # change in angle
    # NOTE: currently unused
    dangle = angle - prev_angle
    
    # clamps the angle
    cangle=rc_utils.clamp(angle, -0.5,0.5)
    
    speed = 0.5

    # NOTE: currently unused
    # if area > 17000:
    #     rc.drive.set_speed_angle(0, 0)
    # else:
    #     pass
    # sends speed and angle values to racecar
    rc.drive.set_speed_angle(speed, cangle)
    
    # Assign prev values to calculate change in the next iteration
    prev_error=error
    prev_angle = angle
    #############################################################################################

# Called once per second
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
                if sign == "Car":
                    # calculates the center and area of the current bounding box
                    # the center is the present value, which is used to calculate the error
                    loc = (obj.bbox.xmin + obj.bbox.xmax)//2
                    area = (obj.bbox.xmin - obj.bbox.xmax ) * (obj.bbox.ymin - obj.bbox.ymax )
                    center = loc
                print(f"{tag[obj.id+1]}")
            sign = ""

if __name__ == "__main__":
    # sets the update slow time to be every 0.1 seconds 
    # meaning every 0.1 seconds, the car in front's center is updated
    rc.set_update_slow_time(0.1)
    rc.set_start_update(start, update, update_slow)
    rc.go()