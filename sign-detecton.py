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
model_name = 'sign-detect-v1_edgetpu.tflite'
label_name = 'sign-detect-objects.txt'

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

sign = ""

center = 0

tag = {1:'do not enter',2:'fake go around',3:'fake stop',4:'fake yield',5:'go around',6:'one way left',7:'one way right',8:'stop sign',9:'yield'}
                
# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if obj.score > 0.6:
            
            print(f"{tag[obj.id+1]}")
            
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

            cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv.putText(cv2_im, label, (x0, y0+30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            break
    return cv2_im

# Called once on start
def start():
    rc.drive.set_speed_angle(0, 0)
    rc.set_update_slow_time(0.5)

# Called every frame
def update():
    time_cp1 = time.time()
    
    global speed, angle  
    global prev_error

    scan = rc.lidar.get_samples() # get lidar scan
    
    # Compute left and right distance, and then compute the error (right - left)
    forward_right_distance = rc_utils.get_lidar_average_distance(scan, 50, 30)
    forward_left_distance = rc_utils.get_lidar_average_distance(scan, 310, 30)
    error = forward_right_distance - forward_left_distance

    # get the change in time
    dt=rc.get_delta_time()

    # proportional factor and derivative factor
    kd = 0.003 # 0.003
    kp = 0.0055 # 0.005
    # Calculate the change in error
    derror=error-prev_error
    
    # Combine the proportional term and the derivative term to compute the angle 
    angle = kp*error + derror/dt*kd
    # print(angle)
    # Clamp the angle 
    prev_angle = angle
    dangle = angle - prev_angle

    kps = 0.1
    # kds = 0.1*0
    
    cangle=rc_utils.clamp(angle, -1.0,1.0)
    # Apply speed controller
    #speed= 1- kps * np.abs(cangle) +dangle/dt *kds
    speed = 0.7 - kps * np.abs(cangle)
    # if np.abs(angle) > 0.7:
    #     speed = 0.8
    if sign=="stop sign":
        rc.drive.set_speed_angle(0, 0)
    elif sign == "yield":
        rc.drive.set_speed_angle(0.1, 0)
    elif sign == "go around":
        if center>=160:
            rc.drive.set_speed_angle(0.3, -0.7)
        else:
            rc.drive.set_speed_angle(0.3, 0.7)
    else:
        rc.drive.set_speed_angle(speed, cangle)
        
    prev_error=error
#############################################################################################

# Called once per second
def update_slow():
    global center
    global sign
    frame = rc.camera.get_color_image()
    
    if frame is not None:
        rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb_image = cv.resize(rgb_image, inference_size)
        
        run_inference(interpreter, rgb_image.tobytes())
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]
        image = append_objs_to_img(frame, inference_size, objs, labels)

        rc.display.show_color_image(image)
        if len(objs) == 0:
            sign = ""
        for obj in objs:
            if obj.score > 0.6:
                sign = tag[obj.id+1]
                if sign == "go around":
                    loc = (obj.bbox.xmin + obj.bbox.xmax)//2
                    center = loc
                print(f"{tag[obj.id+1]}")
                break
            sign = ""

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()