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
NUM_CLASSES = 3


# STEP 1: Load model and labels using pycoral.utils
print('Loading {} with {} labels.'.format(model_path, label_path))
interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()
labels = read_label_file(label_path)
inference_size = input_size(interpreter)

# Initialize car
rc = racecar_core.create_racecar()

# Global variables
contour_center = None
contour_area = 0

# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if obj.score > 0.6: # only draw item if confidence > 75%
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)

            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

            cv2_im = cv.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv.putText(cv2_im, label, (x0, y0+30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

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

    time_cp1 = time.time()
    
    frame = rc.camera.get_color_image()
    
    if frame is not None:
        
        # STEP 4: Preprocess image to the size and shape accepted by model
        rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        rgb_image = cv.resize(rgb_image, inference_size)

        # STEP 5: Let the model do the work
        time_cp2 = time.time()
        run_inference(interpreter, rgb_image.tobytes())
        time_cp3 = time.time()
        
        # STEP 6: Get objects detected from the model
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]

        # STEP 7: Label detected objects to frame
        image = append_objs_to_img(frame, inference_size, objs, labels)

        rc.display.show_color_image(image)

        time_cp4 = time.time()

        # Calculate delays
        fps = round(1/(time_cp4 - time_cp1), 2)
        inference = round((time_cp3 - time_cp2) * 1000, 2)
        print(f"Speed = {fps} fps || Inference = {inference} ms")

# Called once per second
def update_slow():
    pass

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()