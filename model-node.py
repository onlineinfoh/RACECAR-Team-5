import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy as np
import cv2 as cv
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

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

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'sign-detect-v1_edgetpu.tflite'
label_name = 'sign-detect-objects.txt'

model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 9

print("importing model...")
interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()
labels = read_label_file(label_path)
inference_size = input_size(interpreter)

print("model imported")

tag = {1:'do not enter',2:'fake go around',3:'fake stop',4:'fake yield',5:'go around',
       6:'one way left',7:'one way right',8:'stop sign',9:'yield'}

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')

        self.subscription_cam = self.create_subscription(Image, '/camera', self.camera_callback, 10)
        self.publisher_id = self.create_publisher(Vector3, '/id', 10)

    def camera_callback(self, data):
        #frame = rc.get_color_image()
        frame = data.data
        frame = np.frombuffer(frame,np.uint8)
        frame = cv.imdecode(frame,cv.IMREAD_COLOR)
        #print("frame decoded from jpeg")
        frame = cv.cvtColor(frame,cv.COLOR_BGR2RGB)
        if frame is not None:
            rgb_image = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            rgb_image = cv.resize(rgb_image, inference_size)
            
            run_inference(interpreter, rgb_image.tobytes())
            objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]

            #print(objs)
            for obj in objs:
                if obj.score > 0.7:
                    print(obj)
                    print(f"{tag[obj.id+1]}")
        
def main():
    rclpy.init(args=None)
    node = CamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()