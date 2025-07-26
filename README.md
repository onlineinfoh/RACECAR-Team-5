# RACECAR TEAM 5 

Developed by: Tianxi Liang, Aidan Wong, Claire Shen, Krista Sebastian

Welcome to the repository of Team 5 from the MIT BWSI RACECAR program. This repository contains algorithms for car following, lane detection, wall following, and traffic sign recognition using ROS2, Python, and TensorFlow Lite.

---

### Models
- **`car-follower-v1.tflite` / `car-follower-v1_edgetpu.tflite`**  
  TensorFlow Lite models for car-following behavior. The EdgeTPU version is optimized for Coral accelerators.
- **`objects.txt`**  
  List of object labels corresponding to the model’s outputs.

---

### Python Scripts

#### Autonomous Behaviors
- **`car-follower-runner.py`**  
  Runs car-following model with live camera input.
- **`cone-slalom.py`**  
  Controls the car through a cone slalom course using color detection and Lidar.
- **`line-follower.py`**  
  Lane-following via color detection.
- **`wall-follower.py`**  
  Keeps a consistent distance from a wall using LIDAR and PID control.

#### Utilities
- **`data-collection.py`**  
  Captures image and sensor data for training.
- **`img-counter.py`**  
  Utility for counting the number images for training.
- **`model-node.py`**  
  ROS2 node to load and run ML inference (car-follow or sign-detect models).
- **`ros2.py`**  
  ROS2 helper node for IMU pose estimates

---

### Data Archives
- **`car-follow-data.zip`**  
  Collected dataset for car-following model training.
- **`sign-detect-data.zip`**  
  Dataset for traffic sign detection and classification.