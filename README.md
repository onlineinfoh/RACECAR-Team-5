# RACECAR TEAM 5

**MIT Beaver Works Summer Institute (BWSI) Autonomous RACECAR – Team 5 (Tianxi Liang, Aidan Wong, Claire Shen, Krista Sebastian)**
## Week 4 quest log challenge attempting:
* **Github documentation
* **Object classifier
* **Object detector
* **Dynamic obstacle(elevator)
* **Double sensor usage for the final grand-prix code

## Structure

```
RACECAR-Team-5/
├── data_zip/        # Archived training datasets
├── models/          # TFLite models and label files
├── ros2/            # ROS2 nodes and memo
├── *.py             # Autonomous behaviors algorithms
└── README.md        # Project overview
```

## Autonomous Behaviors

* **ar-tag.py**           AR tags detection and info extraction script
* **car-follow.py**      Car following(conga line) using TFLite model inference
* **cone-slalom.py**      Cone slalom, using color and LIDAR
* **line-follower.py**    Color lane following
* **wall-follower.py**    LIDAR and PID wall following
* **grandprix.py**        Final RACECAR 2025 grand prix submission
* **sign-detecton.py**    Traffic sign detection using TFLite models
* **teamlogo.py**         Matrix display of team logo
* **imu.py**              IMU attitude, velocity and pose estimation calculation

## Utilities

* **data-collection.py**  Simplified data collection script
* **img-counter.py**      Counts images all the directories specified

## Models

* **Car Follow**

  * `car-follow-v1.tflite`
  * `car-follow-v1_edgetpu.tflite`
  * `car-follow-v2.tflite`
  * `car-follow-v2_edgetpu.tflite`

* **Sign Detect**

  * `sign-detect-v1.tflite`
  * `sign-detect-v1_edgetpu.tflite`
  * `sign-detect-v2.tflite`
  * `sign-detect-v2_edgetpu.tflite`

* **Elevator**

  * `elevator.tflite`
  * `elevator_edgetpu.tflite`

* **Labels**

  * `objects.txt`                Car following label mapping
  * `sign-detect-objects.txt`    Traffic sign labels
  * `elevator-objects.txt`       Elevator dataset labels

## ROS 2 Nodes (`ros2/`)

* **send.py**       ROS2 publisher node
* **receive.py**    ROS2 subscriber node
* **note.md**       Usage note 

## Data zip folder (`data_zip/`)

* **car-follow-data.zip**         Car follow training data
* **v2-car-follow-data.zip**      Car follow training data with more images added
* **sign-detect-data.zip**        Sign detection training data
* **v2-sign-detect-data.zip**     Sign detection training data with more images added
* **elevator.zip**                Go/Stop sign detection training data for grand prix elevator challenge 