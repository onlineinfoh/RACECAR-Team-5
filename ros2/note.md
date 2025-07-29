TO UPDATE THE ROS CONFIG

change setup.py in racecar_ws/src/racecar_neo/ folder

colcon build
source ~/.bashrc 

sender run: ros2 run racecar_neo send
receiver run: ros2 run racecar_neo receive