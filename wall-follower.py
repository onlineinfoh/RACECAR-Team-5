# necessary imports and setting
import sys
import numpy as np # type: ignore 
sys.path.insert(1, 'library')
import racecar_core # type: ignore 
import racecar_utils as rc_utils # type: ignore 

# Create global variable and initialization
rc = racecar_core.create_racecar()
speed = 0
angle = 0 
prev_error=0

prev_angle = 0

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Initialize speed and angle values
    global speed, angle
    speed = 0
    angle = 0 


def update():
    # Global variable
    global speed, angle
    global prev_error, prev_angle

    scan = rc.lidar.get_samples() # get lidar scan

    # Compute left and right distance, and then compute the error (right - left)
    forward_right_distance = rc_utils.get_lidar_average_distance(scan, 50, 10)
    forward_left_distance = rc_utils.get_lidar_average_distance(scan, 310, 10)
    error = forward_right_distance - forward_left_distance

    # get the change in time
    dt=rc.get_delta_time()

    # proportional factor and derivative factor
    kd = 0.004 # 0.003
    kp = 0.005 # 0.005
    # Calculate the change in error
    derror=error-prev_error

    # Combine the proportional term and the derivative term to compute the angle
    angle = kperror + derror/dt * kd
    # print(angle)
    # Clamp the angle 
    prev_angle = angle
    dangle = angle - prev_angle
    print(f"Speed: {speed} Angle: {angle}")

    kps = 0.245

    cangle=rc_utils.clamp(angle, -1.0,1.0)
    
    # Apply speed controller
    speed = 1 - kps np.abs(cangle)
    rc.drive.set_speed_angle(speed, cangle)
    prev_error=error

def update_slow():
    pass

# Main function, DO NOT CHANGE
if name == "main":
    rc.set_start_update(start, update, update_slow)
    rc.go()