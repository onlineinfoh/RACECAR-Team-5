# necessary imports and setting
import sys
import numpy as np
sys.path.insert(1, '../../library')
import racecar_core
import racecar_utils as rc_utils

# Create global variable and initialization
rc = racecar_core.create_racecar()
speed = 0
angle = 0 
prev_error=0

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    # Initialize speed and angle values
    global speed, angle  
    speed = 0
    angle = 0 

def update():
    # Global variable
    global speed, angle  
    global prev_error

    scan = rc.lidar.get_samples() # get lidar scan
    
    # Compute left and right distance, and then compute the error (right - left)
    forward_right_distance = rc_utils.get_lidar_average_distance(scan, 55, 20)
    forward_left_distance = rc_utils.get_lidar_average_distance(scan, 305, 20)
    error = forward_right_distance - forward_left_distance

    # get the change in time
    dt=rc.get_delta_time()

    # proportional factor and derivative factor
    kd= 0.6
    kp = 0.0085

    # Calculate the change in error
    derror=error-prev_error
    prev_error=error
    
    # Combine the proportional term and the derivative term to compute the angle 
    angle = kp*error + derror/dt*kd
    
    # Clamp the angle 
    angle=rc_utils.clamp(angle, -1.0,1.0)

    # Apply speed controller
    speed=1-0.23*np.abs(angle)
    
    # Set the values
    rc.drive.set_speed_angle(speed, angle)
def update_slow():
    pass

# Main function, DO NOT CHANGE
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()