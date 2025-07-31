# necessary imports and setting
import sys
import numpy as np # type: ignore 
sys.path.insert(1, '../../library')
import racecar_core # type: ignore 
import racecar_utils as rc_utils # type: ignore 
import math
# Create global variable and initialization
rc = racecar_core.create_racecar()
speed = 0
angle = 0 
prev_error=0
deadzone_deg = 10                   # no steering for |angle| < 10°
alpha = 0.3 
prev_angle = 0
conversion_factor_sim = 2
conversion_factor_life = 3
def find_farthest_point():
    scan = rc.lidar.get_samples()
    angle = 0.0
    front_scan = []

    for reading in scan:
        # increment raw angle, then wrap to [0,360)
        if reading == 0:
            reading = 1000000000
        angle += 1/conversion_factor_sim
        # keep only samples in the “front” sector
        if angle < 90 or angle > 270:
            # convert to signed range [-180,180]
            signed_angle = angle if angle <= 180 else angle - 360
            front_scan.append([signed_angle, reading])

    arr = np.array(front_scan)           # shape: (N,2)
    distances = arr[:, 1]
    max_dist = np.max(distances)

    # find the indices of all occurrences of that max distance
    equals = np.where(distances == max_dist)[0]

    # average their signed angles
    angle_of_farthest_point = float(np.mean(arr[equals, 0])) 

    return angle_of_farthest_point, max_dist
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
    global conversion_factor_sim, conversion_factor_life
    # list of everything in a 180 degree radius
    global alpha, deadzone_deg
    Kp_ang = 0.01      # rad steering per rad error
    Kp_dist = 0.5      # rad steering per meter of lateral error
    desired_d = 0.5
    wheelbase = 20
    angle_of_farthest_point, distance = find_farthest_point()
    print(angle_of_farthest_point)
    theta = math.radians(angle_of_farthest_point)
    d_center = distance
    L = wheelbase/2
    d_bumper = d_center*math.cos(theta) - L

    offset = 0.3

    kp = 1/55.

    turning_angle = angle_of_farthest_point * kp

    if angle_of_farthest_point > 10 or angle_of_farthest_point < -10:
        offset = 0.2
    # elif angle_of_farthest_point > 5 or angle_of_farthest_point < -5:
    #     offset = 0.1
    else: 
        offset = 0.

    offset = np.abs(angle_of_farthest_point) * (0.03 if np.abs(angle_of_farthest_point) < 20 else (0.006 if np.abs(angle_of_farthest_point) < 35 else 0))  
    + np.log(distance) * 0.01

    if turning_angle<0:
        turning_angle += offset
    elif turning_angle>0:
        turning_angle -= offset

    if turning_angle < -1:
        turning_angle = -1
    elif turning_angle > 1:
        turning_angle = 1

    
    
    rc.drive.set_speed_angle(1, turning_angle)

def update_slow():
    pass

# Main function, DO NOT CHANGE
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()