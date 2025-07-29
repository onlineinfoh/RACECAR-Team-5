"""
MIT BWSI Autonomous RACECAR
MIT License

Title: velocity calculation node

Purpose: ROS2 node that takes IMU linear acceleration data and integrates it and fuses it with velocity
data derived from measuring optical flow in an image from the LIDAR. 

Expected Outcome: Subscribe to the /imu and /mag topics, and publish to the /velocity
topic with accurate velocity estimations.
"""

#!/usr/bin/env python3
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
from sensor_msgs.msg import Imu, MagneticField, LaserScan # type: ignore
import numpy as np # type: ignore
import math
import cv2 as cv # type: ignore

print("starting imu node sender...")

class OneDKalman:
    def __init__(self, process_var=0.01, sensor_var=1.0):
        # Q = process noise variance, R = measurement noise variance
        self.Q = process_var
        self.R = sensor_var
        # state estimate and covariance
        self.x = 0.0
        self.P = 1.0

    def update(self, z: float) -> float:
        # 1) compute Kalman gain
        K = self.P / (self.P + self.R)
        # 2) update estimate
        self.x = self.x + K * (z - self.x)
        # 3) update covariance
        self.P = (1 - K) * self.P + self.Q
        return self.x


class VelocityNode(Node):
    def __init__(self):
        super().__init__("send_node")

        # Set up subscriber
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # publisher nodes
        self.publisher_velocity = self.create_publisher(Vector3, '/velocity', 10) 
        self.publisher_attitude = self.create_publisher(Vector3, '/attitude', 10) 
        self.publisher_pose_estimate = self.create_publisher(Vector3, '/pose_estimate', 10)  

        # set up timer to send info
        self.create_timer(0.,self.imu_callback)

        # variables initializations for pose calculations
        self.prev_time = self.get_clock().now().nanoseconds / 10**9 # initialize time checkpoint
        self.prev_time_lidar=0

        self.counter=0

        # intialize gyro stuff
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.prev_accel_x = 0.0
        self.prev_accel_y = 0.0
        self.prev_accel_z = 0.0

        self.accel_velocity_x=0.0
        self.accel_velocity_y=0.0
        self.accel_velocity_z=0.0

        self.mag = None

        self.lidar_velocity_x=0.0
        self.lidar_velocity_y=0.0

        # set up velocity
        self.x_velocity = 0.0
        self.y_velocity = 0.0
        self.z_velocity = 0.0

        # Gyro (angular velocity) should be 95%
        self.alpha = 0.95 # temporary value for complementary filter
        self.vel_alpha=1

        # set up attitude params
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.mag = 0.0,0.0,0.0

        self.gyro_roll = 0.0
        self.mag_roll = 0.0

        self.gyro_pitch = 0.0
        self.mag_pitch = 0.0

        self.gyro_yaw = 0.0
        self.mag_yaw = 0.0

        self.velocity_p=0.0

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.avg_x_velocity=[0,0,0]
        self.avg_y_velocity=[0,0,0]

        ### Kalman Filter
        self.kf_ax = OneDKalman(process_var=3.0, sensor_var=0.01)
        self.kf_ay = OneDKalman(process_var=3.0, sensor_var=0.02)
        self.kf_az = OneDKalman(process_var=1.0, sensor_var=0.01)

        self.kf_px = OneDKalman(process_var=3.0, sensor_var=0.01)
        self.kf_py = OneDKalman(process_var=3.0, sensor_var=0.02)

        # lidar stuff
        # params for ShiTomasi corner detection
        self.feature_params = dict( maxCorners = 100,
                            qualityLevel = 0.3,
                            minDistance = 7,
                            blockSize = 7 )
        
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (30, 30),
                        maxLevel = 2,
                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        
        self.prev_lidar_map = cv.cvtColor(np.zeros((480, 640, 3), dtype=np.uint8), cv.COLOR_BGR2GRAY) # blank image

        self.prev_lidar_corners = cv.goodFeaturesToTrack(self.prev_lidar_map, mask=None, **self.feature_params)
        self.mean_disp = [0,0]

    def imu_callback(self, data):
        #print("imu_callback") 
        ## THE FIRST BIT IS GETTING ATTITUDE, NECESSARY FOR GRAVITY

        # Grab linear acceleration and gyroscope values from subscribed data points
        accel = [data.linear_acceleration.x,data.linear_acceleration.y,data.linear_acceleration.z]
        gyro = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]

        # Calculate time delta
        now = self.get_clock().now().nanoseconds / 10**9 # Current ROS time
        dt = now - self.prev_time # Time delta
        self.prev_time = now # refresh checkpoint
    
        # Derive tilt angles from accelerometer
        # tilt angle calculation using linear acceleration data
        tilt_angle = [np.arctan2(accel[1], accel[2]), np.arctan2(-accel[0], np.sqrt(accel[1] * accel[1] + accel[2] * accel[2])), 0]
        
        accel_roll = tilt_angle[0] # theta_x
        accel_pitch = tilt_angle[1]
        accel_roll+=np.pi

        # Integrate gyroscope to get attitude angles
        self.gyro_roll = (self.gyro_roll + gyro[0]*dt) #% 2*np.pi # theta_xt
        self.gyro_pitch = (self.gyro_pitch + gyro[1]*dt) #% 2*np.pi # theta_yt
        self.gyro_yaw = (self.gyro_yaw + gyro[2]*dt) #% 2*np.pi # theta_zt

        # Compute yaw angle from magnetometer
        if self.mag:
            mx, my, mz = self.mag
            x=accel_roll
            y=accel_pitch
            #print(f"Mag norm (~50 uT): {math.sqrt(mx**2 + my**2 + mz**2) * 1e6}") # used for checking magnetic disturbances/offsets
            # now for the magic:
            tilt_compensated_mag = [mx * np.cos(x) + my * np.sin(y) + mz * np.sin(x) * np.cos(y), 
                                    my * np.cos(y)-mz*np.sin(y),
                                    -mx * np.sin(x) + my * np.cos(x) * np.sin(y) + mz * np.cos(x) * np.cos(y)]
            
            mag_accel_yaw=np.arctan2(-tilt_compensated_mag[1],tilt_compensated_mag[0])
        else:
            mag_accel_yaw = self.yaw
        
        # Fuse gyro, mag, and accel derivations in complemtnary filter
        self.roll = self.alpha*self.gyro_roll + (1-self.alpha)*accel_roll
        self.pitch = self.alpha*self.gyro_pitch + (1-self.alpha)*accel_pitch
        self.yaw = (self.alpha*self.gyro_yaw + (1-self.alpha)*mag_accel_yaw)

        #print(self.roll *180/np.pi, self.pitch*180/np.pi, self.yaw*180/np.pi)

        ## START OF VELOCITY EXCLUSIVE PROCESSING ##
        
        # calculate gravity vector based on attitude, then remove it from linear acceleration
        g = -9.81
        gravity = np.array([
            g * np.sin(self.gyro_pitch),
            g * np.sin(self.gyro_roll),
            g * np.cos(self.gyro_pitch) * np.cos(self.gyro_roll)
        ])
        
        accel_array = accel # acceleration from vector3 to  np array to do operations
        no_gravity = accel_array - gravity

        #print("raw accel", accel)
        #print("gravity", gravity)

        #print(no_gravity)

        # calc new xyz velocity by integrating (trapezoid sum not rectangles i forget the name)
        # the accel_ is there because these are the points calculated using the accelerometer
        # however, these are velocity values
        ax_f=self.kf_ax.update(float(no_gravity[0]))
        ay_f=self.kf_ay.update(float(no_gravity[1]))
        az_f=self.kf_az.update(float(no_gravity[2]))

        #print("kalman velocity", ax_f, ay_f, az_f)

        if np.abs(ax_f)<0.1:
            ax_f=0.0
        if np.abs(ay_f)<0.1:
            ay_f=0.0
        if np.abs(az_f)<0.1:
            az_f=0.0

        #print("time", dt)

        self.accel_velocity_x+=ax_f*dt
        self.accel_velocity_y+=ay_f*dt
        self.accel_velocity_z+=az_f*dt

        #print("accel velocity 0", self.accel_velocity_x, self.accel_velocity_y, self.accel_velocity_z)
        #accel_velocity_x = self.x_velocity + 0.5 * (no_gravity[0] + self.prev_accel_x) * dt
        #accel_velocity_y = self.y_velocity + 0.5 * (no_gravity[1] + self.prev_accel_y) * dt
        #accel_velocity_z = self.z_velocity + 0.5 * (no_gravity[2] + self.prev_accel_z) * dt

        # lidar optical flow velocity finding method starts hereee
        # if scan data is ok, collect map

        if hasattr(self, 'scan_data') and self.scan_data is not None:
            #print("scan")
            now_lidar = data.header.stamp.sec + data.header.stamp.nanosec*1e-9
            if self.prev_time_lidar is None:
                self.prev_time_lidar = now_lidar
                return
            dt_lidar = now_lidar - self.prev_time_lidar
            self.prev_time_lidar = now_lidar
            #print("yay")
            if np.sum(self.scan_data) > 0:
                pass
            lidar_map = self.get_lidar_image(self.scan_data)
            lidar_gray = cv.cvtColor(lidar_map, cv.COLOR_BGR2GRAY)

            # If this is the first frame, or we lost our corners, just detect and wait
            if self.prev_lidar_corners is None or len(self.prev_lidar_corners) < 3 or self.prev_lidar_map is None:
                self.prev_lidar_corners = cv.goodFeaturesToTrack(lidar_gray, mask=None, **self.feature_params)
                self.prev_lidar_map = lidar_gray
                return
            
            # Track existing corners
            current_lidar_corners, st, err = cv.calcOpticalFlowPyrLK(
                self.prev_lidar_map,
                lidar_gray,
                self.prev_lidar_corners,
                None,
                **self.lk_params
            )

            if current_lidar_corners is not None and st is not None:
                good_new = current_lidar_corners[st == 1]
                good_old = self.prev_lidar_corners[st == 1]

                if len(good_new) >= 1:
                    displacements = good_new - good_old
                    self.mean_disp = np.mean(displacements, axis=0)
                    if abs(np.sum(self.mean_disp)) > .001:
                        #print(f"Mean optical flow dx, dy: {self.mean_disp.flatten()}")
                        self.lidar_velocity_x, self.lidar_velocity_y = self.mean_disp.flatten()
                        
                    # Update for next round
                    self.prev_lidar_map = lidar_gray
                    self.prev_lidar_corners = good_new.reshape(-1, 1, 2)
                else:
                    # If all tracking lost, reinitialize
                    self.prev_lidar_corners = cv.goodFeaturesToTrack(lidar_gray, mask=None, **self.feature_params)
                    self.prev_lidar_map = lidar_gray
            else:
                # If flow failed, reinitialize
                self.prev_lidar_corners = cv.goodFeaturesToTrack(lidar_gray, mask=None, **self.feature_params)
                self.prev_lidar_map = lidar_gray

            #print("accel vel 1", self.accel_velocity_x, self.accel_velocity_y, self.accel_velocity_z)
            if 0<np.abs(self.lidar_velocity_x)<0.07:
                #print("x decrease")
                self.accel_velocity_x=(0+self.x_velocity)*0.3
            if 0<np.abs(self.lidar_velocity_y)<0.13:
                self.accel_velocity_y=(0+self.y_velocity)*0.3
                #print("y decrease")

            #print("accel vel 2", self.accel_velocity_x, self.accel_velocity_y, self.accel_velocity_z)

        # if the car is still, lidar_velocity is approx 0
        # otherwise, lidar_velocity is random
        
        change_x_velocity=(self.vel_alpha*self.accel_velocity_x + (1-self.vel_alpha)*self.lidar_velocity_x)
        change_y_velocity=(self.vel_alpha*self.accel_velocity_y + (1-self.vel_alpha)*self.lidar_velocity_y)
        self.x_velocity-=change_x_velocity
        self.y_velocity-=change_y_velocity
        self.z_velocity=0

        self.avg_x_velocity.pop(0)
        self.avg_x_velocity.append(self.x_velocity)

        self.avg_y_velocity.pop(0)
        self.avg_y_velocity.append(self.y_velocity)

        if abs(min(self.avg_x_velocity)) >0.05:
            #print("x velocity")
            self.x_pos += self.x_velocity*dt
        if abs(min(self.avg_y_velocity))>0.05:
            #print("y velocity")
            self.y_pos +=self.y_velocity*dt
        #print(self.avg_x_velocity)

        #self.x_pos=self.kf_px.update(self.x_pos)
        #self.y_pos=self.kf_py.update(self.y_pos)


        self.counter+=1
        if self.counter % 4 == 0:
            print("lidar vel", sum(self.avg_x_velocity)/5, self.lidar_velocity_y)
            #print("lidar vel", self.lidar_velocity_x, self.lidar_velocity_y)
            print("vel:", self.x_velocity, self.y_velocity, self.z_velocity)
            print("pos", self.x_pos, self.y_pos)
        
        
        # publish attitude
        attitude = Vector3()
        attitude.x = self.roll
        attitude.y = self.pitch
        attitude.z = self.yaw
        self.publisher_attitude.publish(attitude)

        # publish velocity
        velocity = Vector3()
        velocity.x = self.x_velocity
        velocity.y = self.y_velocity
        velocity.z = self.z_velocity
        self.publisher_velocity.publish(velocity)

        # publish pose estimate
        pose_estimate = Vector3()
        pose_estimate.x = self.x_pos
        pose_estimate.y = self.y_pos
        pose_estimate.z = self.yaw
        self.publisher_pose_estimate.publish(pose_estimate)
        

    # [FUNCTION] Called when magnetometer topic receives an update
    def mag_callback(self, data):
        # Assign self.mag to the magnetometer data points
        self.mag = (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z,)

    def lidar_callback(self, data):
        scan_data_orig = np.flip(np.multiply(np.array(data.ranges), 100))
        self.scan_data = np.array([0 if str(x) == "inf" else x for x in scan_data_orig])

    def get_lidar_image(self, scan):
        # Convert polar to Cartesian
        angles_deg = np.arange(0, 360, 1/3)
        angles_rad = np.deg2rad(angles_deg)
        distances = np.array(scan)

        # Filter valid points
        valid = distances > 0
        distances = distances[valid]
        angles_rad = angles_rad[valid]

        x = distances * np.sin(angles_rad)
        y = distances * np.cos(angles_rad)

        # Create blank image
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Transform points to fit image coords (center at 320,400 and scale)
        scale = 0.2  # adjust for zoom
        x_img = (x * scale + 320).astype(np.uint8)
        y_img = (320 - y * scale).astype(np.uint8)

        # Clip to image bounds
        valid_idx = (x_img >= 0) & (x_img < 640) & (y_img >= 0) & (y_img < 480)
        x_img = x_img[valid_idx]
        y_img = y_img[valid_idx]
        x = x[valid_idx]
        y = y[valid_idx]

        # Draw lidar points
        for xi, yi in zip(x_img, y_img):
            cv.circle(img, (xi, yi), radius=1, color=(255, 255, 255), thickness=-1)  # white

        # Draw robot at center (red dot)
        cv.circle(img, (320, 320), radius=4, color=(0, 0, 255), thickness=-1)  # red
        return img

def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 