"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-summer-labs

File Name: lab_2a.py

Title: BYOA (Build Your Own AHRS)

Author: Tianxi Liang

Purpose: The goal of this lab is to build and deploy a ROS node that can ingest
IMU data and return accurate attitude estimates (roll, pitch, yaw) that can then
be used for autonomous navigation. It is recommended to review the equations of
motion and axes directions for the RACECAR Neo platform before starting. Template
code has been provided for the implementation of a Complementary Filter.

Expected Outcome: Subscribe to the /imu and /mag topics, and publish to the /attitude
topic with accurate attitude estimations.
"""

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from sensor_msgs.msg import Imu, MagneticField # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
import numpy as np # type: ignore
import math
import time

# Clamp the angle value, in radians, within the range [0, 2pi]
def clamp_angle(angle):
    if angle < 0: 
        while angle<0:
            angle += 2*np.pi
    if angle > 2*np.pi:
        while angle > 2*np.pi:
            angle -= 2*np.pi
    return angle

# at the top of your file, _above_ CompFilterNode:

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

class CompFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # Set up subscriber and publisher nodes
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.publisher_attitude = self.create_publisher(Vector3, '/attitude', 10) # output as [roll, pitch, yaw] angles
        self.publisher_velocity = self.create_publisher(Vector3, '/velocity', 10)
        self.publisher_pose_estimate = self.create_publisher(Vector3, '/pose_estimate', 10)

        self.prev_time = self.get_clock().now() # initialize time checkpoint
        self.prev_time = float(self.prev_time.nanoseconds* 1e-9)

        # Gyro (angular velocity) should be 95%
        self.alpha = 0.95 # temporary value for complementary filter

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

        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0

        self.x_pos = 0.0
        self.y_pos = 0.0

        ### Kalman Filter
        self.kf_ax = OneDKalman(process_var=0.01, sensor_var=0.5)
        self.kf_ay = OneDKalman(process_var=0.01, sensor_var=0.5)
        self.kf_az = OneDKalman(process_var=0.01, sensor_var=0.5)


    # Called when new IMU data is received, attitude calculation completed here as well
    def imu_callback(self, data):

        # Grab linear acceleration and angular velocity values from subscribed data points
        # linear acceleration
        accel = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z] 
        print(f"linear accel: {accel}")

        # angular velocity
        gyro = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z] 
        print(f"angular velocity: {gyro}")

        # Calculate time delta
        now = [data.header.stamp.sec, data.header.stamp.nanosec] # Current ROS time
        dt = now[0] + 1e-9 * now[1] - self.prev_time  # Time 
        self.prev_time = now[0] + 1e-9 * now[1] # refresh checkpoint

        # tilt angle calculation using linear acceleration data
        tilt_angle = [np.arctan2(accel[1], accel[2]), np.arctan2(-accel[0], np.sqrt(accel[1] * accel[1] + accel[2] * accel[2])), 0]
        
        # Offset by 180 degrees to account for left hand rule and not right hand rule
        tilt_angle[0] = tilt_angle[0] + np.pi
        
        # tilt angles
        x = tilt_angle[0]
        y = tilt_angle[1]
    
        # Derive tilt angles from accelerometer
        #wrong
        #accel_roll = tilt_compensated_mag[0] # theta_x
        #accel_pitch = tilt_compensated_mag[1] # theta_y

        # TODO: Integrate gyroscope (angular velocity) to get attitude angles (theta)
        self.gyro_roll +=  dt * gyro[0] # theta_xt
        self.gyro_pitch += dt * gyro[1] # theta_yt
        self.gyro_yaw += dt * gyro[2]   # theta_zt

        # TODO: Compute yaw angle from magnetometer
        if self.mag:
            mx, my, mz = self.mag
            # since the tilt angle from the accelerometer is 0 
            # (since the gravity vector is not changed when rotating around the z axis)
            # the tilt needs to be compensated using the magnetometer values

            tilt_compensated_mag = [mx * np.cos(x) + my * np.sin(y) + mz * np.sin(x) * np.cos(y), 
                                    my * np.cos(y)-mz*np.sin(y),
                                    -mx * np.sin(x) + my * np.cos(x) * np.sin(y) + mz * np.cos(x) * np.cos(y)]
            
            mag_accel_yaw=np.arctan2(-tilt_compensated_mag[1],tilt_compensated_mag[0])


            # velocity calculation

            ################################################################
            # 1. With matrix compensation, tilt -> compensate for g, -> untilt
            # tilt to rotate the acceleration matrix
            tilt_b=np.array([
                [np.cos(x),np.sin(x)*np.sin(y),np.sin(x)*np.cos(y)],
                [0,np.cos(y),-np.sin(y)],
                [-np.sin(x),np.cos(x)*np.sin(y),np.cos(x)*np.sin(y)]
            ])

            # tilt the acceleration matrix to match the global axis
            accel_matrix=np.array(accel)
            tilt_compensated_accel=np.matmul(tilt_b, accel_matrix)
            tilt_compensated_accel[2]+=9.81

            ################################################################

            ################################################################
            # 2. Do not anything
            tilt_compensated_accel = accel
            ################################################################

            ################################################################
            # 3. Direct compensation
            g = 9.81
            tilt_compensated_accel = [
                accel[0] - g * np.sin(self.gyro_pitch),
                accel[1] - g * np.sin(self.gyro_roll),
                accel[2] - g * np.cos(self.gyro_pitch) * np.cos(self.gyro_roll)
            ]
            ################################################################
            
            # Kalman filter
            ax_f = self.kf_ax.update(float(tilt_compensated_accel[0]))
            ay_f = self.kf_ax.update(float(tilt_compensated_accel[1]))
            az_f = self.kf_ax.update(float(tilt_compensated_accel[2]))

            # Normal calculation
            self.v_x += dt * tilt_compensated_accel[0]
            self.v_y += dt * tilt_compensated_accel[1]
            self.v_z += dt * tilt_compensated_accel[2]

            # Kalman filter calculation
            self.v_x += dt * ax_f
            self.v_y += dt * ay_f
            self.v_z += dt * az_f
            
            print(f"raw accel: {tilt_compensated_accel}, filt accel: {[ax_f, ay_f, az_f]}")

            print(self.v_x, self.v_y. self.v_z)
        else:
            mag_accel_yaw = self.yaw
        
        # TODO: Fuse gyro, mag, and accel derivations in complementary filter
        self.roll  = self.alpha * self.gyro_roll + (1.0-self.alpha) * x
        self.pitch = self.alpha * self.gyro_pitch + (1.0-self.alpha) * y
        self.yaw = self.alpha * self.gyro_yaw + (1.0-self.alpha) * mag_accel_yaw

        # Print results for sanity checking

        print(f"====== Complementary Filter Results ======")
        print(f"Speed || Freq = {round(1/dt,0)} || dt (ms) = {round(dt*1e3, 2)}")
        print(f"Accel + Mag Derivation")
        print(f"Roll (deg): {x * 180/math.pi}")
        print(f"Pitch (deg): {y * 180/math.pi}")
        print(f"Yaw (deg): {mag_accel_yaw * 180/math.pi}")
        print()
        print(f"Gyro Derivation")
        print(f"Roll (deg): {self.gyro_roll * 180/math.pi}")
        print(f"Pitch (deg): {self.gyro_pitch * 180/math.pi}")
        print(f"Yaw (deg): {self.gyro_yaw * 180/math.pi}")
        print()
        print(f"Fused Results")
        print(f"Roll (deg): {self.roll * 180/math.pi}")
        print(f"Pitch (deg): {self.pitch * 180/math.pi}")
        print(f"Yaw (deg): {self.yaw * 180/math.pi}")
        print("\n")
        
        print(f"d_yaw: {self.yaw* 180.0 / np.pi}")

        print()
        
        # TODO: Publish to attitude topic (convert to degrees)
        attitude = Vector3()
        attitude.x = self.roll
        attitude.y = self.pitch
        attitude.z = self.yaw
        self.publisher_attitude.publish(attitude)

        velocity = Vector3()
        velocity.x = self.v_x
        velocity.y = self.v_y
        velocity.z = self.v_z
        self.publisher_velocity.publish(velocity)

        self.x_pos += dt * self.v_x
        self.y_pos += dt * self.v_y

        pose_estimate = Vector3()
        pose_estimate.x = self.x_pos
        pose_estimate.y = self.y_pos
        pose_estimate.z = self.yaw
        self.publisher_pose_estimate.publish(pose_estimate)
    
    def mag_callback(self, data):
        self.mag = data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z

def main():
    rclpy.init(args=None)
    node = CompFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()