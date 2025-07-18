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


    # Called when new IMU data is received, attitude calc completed here as well
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
        dt = now[0] + 1e-9 * now[1] - self.prev_time  # Time delta
        print(f"dt: {dt} seconds")
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

            tilt_b=np.array([np.cos(x),np.sin(x)*np.sin(y),np.sin(x)*np.cos(y)],[0,np.cos(y),-np.sin(y)],[-np.sin(x),np.cos(x)*np.sin(y),np.cos(x)*np.sin(y)])
            tilt_b_inv=np.linalg.inv(tilt_b)
            accel_matrix=np.array(accel)
            tilt_compensated_accel=np.matmul(tilt_b_inv, accel_matrix)
            tilt_compensated_accel[2]+=9.81
            print(tilt_compensated_accel)
        else:
            mag_accel_yaw = self.yaw
        
        # TODO: Fuse gyro, mag, and accel derivations in complementary filter
        self.roll  = self.alpha * self.gyro_roll + (1.0-self.alpha) * x
        self.pitch = self.alpha * self.gyro_pitch + (1.0-self.alpha) * y
        self.yaw = self.alpha * self.gyro_yaw + (1.0-self.alpha) * mag_theta_z

        # Print results for sanity checking

        print(f"====== Complementary Filter Results ======")
        print(f"Speed || Freq = {round(1/dt,0)} || dt (ms) = {round(dt*1e3, 2)}")
        print(f"Accel + Mag Derivation")
        print(f"Roll (deg): {accel_roll * 180/math.pi}")
        print(f"Pitch (deg): {accel_pitch * 180/math.pi}")
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