#!/usr/bin/env python3
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore

class CommReceiver(Node):
    def __init__(self):
        super().__init__('comm_receiver')
        self.subscription = self.create_subscription(Vector3,'/attitude',self.attitude_callback,10)
        self.subscription = self.create_subscription(Vector3,'/velocity',self.velocity_callback,10)
        self.subscription = self.create_subscription(Vector3,'/pose_estimate',self.pose_estimate_callback,10)
        self.subscription  # prevent unused-variable warning

    def attitude_callback(self, data: Vector3):
        roll = data.x
        pitch = data.y
        yaw = data.z
        self.get_logger().info(f"roll: {roll} | pitch: {pitch} | yaw: {yaw}")

    def velocity_callback(self, data: Vector3):
        vx = data.x
        vy = data.y
        vz = data.z
        self.get_logger().info(f"vx: {vx} | vy: {vy} | vz: {vz}")

    def pose_estimate_callback(self, data: Vector3):
        xpos = data.x
        ypos = data.y
        angle = data.z
        self.get_logger().info(f"x pos: {xpos} | y pos: {ypos} | angle: {angle}")

def main(args=None):
    rclpy.init(args=args)
    node = CommReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()