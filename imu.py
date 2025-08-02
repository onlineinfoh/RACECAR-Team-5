import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
from sensor_msgs.msg import Imu, MagneticField, LaserScan # type: ignore
import numpy as np # type: ignore
import math
import cv2 as cv # type: ignore

print("starting imu node sender...")

# kalman filter
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

        # Subscriptions
        self.subscription_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.subscription_mag = self.create_subscription(MagneticField, '/mag', self.mag_callback, 10)
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publishers
        self.publisher_velocity = self.create_publisher(Vector3, '/velocity', 10)
        self.publisher_attitude = self.create_publisher(Vector3, '/attitude', 10)
        self.publisher_pose_estimate = self.create_publisher(Vector3, '/pose_estimate', 10)

        # Timing
        self.prev_time_lidar = None

        # Attitude
        self.alpha = 0.95
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.gyro_roll = 0.0; self.gyro_pitch = 0.0; self.gyro_yaw = 0.0
        self.mag = (0.0, 0.0, 0.0)

        # Velocity and position
        self.x_velocity = 0.0; self.y_velocity = 0.0; self.z_velocity = 0.0
        self.x_pos = 0.0; self.y_pos = 0.0

        # Lidar optical flow setup
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(30,30), maxLevel=2,
                              criteria=(cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        self.prev_lidar_map = cv.cvtColor(np.zeros((480, 640, 3), dtype=np.uint8), cv.COLOR_BGR2GRAY)
        self.prev_lidar_corners = cv.goodFeaturesToTrack(self.prev_lidar_map, mask=None, **self.feature_params)
        self.lidar_velocity_x = 0.0; self.lidar_velocity_y = 0.0
        self.scan_data = None

    def imu_callback(self, data):
        # Update attitude for yaw reference
        accel = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
        gyro = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]

        # Integrate gyro for attitude
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, 'prev_time'): 
            self.prev_time = now
        dt = now - self.prev_time
        self.prev_time = now
        self.gyro_roll += gyro[0] * dt
        self.gyro_pitch += gyro[1] * dt
        self.gyro_yaw += gyro[2] * dt

        # Simple complementary filter for yaw using magnetometer
        mx, my, mz = self.mag
        yaw_mag = math.atan2(-my, mx)
        self.yaw = self.alpha * self.gyro_yaw + (1 - self.alpha) * yaw_mag

        # Only start if there’s a lidar scan
        # Lidar-based velocity (body frame) -> rotate to global frame
        if self.scan_data is not None:
            stamp = data.header.stamp if hasattr(data, 'header') else None
            now_lidar = (stamp.sec + stamp.nanosec * 1e-9) if stamp else now
            if self.prev_time_lidar is None:
                self.prev_time_lidar = now_lidar
            else:
                dt_lidar = now_lidar - self.prev_time_lidar
                self.prev_time_lidar = now_lidar

                # Compute optical flow
                # turns lidar map into image 
                # takes into account the robot's yaw so absolute velocity (not js relative to 
                lidar_map = self.get_lidar_image(self.scan_data)

                # use cv to convert the RGB image to grayscale (bc required by optical flow)
                gray = cv.cvtColor(lidar_map, cv.COLOR_BGR2GRAY)

                # if prev detection didnt have enough corners, detect more corners
                # because need prev and current corners to get displacement (to get velocity)
                # (Shi–Tomasi corner detection)
                if self.prev_lidar_corners is None or len(self.prev_lidar_corners) < 3:
                    self.prev_lidar_corners = cv.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
                    self.prev_lidar_map = gray
                else:
                    # track the PREVIOUSLY detected corners in the NEW frame
                    # (Lucas–Kanade optical flow)
                    new_pts, st, err = cv.calcOpticalFlowPyrLK(self.prev_lidar_map, gray,
                                                               self.prev_lidar_corners, None,
                                                               **self.lk_params)
                    # If tracking succeeded, filter out the good points
                    if new_pts is not None and st is not None:
                        #  If we have at least one tracked point, compute the average displacement
                        good_new = new_pts[st == 1]; good_old = self.prev_lidar_corners[st == 1]
                        if len(good_new) > 0:
                            disp = np.mean(good_new - good_old, axis=0)
                            # Body-frame velocities (pixels per frame)
                            vx_b, vy_b = disp.flatten()
                            # Convert to real-world units if scaling known; here use as-is
                            # Rotate to global frame using current yaw
                            cos_y = math.cos(self.yaw)
                            sin_y = math.sin(self.yaw)
                            #self.x_velocity = vx_b * cos_y - vy_b * sin_y
                            #self.y_velocity = vx_b * sin_y + vy_b * cos_y
                            self.x_velocity=vx_b*0.6
                            self.y_velocity=vy_b*0.6
                            self.z_velocity = 0.0

                            # Integrate position
                            self.x_pos += self.x_velocity * dt_lidar
                            self.y_pos += self.y_velocity * dt_lidar
                        self.prev_lidar_map = gray
                        self.prev_lidar_corners = good_new.reshape(-1,1,2)
                    else:
                        self.prev_lidar_corners = cv.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
                        self.prev_lidar_map = gray

        # Publish attitude
        att = Vector3(x=float(self.roll), y=float(self.pitch), z=float(self.yaw))
        self.publisher_attitude.publish(att)
        # Publish velocity
        vel = Vector3(x=float(self.x_velocity), y=float(self.y_velocity), z=float(self.z_velocity))
        self.publisher_velocity.publish(vel)
        # Publish pose estimate
        pose = Vector3(x=float(self.x_pos), y=float(self.y_pos), z=float(self.yaw))
        self.publisher_pose_estimate.publish(pose)

    def mag_callback(self, data):
        self.mag = (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z)

    def lidar_callback(self, data):
        scan = np.flip(np.multiply(np.array(data.ranges), 100))
        self.scan_data = np.array([0 if np.isinf(x) else x for x in scan])

    # ONLY used for displaying lidar image
    def get_lidar_image(self, scan):
        #create array of angles for each angle (because real life are every 1/3 degree)
        angles = np.deg2rad(np.arange(0, 360, 1/3))
        dists = np.array(scan)

        # assume 0.0 distances are error
        valid = dists > 0

        # converts cartesian to polar
        x = dists[valid] * np.sin(angles[valid])
        y = dists[valid] * np.cos(angles[valid])

        # blank image
        img = np.zeros((480,640,3), dtype=np.uint8)

        # converts from lidar -> pixels
        xi = (x*0.2 + 320).astype(int); yi = (320 - y*0.2).astype(int)
        
        # filters out points outside of 640x480 image
        mask = (xi>=0)&(xi<640)&(yi>=0)&(yi<480)

        # plot points
        for x0,y0 in zip(xi[mask], yi[mask]): cv.circle(img, (x0,y0), 1, (255,255,255), -1)
        
        # mark car position
        cv.circle(img,(320,320),4,(0,0,255),-1)
        return img


def main(args=None):
    rclpy.init(args=args)
    print("init completed")
    node = VelocityNode()
    print("velocity node done")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
