import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleVisualOdometry
from px4_msgs.msg import EstimatorVisualOdometryAligned
from px4_msgs.msg import VehicleImu
from geometry_msgs.msg import PoseStamped
from math import *

class VehicleVisualOdomPublisher(Node):
    def __init__(self):
        super().__init__("vicon_bridge")

        self.position_publisher_pi_ = self.create_publisher(PoseStamped, "/com/odom_feedback", 10)
        self.mocap_pub_ = self.create_publisher(VehicleVisualOdometry, "fmu/vehicle_visual_odometry/in", 10)
        
        self.vehicle_imu_sub_ = self.create_subscription(VehicleImu, "fmu/vehicle_imu/out", self.vehicle_imu_callback, 10)
        self.estimator_odom_sub_ = self.create_subscription(EstimatorVisualOdometryAligned, "fmu/estimator_visual_odometry_aligned/out", self.estimator_visual_odom_callback, 10)
        self.mocap_sub_ = self.create_subscription(PoseStamped, "/drohne_lps/pose", self.vicon_callback, 10)

        timer_period = 0.02  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.timestamp = 0
        self.timestamp_sample = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.q = [0.0,0.0,0.0,0.0]
    
    def vicon_callback(self, msg_in):
        
        #self.timestamp_sample = self.timestamp
        self.x = msg_in.pose.position.x
        self.y = -msg_in.pose.position.y
        self.z = -msg_in.pose.position.z
        self.q[0] = msg_in.pose.orientation.w
        self.q[1] = msg_in.pose.orientation.x
        self.q[2] = -msg_in.pose.orientation.y
        self.q[3] = -msg_in.pose.orientation.z

    def vehicle_imu_callback(self, msg):
        
        self.timestamp = msg.timestamp
        self.timestamp_sample = msg.timestamp_sample

    def estimator_visual_odom_callback(self, msg):

        msg_pub = PoseStamped()
        msg_pub.header.frame_id = "world"
        msg_pub.header.stamp = self.get_clock().now().to_msg()
        msg_pub.pose.position.x = msg.x
        msg_pub.pose.position.y = -msg.y
        msg_pub.pose.position.z = -msg.z
        msg_pub.pose.orientation.w = float(msg.q[0])
        msg_pub.pose.orientation.x = float(msg.q[1])
        msg_pub.pose.orientation.y = float(-msg.q[2])
        msg_pub.pose.orientation.z = float(-msg.q[3])
        self.position_publisher_pi_.publish(msg_pub)

    def timer_callback(self):        
        msg = VehicleVisualOdometry()

        msg.timestamp = self.timestamp
        msg.timestamp_sample = self.timestamp_sample
        msg.local_frame = VehicleVisualOdometry.LOCAL_FRAME_NED
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.q[0] = self.q[0]
        msg.q[1] = self.q[1]
        msg.q[2] = self.q[2]
        msg.q[3] = self.q[3]
        msg.q_offset[0] = float("NaN")
        msg.pose_covariance[0] = float("NaN")
        msg.vx = msg.vy = msg.vz = float("NaN")
        msg.rollspeed = msg.pitchspeed = msg.yawspeed = float("NaN")
        msg.velocity_covariance[0] = float("NaN")
        self.mocap_pub_.publish(msg)

def main(args=None):
    rclpy.init()
    vehicle_visual_pub = VehicleVisualOdomPublisher()
    rclpy.spin(vehicle_visual_pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
