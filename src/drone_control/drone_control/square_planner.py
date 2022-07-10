import rclpy
from rclpy.node import Node
from time import sleep
import numpy as np

from px4_msgs.msg import TrajectorySetpoint

class SquarePlanner(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')

        # publishers:
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "/com/use_setpoint", 10)

        # timer:
        self.offboard_setpoint_counter_ = 0  # counter for the number of setpoints sent
        timer_period = 0.2  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        
        # cage limits:
        self.xmax = 2.0
        self.xmin = -2.0
        self.ymax = 2.5
        self.ymin = -2.5
        self.zmax = 2.5
        self.zmin = -0.5

    def timer_callback(self):
        
        print("------------  Size of Square Route ------------")

        self.x = input("x [m]: ")
        while True:
            if self.xmin > float(self.x) or float(self.x) > self.xmax:
                self.x = input("Try again! Position in x [m]: ")
            else:
                break

        self.y = input("y [m]: ")
        while True:
            if self.ymin > float(self.y) or float(self.y) > self.ymax:
                self.y = input("Try again! Position in y [m]: ")
            else:
                break
        
        self.z = input("z [m]: ")
        while True:
            if self.zmin > float(self.z) or float(self.z) > self.zmax:
                self.z = input("Try again! Position in z [m]: ")
            else:
                break

        self.yaw = input("Yaw angle [°]: ")

        try:
            print("------------  Flying!  ------------")

            msg = TrajectorySetpoint()

            msg.x = 0.0
            msg.y = 0.0
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)

            sleep(5)

            msg = TrajectorySetpoint()

            msg.x = float(self.x)
            msg.y = 0.0
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)

            sleep(5)
            
            msg = TrajectorySetpoint()

            msg.x = float(self.x)
            msg.y = float(self.y)
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)
            
            sleep(5)
            
            msg = TrajectorySetpoint()

            msg.x = 0.0
            msg.y = float(self.y)
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)

            sleep(5)

            msg = TrajectorySetpoint()

            msg.x = 0.0
            msg.y = 0.0
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)
        except ValueError:
            print("Inputs are not valid.")
        

def main(args=None):
    rclpy.init()
    square_planner = SquarePlanner()
    rclpy.spin(square_planner)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
