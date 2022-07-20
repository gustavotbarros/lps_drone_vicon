from itertools import count
import rclpy
from rclpy.node import Node
from time import sleep
import numpy as np

from px4_msgs.msg import TrajectorySetpoint

class CirclePlanner(Node):
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
        self.count = 0.0

    def timer_callback(self):
        
        print("------------  Size of Circle Route ------------")

        self.radius = input("Radius [m]: ")
        while True:
            if self.xmin > float(self.radius) or float(self.radius) > self.xmax:
                self.radius = input("Try again! Radius [m]: ")
            else:
                self.x = self.radius
                self.y = self.radius
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
            msg.yaw = 0.0

            self.trajectory_setpoint_publisher_.publish(msg)

            sleep(5)

            msg = TrajectorySetpoint()

            msg.x = 0.0
            msg.y = 0.0
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)

            sleep(8)

            msg = TrajectorySetpoint()

            msg.x = float(self.radius)
            msg.y = 0.0
            msg.z = float(self.z)*(-1)
            msg.yaw = np.deg2rad(float(self.yaw))

            self.trajectory_setpoint_publisher_.publish(msg)
            print(self.radius)

            sleep(5)
            
            while self.count < 100.0:
                
                self.x = float(self.radius)*np.cos(self.count*0.5)
                self.y = float(self.radius)*np.sin(self.count*0.5)

                msg = TrajectorySetpoint()

                msg.x = self.x
                msg.y = self.y
                msg.z = float(self.z)*(-1)
                msg.yaw = np.deg2rad(float(self.yaw))

                print(msg.x)
                print(msg.y)

                self.trajectory_setpoint_publisher_.publish(msg)
                
                sleep(1.0)

                self.count += 0.8
                print(self.count)
                
        except ValueError:
            print("Inputs are not valid.")
        

def main(args=None):
    rclpy.init()
    circle_planner = CirclePlanner()
    rclpy.spin(circle_planner)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
