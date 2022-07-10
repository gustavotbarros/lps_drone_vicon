
from time import sleep
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand


class OffboardMode(Node):
    def __init__(self):
        super().__init__('offboard_mode')

        # Publishers:
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/vehicle_command/in", 10)

        # Subscribers:
        self.timesync_sub_ = self.create_subscription(Timesync, "fmu/timesync/out", self.timesync_callback, 10)
        self.trajectory_setpoint_sub_ = self.create_subscription(TrajectorySetpoint, "com/use_setpoint", self.process_trajectory_setpoint, 10)

        # Parameters and Local Variables:
        # TrajectorySetpoint:
        self.x = 0.0
        self.y = 0.0
        self.z = -1.2
        self.yaw = 0.0
        self.timestamp_ = 0
        
        # For timer:
        self.offboard_setpoint_counter_ = 0  # counter for the number of setpoints sent
        timer_period = 0.2  # seconds
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def timesync_callback(self, msg):
        self.timestamp_ = msg.timestamp

    def timer_callback(self):
        self.offboard_setpoint_counter_+= 1
        
        if self.offboard_setpoint_counter_ >= 2:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
            self.arm()
            
            self.offboard_setpoint_counter_ += 1
        
        if self.offboard_setpoint_counter_ >= 10:

            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()

    def process_trajectory_setpoint(self, traj):

        self.x = traj.x
        self.y = traj.y
        self.z = traj.z
        self.yaw = traj.yaw
        
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    # @brief  Send a command to Disarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    # @brief Publish the offboard control mode. For this example, only position and altitude controls are active.
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        msg.timestamp = self.timestamp_
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)
        
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp_
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw = self.yaw

        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher_.publish(msg)

    def publish_takeoff(self, command, param1, param2, param3):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    print("Starting drone control...")
    rclpy.init()
    offboard_mode = OffboardMode()
    rclpy.spin(offboard_mode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
