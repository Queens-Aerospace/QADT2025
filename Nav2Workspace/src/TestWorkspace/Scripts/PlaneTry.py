#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
from time import time


class OffboardControlNode(Node):
    """ROS 2 Node to control a plane in offboard mode."""

    def __init__(self):
        super().__init__('offboard_control_node')

        # Publishers
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        # Timer to repeatedly send setpoints
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State variables
        self.offboard_setpoint_counter = 0
        self.target_altitude = -10.0  # Desired altitude (negative for PX4 convention)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Sent Arm command')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Sent Disarm command')

    def set_offboard_mode(self):
        """Send a command to enable offboard mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Sent Offboard mode command')

    def publish_vehicle_command(self, command, **params):
        """Publish a VehicleCommand message."""
        msg = VehicleCommand()
        msg.timestamp = int(time() * 1_000_000)
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_publisher.publish(msg)

    def publish_control_mode(self):
        """Publish OffboardControlMode to define how control inputs are interpreted."""
        msg = OffboardControlMode()
        msg.timestamp = int(time() * 1_000_000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.control_mode_publisher.publish(msg)
        self.get_logger().info('Offboard control mode set (position control)')

    def publish_trajectory_setpoint(self):
        """Publish a TrajectorySetpoint to command forward movement."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(time() * 1_000_000)
        msg.position = [50.0, 0.0, self.target_altitude]  # Move forward to x=50m, keep altitude
        msg.velocity = [10.0, 0.0, 0.0]  # Forward velocity
        msg.yaw = 0.0  # Keep heading forward
        self.setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing trajectory setpoint: {msg.position}")


    def timer_callback(self):
        """Callback function to handle periodic commands."""
        if self.offboard_setpoint_counter == 0:
            self.publish_control_mode()
            self.set_offboard_mode()
            self.arm()

        self.publish_trajectory_setpoint()

        self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
