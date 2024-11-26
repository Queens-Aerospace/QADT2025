#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
import time


class PlaneTakeoffAndForward(Node):
    def __init__(self):
        super().__init__('plane_takeoff_and_forward')
        self.get_logger().info("Initializing Plane Takeoff and Forward Node")

        # Publishers
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        # Altitude placeholder
        self.current_altitude = 0.0

        # Subscribe to altitude feedback
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.altitude_callback,
            10
        )

        # Timer to send offboard commands at regular intervals
        self.timer = self.create_timer(0.1, self.send_commands)

        # Flight state variables
        self.takeoff_altitude = 10.0  # Desired altitude in meters
        self.forward_velocity = 10.0  # Desired forward velocity in m/s
        self.takeoff_completed = False

        # Start the offboard mode
        self.start_offboard_mode()

    def altitude_callback(self, msg):
        # Update current altitude based on feedback
        self.current_altitude = -msg.z  # Convert from NED (z is negative for altitude above ground)

    def get_altitude(self):
        # Return the current altitude
        return self.current_altitude

    def start_offboard_mode(self):
        # Arm the vehicle
        self.arm()
        time.sleep(2)

        # Send offboard activation command
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Offboard mode enabled")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arming vehicle...")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarming vehicle...")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 requires microseconds
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def send_commands(self):
        # Set offboard control mode to position and velocity
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Publish trajectory setpoint
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # Ensure arrays have correct lengths
        traj_msg.position = [0.0, 0.0, 0.0]  # Default position [x, y, z]
        traj_msg.velocity = [0.0, 0.0, 0.0]  # Default velocity [vx, vy, vz]

        if not self.takeoff_completed:
            traj_msg.position[2] = -self.takeoff_altitude  # Altitude (negative in NED)
            traj_msg.velocity[0] = 0.0  # No forward velocity during takeoff
            self.get_logger().info(f"Taking off to altitude {self.takeoff_altitude} meters")
            if self.get_altitude() >= self.takeoff_altitude - 1.0:  # Check if takeoff is nearly complete
                self.takeoff_completed = True
                self.get_logger().info("Takeoff completed!")
        else:
            traj_msg.position[2] = -self.takeoff_altitude
            traj_msg.velocity[0] = self.forward_velocity  # Forward velocity in m/s
            self.get_logger().info(f"Flying forward at {self.forward_velocity} m/s")

        self.trajectory_setpoint_pub.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlaneTakeoffAndForward()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.disarm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

