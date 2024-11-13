#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import termios


class KeyboardControlNode(Node):
    """ROS2 Node to control a robot's wheels using keyboard input via `/cmd_vel` messages."""

    def __init__(self):
        """Initialize the KeyboardControlNode, setting up publishers and subscribers."""
        super().__init__('keyboard_control_node')

        # Publisher to send wheel velocities to the velocity controller
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # Publisher for joint states (currently not used but set up)
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # Subscriber to receive Twist messages from `/cmd_vel` (linear and angular velocities)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Robot's physical parameters
        self.wheel_base = 0.74
        self.wheel_rad = 0.461 / 2.0

        # Save current terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def cmd_vel_callback(self, msg):
        """Callback function to process the received `/cmd_vel` messages.

        Args:
            msg (Twist): Incoming Twist message with linear and angular velocities.
        """
        self.cmd_vel = msg
        self.get_logger().info('Received cmd_vel message: "%s"' % msg)
        self.calc_vel()

    def calc_vel(self):
        """Calculate and publish wheel velocities based on linear and angular velocities."""
        # Extract linear and angular velocities from cmd_vel
        self.linear_vel = self.cmd_vel.linear.x
        self.angular_vel = self.cmd_vel.angular.z

        # Calculate individual wheel velocities
        self.left_vel = (self.linear_vel - self.angular_vel * self.wheel_base / 2.0) / self.wheel_rad
        self.right_vel = (self.linear_vel + self.angular_vel * self.wheel_base / 2.0) / self.wheel_rad

        # Create and populate Float64MultiArray with wheel velocities
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [-self.left_vel, -self.right_vel, -self.left_vel, -self.right_vel]

        # Publish wheel velocities
        self.wheel_velocities_pub.publish(wheel_velocities)


def main(args=None):
    """Main function to initialize and spin the KeyboardControlNode."""
    rclpy.init(args=args)
    node = KeyboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
