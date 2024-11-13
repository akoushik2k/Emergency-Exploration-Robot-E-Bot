#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import JointState
from math import pow, atan2, sqrt, asin
import math
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import Quaternion
import numpy as np

class EbotWaypoint(Node):
    """ROS2 Node for guiding a robot through predefined waypoints and managing interactions with an arm task."""

    def __init__(self):
        """Initialize the EbotWaypoint node, setup publishers, subscribers, and state variables."""
        super().__init__('ebot_waypoint')

        # Subscriber for receiving the robot's current position
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wheel_odom',
            self.current_position_callback,
            10
        )

        # Publisher for sending wheel velocities
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscriber to receive the arm's task status
        self.sub_arm_status = self.create_subscription(
            Int32,
            '/arm_task_status',
            self.arm_status_callback,
            10
        )
        
        # Publisher to send the task goal to the arm
        self.send_arm_task = self.create_publisher(
            Int32, '/set_goal', 10)
        
        # Set timers to periodically call the callback functions
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer2 = self.create_timer(self.timer_period, self.timer_callback2)

        # Robot's physical parameters
        self.wheel_base = 0.74
        self.wheel_rad = 0.461 / 2.0
        self.continue_nav = False

        # Robot state variables
        self.position_ = Point()
        self.yaw_ = 0
        self.state_ = 0
        self.desired_position_ = Point()
        self.desired_position_.x = 10.0
        self.desired_position_.y = 0.0
        self.desired_position_.z = 0.0

        # Navigation precision parameters
        self.yaw_precision_ = math.pi / 45  # +/- 2 degrees allowed
        self.dist_precision_ = 0.3

        # Define waypoints
        self.waypoints = [
            [26.916, -0.51], 
            [27.22, 2.82], 
            [33.47, 2.82], 
            [33.47, 8.12], 
            [30.47, 8.12], 
            [27.7, 8.02]
        ]
        self.cur_waypoint = -1
        self.wait_for_arm = False

        # Storage for wheel velocities
        self.wheel_velocities = Float64MultiArray()

    def euler_from_quaternion(self, quaternion):
        """Convert quaternion to euler angles.

        Args:
            quaternion (tuple): Quaternion (x, y, z, w).

        Returns:
            tuple: Roll, pitch, and yaw angles.
        """
        x, y, z, w = quaternion

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def current_position_callback(self, msg):
        """Callback for updating the robot's current position and orientation.

        Args:
            msg (PoseStamped): Current position of the robot.
        """
        self.position_ = msg.pose.position
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        euler = self.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]

    def change_state(self, state):
        """Change the robot's navigation state.

        Args:
            state (int): New state to change to.
        """
        self.state_ = state

    def fix_yaw(self, des_pos):
        """Adjust robot orientation to face the target waypoint.

        Args:
            des_pos (Point): Desired waypoint position.
        """
        desired_yaw = math.atan2(des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_

        if err_yaw > math.pi:
            err_yaw -= 2 * math.pi
        elif err_yaw < -math.pi:
            err_yaw += 2 * math.pi

        twist_msg = Twist()
        if abs(err_yaw) > self.yaw_precision_:
            twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
            self.pub_vel(twist_msg)

        # Change state if within yaw precision
        if abs(err_yaw) <= self.yaw_precision_:
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        """Move the robot straight towards the target waypoint.

        Args:
            des_pos (Point): Desired waypoint position.
        """
        desired_yaw = math.atan2(des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_

        if err_yaw > math.pi:
            err_yaw -= 2 * math.pi
        elif err_yaw < -math.pi:
            err_yaw += 2 * math.pi

        err_pos = sqrt(pow(des_pos.y - self.position_.y, 2) + pow(des_pos.x - self.position_.x, 2))

        if err_pos > self.dist_precision_:
            twist_msg = Twist()
            twist_msg.linear.x = 0.05 * err_pos if self.cur_waypoint >= 4 else 0.1 * err_pos
            self.pub_vel(twist_msg)
        else:
            self.change_state(2)

        if abs(err_yaw) > self.yaw_precision_:
            self.change_state(0)

    def done(self):
        """Stop the robot."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_vel(twist_msg)

    def pub_vel(self, twist_msg):
        """Publish wheel velocities based on linear and angular velocities.

        Args:
            twist_msg (Twist): Contains linear and angular velocities.
        """
        self.lin_vel = twist_msg.linear.x
        self.ang_vel = twist_msg.angular.z
        self.left_vel = (self.lin_vel - self.ang_vel * self.wheel_base / 2.0) / self.wheel_rad
        self.right_vel = (self.lin_vel + self.ang_vel * self.wheel_base / 2.0) / self.wheel_rad
        self.wheel_velocities.data = [-self.left_vel, -self.right_vel, -self.left_vel, -self.right_vel]
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        
    def timer_callback(self):
        """Periodic callback to manage navigation through states and waypoints."""
        if self.state_ == 0:
            self.fix_yaw(self.desired_position_)
        elif self.state_ == 1:
            self.go_straight_ahead(self.desired_position_)
        elif self.state_ == 2:
            self.done()
            self.check_arm(self.cur_waypoint)

            if self.wait_for_arm:
                pass
            else:
                if not self.continue_nav:
                    self.cur_waypoint += 1
                self.continue_nav = False

                if self.cur_waypoint < len(self.waypoints):
                    self.desired_position_.y = self.waypoints[self.cur_waypoint][1]
                    self.desired_position_.x = self.waypoints[self.cur_waypoint][0]
                    self.state_ = 0     

    def check_arm(self, cur_waypoint):
        """Check if the arm needs to perform a task at the current waypoint.

        Args:
            cur_waypoint (int): Current waypoint index.
        """
        if cur_waypoint == 4 or cur_waypoint == 6:
            self.wait_for_arm = True

    def timer_callback2(self):
        """Timer callback to publish arm tasks based on waypoints."""
        if self.wait_for_arm and self.cur_waypoint == 4:
            self.send_arm_task.publish(Int32(data=2))

    def arm_status_callback(self, msg):
        """Callback for updating navigation based on arm task completion.

        Args:
            msg (Int32): Arm task completion status.
        """
        if msg.data == 1:
            self.wait_for_arm = False
            self.cur_waypoint += 1
            self.continue_nav = True

        elif msg.data == 4:
            self.wait_for_arm = False
            self.cur_waypoint += 1
            self.continue_nav = True

def main(args=None):
    """Main function to initialize and spin the EbotWaypoint node."""
    rclpy.init(args=args)
    ebot_waypoint = EbotWaypoint()
    rclpy.spin(ebot_waypoint)
    ebot_waypoint.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
