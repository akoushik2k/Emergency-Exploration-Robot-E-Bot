#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Int32
from geometry_msgs.msg import Quaternion
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Vector3
import sympy as sp
from sympy import Matrix, symbols
import math
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ArmControl(Node):
    """ROS2 Node for controlling a robotic arm using inverse kinematics."""

    def __init__(self):
        """Initialize the ArmControl node and set up publishers, subscribers, and variables."""
        super().__init__('ebot_arm_control_node')

        # Publisher for joint positions to control the arm
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        # Client for the gripper service to open/close the gripper
        self.gripper_service = self.create_client(SetBool, '/custom_switch')

        # Publisher for arm task status
        self.pub_arm_task_status = self.create_publisher(
            Int32, '/arm_task_status', 10)

        # Subscriber for setting the goal position
        self.goal_sub = self.create_subscription(
            Int32, '/set_goal', self.goal_callback, 10)

        # Timer to periodically check and execute tasks
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscriber to receive joint states (currently not used)
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        # Initialize joint positions
        self.joint_positions = Float64MultiArray()
        self.home_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Current position
        self.curr = [0.0, 356.0, 1428.0]
        self.goal = [-120.0, 400.0, 600.0]
        self.cur_pos = [0.0, 356.0, 1428.0]

        # Goal management variables
        self.set_goal = -1
        self.cur_goal = -1

        # Time allocated for the movement
        self.time_given = 20

        # Initial joint angles
        self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        # Time counter
        self.t = 0

        # Velocity computation
        self.velocity = (np.array(self.goal) -
                         np.array(self.curr)) / self.time_given
        print(self.velocity)

        # Publication flag
        self.pub = False

        # Lists for plotting trajectory
        self.X = []
        self.Y = []
        self.Z = []

    def goal_callback(self, msg):
        """Callback function to receive goal commands."""
        self.set_goal = msg.data

    def joint_states_callback(self, msg):
        """Callback function to receive joint states (currently not used)."""
        pass

    def dh_transform_matrix(self, theta, d, a, alpha):
        """Compute the individual transformation matrix using DH parameters.

        Args:
            theta (float): Joint angle.
            d (float): Link offset.
            a (float): Link length.
            alpha (float): Link twist.

        Returns:
            sp.Matrix: The transformation matrix.
        """
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),
             sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -
             sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_T(self, dh_params):
        """Compute the transformation matrices for all joints.

        Args:
            dh_params (list): List of DH parameters for each joint.

        Returns:
            tuple: List of individual transformation matrices and the final position.
        """
        A = []
        T = []
        num_joints = len(dh_params)
        transformation_matrix = sp.eye(4)

        for i in range(num_joints):
            theta, d, a, alpha = dh_params[i]
            transformation_matrix = transformation_matrix * \
                (self.dh_transform_matrix(theta, d, a, alpha))
            A.append(self.dh_transform_matrix(theta, d, a, alpha))

        # Compute cumulative transformation matrices for each joint
        T.append(sp.eye(4))
        T.append(A[0])
        T.append(A[0]*A[1])
        T.append(A[0]*A[1]*A[2])
        T.append(A[0]*A[1]*A[2]*A[3])
        T.append(A[0]*A[1]*A[2]*A[3]*A[4])
        T.append(A[0]*A[1]*A[2]*A[3]*A[4]*A[5])

        return T, [transformation_matrix[0, 3], transformation_matrix[1, 3], transformation_matrix[2, 3]]

    def final_transformation_matrix(self, th1, th2, th3, th4, th5, th6):
        """Compute the final transformation matrix for given joint angles.

        Args:
            th1, th2, th3, th4, th5, th6 (float): Joint angles.

        Returns:
            tuple: List of individual transformation matrices and the final position.
        """
        dh_parameters = [
            [th1 + math.pi, 128.0, 0.0, math.pi/2],
            [th2 + math.pi/2, 0.0, 612.7, math.pi],
            [th3, 0.0, 571.6, math.pi],
            [th4 - math.pi/2, 163.9, 0.0, -math.pi/2],
            [th5, 115.7, 0.0, math.pi/2],
            [th6, 192.2, 0.0, 0.0]
        ]
        return self.compute_T(dh_parameters)

    def compute_J(self, th1, th2, th3, th4, th5, th6):
        """Compute the Jacobian matrix for the given joint angles.

        Args:
            th1, th2, th3, th4, th5, th6 (float): Joint angles.

        Returns:
            tuple: Jacobian matrix and the final position.
        """
        T, [x_final, y_final, z_final] = self.final_transformation_matrix(
            th1, th2, th3, th4, th5, th6)
        J_final = sp.Matrix.zeros(6, 6)

        # Extract z vectors and origin positions from transformation matrices
        z = [matrix.col(2)[:3] for matrix in T]
        O = [matrix.col(3)[:3] for matrix in T]

        z_array = sp.Matrix([*z])
        O_array = sp.Matrix([*O])

        # Calculate each column of the Jacobian
        for i in range(6):
            z_i = z_array[i, :]
            t6 = O_array[6, :]
            ti = O_array[i, :]
            cross_product = z_i.cross(t6 - ti)

            # Convert to list for concatenation
            cross_product_list = list(cross_product)
            z_list = list(z_i)

            # Concatenate linear and angular velocities
            J_i_list = cross_product_list + z_list

            # Assign to Jacobian matrix
            J_final[:, i] = Matrix(J_i_list).reshape(6, 1)

        return J_final, [x_final, y_final, z_final]

    def tasks(self, x):
        """Set up predefined tasks with specific start and goal positions.

        Args:
            x (int): Task identifier.

        Returns:
            tuple: Current position and goal position.
        """
        if x == 1:
            # Task 1: Move to bin
            current = self.cur_pos
            goal = [-120.0, 350.0, 500.0]

        elif x == 2:
            # Task 2: Pick an object
            current = [0.0, 356.0, 1428.0]
            goal = [-900.0, 150.0, -130.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        elif x == 3:
            # Task 3: Move to lever
            current = [0.0, 356.0, 1428.0]
            goal = [-900.0, 100.0, -130.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        elif x == 4:
            # Task 4: Final lever position
            current = self.cur_pos
            goal = [self.cur_pos[0], self.cur_pos[1]+400, self.cur_pos[2]+200]

        elif x == 5:
            # Task 5: Return to home position
            current = [0.0, 356.0, 1428.0]
            goal = [0.0, 355.0, 1428.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        elif x == 11:
            # Task 11: Move to lever (alternative)
            current = [0.0, 356.0, 1428.0]
            goal = [-900.0, 100.0, 600.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        elif x == 9:
            # Task 9: Plot the trajectory
            self.plot_trajectory(self.X, self.Y, self.Z)

        return current, goal

    def compute_q(self, x):
        """Compute joint angles to achieve the task specified by x.

        Args:
            x (int): Task identifier.
        """
        current, goal = self.tasks(x)
        dt = 0.01
        velocity = (np.array(goal) - np.array(current)) / self.time_given

        if x == 5:
            # Move to home position
            self.pub = True
            if self.pub:
                self.joint_positions.data = self.home_joint_positions
                self.joint_position_pub.publish(self.joint_positions)
        else:
            while self.t < self.time_given:
                E = sp.Matrix([velocity[0], velocity[1], velocity[2], 0, 0.0, 0])

                # Compute Jacobian and current position
                J, [x_final, y_final, z_final] = self.compute_J(
                    self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5])
                J = np.array(J).astype('float64')

                # Compute pseudo-inverse of the Jacobian
                J_inv = np.linalg.pinv(J)

                # Compute joint velocities
                q_dot = np.dot(J_inv, E)

                # Update joint angles
                self.q = self.q + q_dot * dt

                # Limit joint angles within -pi to pi
                k = [0.0] * 6
                for i in range(6):
                    if self.q[i] > math.pi:
                        self.q[i] = float(self.q[i] - 2 * math.pi)
                        k[i] = float(self.q[i])
                    elif self.q[i] < -math.pi:
                        self.q[i] = float(self.q[i] + 2 * math.pi)
                        k[i] = float(self.q[i])
                    else:
                        k[i] = float(self.q[i])

                # Publish joint positions
                self.joint_positions.data = k
                print("=====================================")
                print("Time:", self.t)
                print("Joint Angles (q):", self.q)
                print("Velocity:", self.velocity)
                print("Joint Positions:", self.joint_positions.data)
                print("Current Position:", x_final, y_final, z_final)
                print("=====================================")

                if x == 2:
                    self.X.append(x_final)
                    self.Y.append(y_final)
                    self.Z.append(z_final)

                self.t += dt
                self.joint_position_pub.publish(self.joint_positions)

            # Reset time counter
            self.t = 0

            # Handle post-task actions based on task
            if x == 3:
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                print("Computing q for Task 4")
                self.compute_q(4)

            elif x == 11:
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                self.compute_q(4)

            elif x == 2:
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                # Activate the gripper
                self.gripper_service.wait_for_service()
                request = SetBool.Request()
                request.data = True
                self.gripper_service.call_async(request)
                print("Computing q for Task 3")
                self.compute_q(1)
                self.pub_arm_task_status.publish(Int32(data=1))

            elif x == 1:
                # Deactivate the gripper
                self.gripper_service.wait_for_service()
                request = SetBool.Request()
                request.data = False
                self.gripper_service.call_async(request)

    def timer_callback(self):
        """Timer callback function to check and execute tasks."""
        if self.cur_goal != self.set_goal:
            self.compute_q(self.set_goal)
            self.cur_goal = self.set_goal
            print("Goal updated")

    def plot_trajectory(self, X, Y, Z):
        """Plot the trajectory of the end effector.

        Args:
            X (list): X coordinates.
            Y (list): Y coordinates.
            Z (list): Z coordinates.
        """
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the trajectory points
        ax.scatter(self.X, self.Y, self.Z, color='red', marker='o')

        # Set axis labels
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        # Set axis limits
        ax.set_xlim([float(min(self.X)), float(max(self.X))])
        ax.set_ylim([float(min(self.Y)), float(max(self.Y))])
        ax.set_zlim([float(min(self.Z)), float(max(self.Z))])

        # Show the plot
        plt.show()


def main(args=None):
    """Main function to initialize the node and spin."""
    rclpy.init(args=args)
    node = ArmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
