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
# std_srvs/srv/SetBool
from std_srvs.srv import SetBool
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class ArmControl(Node):
    def __init__(self):
        super().__init__('ebot_arm_control_node')

        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        # call a ros2 service to operate the gripper

        self.gripper_service = self.create_client(SetBool, '/custom_switch')
        self.pub_arm_task_status = self.create_publisher(
            Int32, '/arm_task_status', 10)

        self.goal_sub = self.create_subscription(
            Int32, '/set_goal', self.goal_callback, 10)

        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # subscribe to joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        self.joint_positions = Float64MultiArray()
        self.home_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.curr = [0.0, 356.0, 1428.0]
        # self.goal = [500.0,800.0,600.0]
        self.goal = [-120.0, 400.0, 600.0]
        self.cur_pos = [0.0, 356.0, 1428.0]
        self.set_goal = -1
        self.cur_goal = -1

        self.time_given = 20
        self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])
        self.t = 0
        self.velocity = (np.array(self.goal) -
                         np.array(self.curr))/self.time_given
        print(self.velocity)
        self.pub = False
        self.X = []
        self.Y = []
        self.Z = []

    def goal_callback(self, msg):
        self.set_goal = msg.data

    def joint_states_callback(self, msg):

        pass

    def dh_transform_matrix(self, theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha),
             sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -
             sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_T(self, dh_params):

        A = []
        T = []
        num_joints = len(dh_params)
        transformation_matrix = sp.eye(4)

        for i in range(num_joints):
            theta, d, a, alpha = dh_params[i]
            transformation_matrix = transformation_matrix * (self.dh_transform_matrix(theta, d, a, alpha))

            A.append(self.dh_transform_matrix(theta, d, a, alpha))
        # sp.pprint(transformation_matrix)

        T.append(sp.eye(4))
        T.append(A[0])
        T.append(A[0]*A[1])
        T.append(A[0]*A[1]*A[2])
        T.append(A[0]*A[1]*A[2]*A[3])
        T.append(A[0]*A[1]*A[2]*A[3]*A[4])
        T.append(A[0]*A[1]*A[2]*A[3]*A[4]*A[5])

        return T, [transformation_matrix[0, 3], transformation_matrix[1, 3], transformation_matrix[2, 3]]

    def final_transformation_matrix(self, th1, th2, th3, th4, th5, th6):
        dh_parameters = [
            [th1+math.pi, 128.0, 0.0, math.pi/2],
            [th2+math.pi/2, 0.0, 612.7, math.pi],
            [th3, 0.0, 571.6, math.pi],
            [th4-math.pi/2, 163.9, 0.0, -math.pi/2],
            [th5, 115.7, 0.0, math.pi/2],
            [th6, 192.2, 0.0, 0.0]
        ]
        return self.compute_T(dh_parameters)

    def compute_J(self, th1, th2, th3, th4, th5, th6):
        T, [x_final, y_final, z_final] = self.final_transformation_matrix(
            th1, th2, th3, th4, th5, th6)
        J_final = sp.Matrix.zeros(6, 6)
        # First three elements of the third column
        z = [matrix.col(2)[:3] for matrix in T]
        # First three elements of the fourth column
        O = [matrix.col(3)[:3] for matrix in T]

        z_array = sp.Matrix([*z])
        O_array = sp.Matrix([*O])
        # Calculate each column of the Jacobian
        for i in range(6):

            z = z_array[i, :]
            t6 = O_array[6, :]
            ti = O_array[i, :]
            cross_product = z.cross(t6 - ti)

            # Convert the cross product and z to Python lists
            cross_product_list = list(cross_product)
            z_list = list(z)

            J_i_list = cross_product_list + z_list
            # Convert the concatenated list back to a column vector
            J_i = Matrix(J_i_list).reshape(6, 1)
            # print(J_i)

            J_final[:, i] = J_i

        return J_final, [x_final, y_final, z_final]

    def tasks(self, x):
        if x == 1:
            # bin
            current = self.cur_pos
            goal = [-120.0, 350.0, 500.0]
            # self.q = sp.Matrix([0.01,-0.01,0.01,0.01,0.01,0.01])

        if x == 2:
            # pick
            current = [0.0, 356.0, 1428.0]
            # goal = [-900.0,150.0,300.0]
            goal = [-900.0, 150.0, -130.0]

            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        if x == 3:
            # lever
            current = [0.0, 356.0, 1428.0]
            goal = [-900.0, 100.0, -130.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])

        if x == 4:
            # lever final
            current = self.cur_pos
            goal = [self.cur_pos[0], self.cur_pos[1]+400, self.cur_pos[2]+200]
            # self.q = sp.Matrix([0.01,-0.01,0.01,0.01,0.01,0.01])

        if x == 5:
            current = [0.0, 356.0, 1428.0]
            goal = [0.0, 355.0, 1428.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])
        
        if x == 11:
            # lever
            current = [0.0, 356.0, 1428.0]
            goal = [-900.0,100.0,600.0]
            self.q = sp.Matrix([0.01, -0.01, 0.01, 0.01, 0.01, 0.01])
        
        if x == 9:
            self.plot_trajectory(self.X, self.Y, self.Z)
            

        return current, goal

    def compute_q(self, x):
        current, goal = self.tasks(x)
        dt = 0.01
        velocity = (np.array(goal)-np.array(current))/self.time_given
        if x == 5:
            self.pub = True
            if (self.pub):
                self.joint_positions.data = self.home_joint_positions
                self.joint_position_pub.publish(self.joint_positions)

        else:
            while self.t < 20:
                E = sp.Matrix(
                    [velocity[0], velocity[1], velocity[2], 0, 0.0, 0])
                # E = sp.Matrix([5.0,0,0 ,0, 0, 0])

                J, [x_final, y_final, z_final] = self.compute_J(
                    self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5])
                # make j into folat
                J = np.array(J).astype('float64')

                # find inverse of J using pseudo inverse np
                J_inv = np.linalg.pinv(J)

                q_dot = np.dot(J_inv, E)

                # print(x)
                k = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                self.q = self.q + q_dot*dt
                # limit q from -pi to pi
                for i in range(6):
                    if self.q[i] > math.pi:
                        self.q[i] = float(self.q[i] - 2*math.pi)
                        k[i] = float(self.q[i] - 2*math.pi)
                    elif self.q[i] < -math.pi:
                        self.q[i] = float(self.q[i] + 2*math.pi)
                        k[i] = float(self.q[i] + 2*math.pi)
                    else:

                        k[i] = float(self.q[i])

                # k[i] = float(self.q[i])

                for i in range(6):
                    k[i] = float(self.q[i])  # % (math.pi))

                self.joint_positions.data = k
                print("=====================================")
                print("T:::")
                print(self.t)
                print("q:::")
                print(self.q)
                print("velocity::")
                print(self.velocity)
                print("joint positions::")
                print(self.joint_positions.data)
                print("curr")
                print(x_final, y_final, z_final)
                print("=====================================")
                if (x == 2):
                    self.X.append(x_final)
                    self.Y.append(y_final)
                    self.Z.append(z_final)
                self.t = self.t + dt
                self.joint_position_pub.publish(self.joint_positions)

            self.t = 0
            if (x == 3):
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                print("computing q for 4")
                self.compute_q(4)
                # self.pub_arm_task_status.publish(Int32(data=4))
                # self.compute_q(5)
         # self.compute_q(5)

            if (x == 11):
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                self.compute_q(4)
            if (x == 2):
                self.cur_pos = [x_final, y_final, z_final]
                print(self.cur_pos)
                # switch gripper on
                self.gripper_service.wait_for_service()
                request = SetBool.Request()
                request.data = True
                self.gripper_service.call_async(request)
                # future = self.gipper_service.call_async(request)
                # rclpy.spin_until_future_complete(self, future)
                # response = future.result()
                # print(response)
                print("computing q for 3")
                self.compute_q(1)
                self.pub_arm_task_status.publish(Int32(data=1))
                # self.compute_q(5)

            if (x == 1):
                self.gripper_service.wait_for_service()
                request = SetBool.Request()
                request.data = False
                self.gripper_service.call_async(request)

    def timer_callback(self):

        if (self.cur_goal != self.set_goal):
            self.compute_q(self.set_goal)
            self.cur_goal = self.set_goal
            print("goal updated")

    def plot_trajectory(self, X, Y, Z):

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the points
        ax.scatter(self.X, self.Y, self.Z, color='red', marker='o')

        # Set labels
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        # Set the minimum limit for each axis
        # min_limit =
        ax.set_xlim([float(min(self.X)), float(max(self.X))])
        ax.set_ylim([float(min(self.Y)), float(max(self.Y))])
        ax.set_zlim([float(min(self.Z)), float(max(self.Z))])
        # ax.view_init(elev=10, azim=90)  # Adjust the viewing angle

        # Show the plot
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
