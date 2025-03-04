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
    def __init__(self):
        super().__init__('ebot_waypoint')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/wheel_odom',
            self.current_position_callback,
            10
        )
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        self.sub_arm_status = self.create_subscription(
            Int32,
            '/arm_task_status',
            self.arm_status_callback,
            10
        )
        
        self.send_arm_task = self.create_publisher(
            Int32, '/set_goal', 10)
        
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.timer_period = 0.1  # seconds
        self.timer2 = self.create_timer(self.timer_period, self.timer_callback2)

        self.wheel_base = 0.74
        self.wheel_rad = 0.461/2.0
        self.continue_nav = False

 
        # robot state variables
        self.position_ = Point()
        self.yaw_ = 0
        self.state_ = 0
        self.desired_position_ = Point()
        self.desired_position_.x = 10.0
        self.desired_position_.y = 0.0
        self.desired_position_.z = 0.0
        self.yaw_precision_ = math.pi / 45 # +/- 2 degree allowed
        self.dist_precision_ = 0.3
        self.waypoints = [[26.916,-0.51], [27.22, 2.82], [33.47, 2.82], [33.47, 8.12], [30.47,8.12], [27.7, 8.02] ]

        self.cur_waypoint = -1
        self.wait_for_arm = False

        # self.goal_pose = [10.0, 0.0, 2.0]
        # self.distance_tolerance = 0.2
                
        self.wheel_velocities = Float64MultiArray()
        
        # self.navWaypoint(goal_pose)

    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

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
        # ...
        self.position_ = msg.pose.position
        quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)
        euler = self.euler_from_quaternion(quaternion)
        self.yaw_ = euler[2]
        
        
        
    
    def change_state(self,state):
        self.state_ = state
        # print 'State changed to [%s]' % state_

    def fix_yaw(self,des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_
        if(err_yaw > math.pi):
            err_yaw = err_yaw - 2*math.pi
        elif(err_yaw < -math.pi):
            err_yaw = err_yaw + 2*math.pi
        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision_:
            twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        
            self.pub_vel(twist_msg)
        
        # state change conditions
        if math.fabs(err_yaw) <= self.yaw_precision_:
            print('Yaw error: [%s]' % err_yaw)
            self.change_state(1)

    def go_straight_ahead(self,des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position_.y, des_pos.x - self.position_.x)
        err_yaw = desired_yaw - self.yaw_
        if(err_yaw > math.pi):
            err_yaw = err_yaw - 2*math.pi
        elif(err_yaw < -math.pi):
            err_yaw = err_yaw + 2*math.pi
        # err_yaw =-(err_yaw+math.pi)%(2*math.pi)-math.pi
        err_pos = math.sqrt(pow(des_pos.y - self.position_.y, 2) + pow(des_pos.x - self.position_.x, 2))
        
        if err_pos > self.dist_precision_:
            twist_msg = Twist()
            # twist_msg.linear.x = 0.6
            if(self.cur_waypoint >=4):
                twist_msg.linear.x = 0.05*err_pos
            else:
                twist_msg.linear.x = 0.1*err_pos

            self.pub_vel(twist_msg)
        else:
            # print 'Position error: [%s]' % err_pos
            self.change_state(2)
        
        # state change conditions
        if math.fabs(err_yaw) > self.yaw_precision_:
            print('---------------------------------')
            print('straight Yaw error: [%s]' % err_yaw)
            print('desired_yaw: [%s]' % desired_yaw)
            print('current_yaw: [%s]' % self.yaw_)
            print('---------------------------------')
            self.change_state(0)

    def done(self):
        
        twist_msg = Twist()

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_vel(twist_msg)

    def pub_vel(self, twist_msg):
        
        self.lin_vel = twist_msg.linear.x
        self.ang_vel = twist_msg.angular.z
        self.left_vel =  (self.lin_vel - self.ang_vel*self.wheel_base/2.0)/self.wheel_rad
        self.right_vel = (self.lin_vel + self.ang_vel*self.wheel_base/2.0)/self.wheel_rad
 
    
        self.wheel_velocities.data = [-self.left_vel, -self.right_vel, -self.left_vel, -self.right_vel]
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        
    def timer_callback(self):
        # ...
        if self.state_ == 0:
            # print( len(self.waypoints))
            self.fix_yaw(self.desired_position_)
        elif self.state_ == 1:
            self.go_straight_ahead(self.desired_position_)
        elif self.state_ == 2:
            self.done()                
                
            self.check_arm(self.cur_waypoint)
            if(self.wait_for_arm):
                pass
            else:
                if(self.continue_nav):
                    pass
                else:
                    self.cur_waypoint += 1
                self.continue_nav = False
                if self.cur_waypoint < len(self.waypoints):
                    print( (self.cur_waypoint, self.waypoints[self.cur_waypoint]))
                    self.desired_position_.y = self.waypoints[self.cur_waypoint][1]
                    self.desired_position_.x = self.waypoints[self.cur_waypoint][0]  
                    self.state_ = 0     
        
            
        else:
            # rospy.logerr('Unknown state!')
            pass
        
    def check_arm(self, cur_waypoint):
        
            
        if cur_waypoint == 4 or cur_waypoint == 6 :
            self.wait_for_arm = True
            


             
            
       
    def timer_callback2(self):
        if self.wait_for_arm and self.cur_waypoint == 4:
            self.send_arm_task.publish(Int32(data=2))
        # elif self.wait_for_arm and self.cur_waypoint == 6:
        #     self.send_arm_task.publish(Int32(data=3))
            
           
    def arm_status_callback(self, msg):
        if msg.data == 1:
            self.wait_for_arm = False
            print("recevied msg")
            self.cur_waypoint += 1
            self.continue_nav = True

        elif msg.data == 4:
            self.wait_for_arm = False
            self.cur_waypoint += 1
            self.continue_nav = True
            
        else:
            pass
        # print("Arm status: ", msg.data)

def main(args=None):
    rclpy.init(args=args)
    ebot_waypoint = EbotWaypoint()
    rclpy.spin(ebot_waypoint)
    ebot_waypoint.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
