#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math

class RealOdometryNode(Node):
    def __init__(self):
        super().__init__('real_odometry_node')
        
        # Listen to the REAL speed coming from the Arduino encoder
        self.create_subscription(Float64, '/measured_speed', self.speed_callback, 10)
        
        # Listen to cmd_vel ONLY to know the steering angle of the servo
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publish the standard Odometry message for the Local EKF
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        
        # Variables for Kinematic calculations
        self.real_velocity = 0.0
        self.steering_angle = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Distance between front and rear axles (Wheelbase)
        self.wheelbase = 0.34  # Meters 
        
        self.last_time = self.get_clock().now()
        
        # Timer to calculate and publish odometry at 20 Hz
        self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info("Real Odometry Node started! Fusing Arduino speed and steering.")

    def speed_callback(self, msg: Float64):
        self.real_velocity = msg.data

    def cmd_vel_callback(self, msg: Twist):
        self.steering_angle = msg.angular.z

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # --- ACKERMANN KINEMATICS ---
        if abs(self.real_velocity) > 0.01:
            ds = self.real_velocity * dt
            d_theta = (self.real_velocity / self.wheelbase) * math.tan(self.steering_angle) * dt
            
            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
            self.theta += d_theta

        # --- CREATE ODOMETRY MESSAGE ---
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Set Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        
        # Manual Quaternion calculation for Yaw-only rotation
        # This completely avoids the transforms3d numpy deprecation error
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)

        # Add Covariance
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05

        self.odom_publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = RealOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()