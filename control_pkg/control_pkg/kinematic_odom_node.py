#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class KinematicOdomNode(Node):
    def __init__(self):
        super().__init__('kinematic_odom_node')
        
        # Subscribe to the velocity commands sent to the motors
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Publish the simulated/kinematic odometry
        self.publisher = self.create_publisher(Odometry, '/odom_kinematic', 10)
        
        # Timer to integrate velocity into position at 30 Hz
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # Internal state variables for position and velocity
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        self.get_logger().info("Kinematic Odometry Node (Fake Encoders) Started.")

    def cmd_callback(self, msg: Twist):
        # Update current velocity based on the command received
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def quaternion_from_euler(self, roll, pitch, yaw):
        # Convert euler angles to quaternion for ROS 2 messages
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Integrate velocity over time to calculate the new physical position
        delta_x = (self.vx * math.cos(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw)) * dt
        delta_yaw = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        
        # Construct the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'camera_frame'
        
        # Set the integrated position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set the integrated orientation
        q = self.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set the instantaneous velocities
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        # Publish the simulated odometry
        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()