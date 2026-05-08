#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import the correct message types
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

# Import QoS settings to match the SLAM publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PoseConverterNode(Node):
    def __init__(self):
        super().__init__('pose_converter_node')
        
        # Match the RELIABLE QoS profile of Stella VSLAM
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to SLAM expecting Odometry messages
        self.subscription = self.create_subscription(
            Odometry,
            '/run_slam/camera_pose',
            self.pose_callback,
            qos_reliable)
            
        # Publish to the Global EKF
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/run_slam/camera_pose_cov',
            10)
        
        self.get_logger().info("Pose Converter 2.0 Started: Listening for Odometry...")

    def pose_callback(self, msg: Odometry):
        cov_msg = PoseWithCovarianceStamped()
        cov_msg.header = msg.header
        cov_msg.header.frame_id = 'map'  # SLAM operates in the global map frame
        
        # Extract the pose nested inside the Odometry message
        cov_msg.pose.pose = msg.pose.pose
        
        # Append the required fake covariance matrix for the Kalman Filter (5cm uncertainty)
        cov_msg.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.05, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.05, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.05, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]
        
        self.publisher.publish(cov_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()