import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                # The correct parameter names for recent ROS 2 versions
                'rgb_camera.color_profile': '640x480x30',  
                'rgb_camera.color_format': 'bgr8',   
                'depth_module.depth_profile': '640x480x30', 
                
                # IMU and Sync settings
                'enable_gyro': True,
                'enable_accel': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'unite_imu_method': 2,
                'enable_sync': True,
                'align_depth.enable': True,
                'initial_reset': False
            }],
            output='screen'
        )
    ])