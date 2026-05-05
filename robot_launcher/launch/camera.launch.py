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
                'rgb_camera.profile': '1280x720x30',  
                'rgb_camera.color_format': 'bgr8',   
                'depth_module.profile': '1280x720x30',
                'enable_gyro': True,
                'enable_accel': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'unite_imu_method': 2,
                'enable_sync': True,
                'align_depth.enable': True,
                'initial_reset': True
            }],
            output='screen'
        )
    ])