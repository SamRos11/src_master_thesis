import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'robot_launcher'
    pkg_share = get_package_share_directory(pkg_name)
    
    camera_launch = os.path.join(pkg_share, 'launch', 'camera.launch.py')
    
    config_file = os.path.join(pkg_share, 'config', 'stella_config.yaml')
    vocab_file = os.path.join(pkg_share, 'config', 'orb_vocab.fbow')

    start_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch)
    )

    tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    tf_base_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.2', '0', '0.4', '0', '0', '0', 'base_link', 'camera_link']
    )

    slam_node = Node(
            package='stella_vslam_ros',
            executable='run_slam',
            name='run_slam',
            arguments=['-v', vocab_file, '-c', config_file],
            output='screen',
            remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'), 
            ('/camera/depth/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/camera_info', '/camera/camera/color/camera_info'), 
        ],
    )

    return LaunchDescription([
        tf_odom_base,
        tf_base_camera,
        slam_node
    ])