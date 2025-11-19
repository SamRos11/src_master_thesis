from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    turtlebot3_pkg_dir = get_package_share_directory('turtlebot3_gazebo')

    gazebo_launch_file = os.path.join(
        turtlebot3_pkg_dir,
        'launch',
        'turtlebot3_world.launch.py'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    stella_vslam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_slam',
        output='screen', 

        
        arguments=[
            '-v', '/home/samu/dev_libs/stella_vslam_examples/build/orb_vocab.fbow', # Vocabolario
            '-c', '/home/samu/ros2_ws/src/launch/launch/config.yaml', 
            '--map-db-out', '/home/samu/ros2_ws/src/launch/launch/camera.launch.py/map.msg' 
        ],

        parameters=[
            {'publish_tf': False},
        ], 
    )

    return LaunchDescription([
        gazebo_launch,
        stella_vslam_node
    ])
