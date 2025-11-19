from launch import LaunchDescription
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

    return LaunchDescription([
        gazebo_launch
    ])
