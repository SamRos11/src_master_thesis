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
    slam_launch = os.path.join(pkg_share, 'launch', 'slam.launch.py')
    tracking_launch = os.path.join(pkg_share, 'launch', 'tracking.launch.py')

    start_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch)
    )

    start_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch)
    )

    start_tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tracking_launch)
    )

    start_control = Node(
        package='control_pkg',
        executable='control_node',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        start_camera,
        start_slam,
        start_tracking,
        start_control,

    ])