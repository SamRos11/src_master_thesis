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
    
    start_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch)
    )

    return LaunchDescription([
        
        #start_camera,

        # Static TF
        Node(
            package='tracking_pkg',
            executable='tracking_node',
            output='screen'
        )
    ])