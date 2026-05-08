import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'robot_launcher'
    pkg_share = get_package_share_directory(pkg_name)
    
    control_launch = os.path.join(pkg_share, 'launch', 'control.launch.py')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot.launch_test.rviz')
    ekf_config_path = os.path.join(
        get_package_share_directory('robot_launcher'),
        'config',
        'ekf_local.yaml'
    )
    ekf_global_config_path = os.path.join(
        get_package_share_directory('robot_launcher'),
        'config',
        'ekf_global.yaml'
    )

    start_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )

    start_laser = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
                ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
                ('scan', '/scan') 
            ],
            parameters=[{
                'output_frame': 'camera_link', 
                'range_min': 0.3,              # Ignores things within 30cm (noise)
                'range_max': 5.0,              # Ignore things over 5m
                'scan_height': 50              # How many pixel rows to use (higher = sees obstacles of different heights)
            }]
    )
    

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file], 
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config_path],
        remappings=[
            ('odometry/filtered', 'odometry/global') # Cambiamo nome al topic per non confonderlo con quello locale
        ]
    )

    pose_converter_node = Node(
    package='control_pkg',
    executable='pose_converter',
    name='pose_converter_node'
)

    kinematic_odom = Node(
        package='control_pkg',
        executable='kinematic_odom',
        name='kinematic_odom',
        output='screen',
        parameters=[ekf_config_path]
    )

    motor_node = Node(
        package='control_pkg',
        executable='motor_driver',
        name='motor_driver',
        output='screen'
    )

    return LaunchDescription([
        start_control,
        start_laser,
        kinematic_odom,
        ekf_node,
        ekf_global_node,
        pose_converter_node,
        nav2_launch,
        rviz_node,
        motor_node
    ])