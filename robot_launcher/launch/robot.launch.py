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

    car_control_dir = get_package_share_directory('car_control')

    start_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch)
    )


    # Aggiungi questo nodo se non c'è già
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
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
                'range_min': 0.2,              # Ignores things within 30cm (noise)
                'range_max': 4.0,              # Ignore things over 5m
                'scan_height': 300              # How many pixel rows to use (higher = sees obstacles of different heights)
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

    car_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_control_dir, 'launch', 'car_control_with_microros.launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyTHS1', # Assicurati che sia la porta giusta per la Jetson
            'baud_rate': '115200'
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

    real_odom = Node(
        package='control_pkg',
        executable='real_odom',
        name='real_odom',
        output='screen',
        parameters=[ekf_config_path]
    )

    # Car Control Node
    car_control_node = Node(
        package='car_control',
        executable='car_control_node',
        output='screen',
        name='car_control_node',
    )

    return LaunchDescription([
        start_control,
        start_laser,
        real_odom,
        ekf_node,
        ekf_global_node,
        pose_converter_node,
        nav2_launch,
        #rviz_node,
        #car_control_launch,
        car_control_node,
        static_transform_publisher
    ])