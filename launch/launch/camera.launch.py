from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os

def generate_launch_description():

    use_sim_time = SetLaunchConfiguration(name='use_sim_time', value='True')

    turtlebot3_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg_dir = get_package_share_directory('gazebo_ros')

    launch_share_dir = get_package_share_directory('launch')
    texture_box_file = os.path.join(launch_share_dir, 'models', 'textured_cube.sdf')
    
    world_file_name = 'turtlebot3_dqn_stage1.world'
    world_path = os.path.join(turtlebot3_pkg_dir, 'worlds', world_file_name)
    
    robot_sdf_path = os.path.join(
        turtlebot3_pkg_dir,
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf'
    )

    set_depth_env = SetEnvironmentVariable(
        name='TURTLEBOT3_GAZEBO_DEPTH',
        value='true'
    )

    gazebo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    spawn_turtlebot_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle_pi', '-file', robot_sdf_path, '-x', '0.2', '-y', '0.2', '-z', '0.01'],
        output='screen',
    )

    spawn_texture_box = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'reference_cube', 
               '-file', texture_box_file], 
    output='screen',
    )

    depth_converter_node = Node(
        package='depth_image_proc',
        executable='convert_metric_node', # Il nodo principale di conversione
        name='depth_converter',
        output='screen',
        
        # Mappiamo i topic di input e output
        remappings=[
        # Input: Topic RAW di profondità e Camera Info
        ('image_raw', '/camera/depth/image_raw'), 
        ('camera_info', '/camera/camera_info'), 
        ('image', '/depth/rect_metric'), 
    ]
    )

    stella_vslam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_slam',
        output='screen', 

        
        arguments=[
            '-v', '/home/samu/dev_libs/stella_vslam_examples/build/orb_vocab.fbow', # Vocabolario
            '-c', '/home/samu/ros2_ws/src/launch/launch/config.yaml', 
            '--map-db-out', '/home/samu/.ros/stella_map_output.msg',
            #'--log-level', 'debug'
        ],

        parameters=[
            {'use_sim_time': True},
            {'publish_tf': False},
        ], 

        remappings=[
            ('/camera/color/image_raw', '/camera/image_raw'), 
            ('/camera/depth/image_raw', '/depth/rect_metric'),
            ('/camera/camera_info', '/camera/camera_info'), 
        ],
    )

    return LaunchDescription([
        use_sim_time,
        set_depth_env,
        gazebo_start,
        spawn_turtlebot_entity,
        spawn_texture_box,
        depth_converter_node,
        stella_vslam_node,
    ])

