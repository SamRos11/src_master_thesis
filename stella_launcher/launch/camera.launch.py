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

    home_dir = os.environ['HOME'] 

    share_dir = get_package_share_directory('stella_launcher')  

    config_path = os.path.join(share_dir, 'launch', 'config.yaml')
    vocab_path = os.path.join(share_dir, 'launch', 'vocab', 'orb_vocab.fbow')
    map_db_path = os.path.join(home_dir, '.ros/stella_map_output.msg')

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
        ('image', '/depth/rect_metric'), #output
    ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_link']
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        parameters=[{
            'initial_reset': True,
            'align_depth.enable': True,
            'pointcloud.enable': False,
            'enable_sync': True,
            'config_file': config_path,
            # Disable IMU to stop Hardware Errors
            'enable_gyro': False,
            'enable_accel': False,
            'unite_imu_method': 0,
            
            # Force Native Resolutions (HD)
            # We match what the camera WANTS to do, so it doesn't fight us.
            'depth_module.profile': '848x480x15',
            'rgb_camera.profile': '1280x720x15',
            
            # Auto Exposure helps with synchronization
            'enable_auto_exposure': True
        }],
        output='screen'
    )

    stella_vslam_node = Node(
        package='stella_vslam_ros',
        executable='run_slam',
        name='stella_slam',
        output='screen', 
        
        arguments=[
            '-v', vocab_path,
            '-c', config_path,
            '--map-db-out', map_db_path,
        ],
        parameters=[
            {'use_sim_time': False},
            {'publish_tf': True},
        ], 
        remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'), 
            ('/camera/depth/image_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/camera_info', '/camera/camera/color/camera_info'), 
        ],
    )

    return LaunchDescription([
        #use_sim_time,
        #set_depth_env,
        #gazebo_start,
        #spawn_turtlebot_entity,
        #spawn_texture_box,
        #depth_converter_node,
        realsense_node,
        stella_vslam_node,
        static_tf
    ])

