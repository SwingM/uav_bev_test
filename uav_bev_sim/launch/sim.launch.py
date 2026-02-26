from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare('uav_bev_sim'), 'worlds', 'mosaic_world.sdf']),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',
            'uav_platform',
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '5',
            '-file',
            PathJoinSubstitution([FindPackageShare('uav_bev_sim'), 'models', 'uav_platform', 'model.sdf']),
        ],
    )

    # Explicit one-way bridges:
    # - image/camera_info: Gazebo -> ROS 2
    # - pose command: ROS 2 -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/uav/down_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/uav/down_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/uav_platform/pose_cmd@geometry_msgs/msg/Pose]gz.msgs.Pose',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/uav/down_camera/image', '/camera/image_raw'),
            ('/uav/down_camera/camera_info', '/camera/camera_info'),
            ('/uav_platform/pose_cmd', '/model/uav_platform/pose'),
        ],
    )

    motion_node = Node(
        package='uav_bev_sim',
        executable='motion_controller',
        output='screen',
        parameters=[{'topic': '/cmd_vel', 'forward_speed_mps': 1.0, 'sideways_speed_mps': 0.0}],
    )


    cmd_vel_pose_node = Node(
        package='uav_bev_sim',
        executable='cmd_vel_pose_controller',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel',
                'pose_topic': '/uav_platform/pose_cmd',
                'fixed_altitude_m': 5.0,
                'rate_hz': 30.0,
                'initial_x': 0.0,
                'initial_y': 0.0,
            }
        ],
    )

    capture_node = Node(
        package='uav_bev_sim',
        executable='image_capture',
        output='screen',
        parameters=[{'camera_topic': '/camera/image_raw', 'output_dir': 'captures'}],
    )

    stitch_node = Node(
        package='uav_bev_sim',
        executable='stitching_node',
        output='screen',
        parameters=[{'camera_topic': '/camera/image_raw', 'output_file': 'captures/mosaic_result.png'}],
    )

    return LaunchDescription(
        [
            world_arg,
            gz_sim,
            spawn_robot,
            bridge,
            motion_node,
            cmd_vel_pose_node,
            capture_node,
            stitch_node,
        ]
    )
