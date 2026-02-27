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
        default_value=PathJoinSubstitution([
            FindPackageShare('uav_bev_sim'), 'worlds', 'mosaic_world.sdf'
        ]),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'uav_platform',
            '-x', '-5.5',
            '-y', '7.5',
            '-z', '5',
            '-file',
            PathJoinSubstitution([
                FindPackageShare('uav_bev_sim'),
                'models',
                'uav_platform',
                'model.sdf'
            ]),
        ],
    )

    # NOTE: keep Humble-compatible pose bridge.
    # Do not switch this back to Pose_V (/world/.../pose/info),
    # because ros_gz_interfaces/msg/Pose_V may be unavailable on Humble installs.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/uav/down_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/uav_platform/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/model/uav_platform/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
        ],
        remappings=[
            ('/uav/down_camera/image', '/camera/image_raw'),
            ('/model/uav_platform/cmd_vel', '/cmd_vel'),
        ],
    )

    uav_pose_node = Node(
        package='uav_bev_sim',
        executable='uav_pose_publisher',
        output='screen',
        parameters=[{
            'input_pose_topic': '/model/uav_platform/pose',
            'output_topic': '/uav_platform/pose',
        }],
    )

    uav_autopilot_node = Node(
        package='uav_bev_sim',
        executable='uav_autopilot',
        output='screen',
        parameters=[{
            'pose_topic': '/uav_platform/pose',
            'cmd_vel_topic': '/cmd_vel',
            'x_min': -8.0,
            'x_max': 10.0,
            'y_min': -8.0,
            'y_max': 8.0,
            'target_altitude_m': 5.0,
            'camera_interval_s': 0.7,
            'camera_footprint_x_m': 4.0,
            'camera_footprint_y_m': 3.0,
            'coverage_overlap_ratio': 0.25,
            'max_speed_mps': 1.2,
            'position_tolerance_m': 0.25,
            'control_rate_hz': 10.0,
        }],
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        spawn_robot,
        bridge,
        uav_pose_node,
        uav_autopilot_node,
    ])
