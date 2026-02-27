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
            # '-x', '0',
            # '-y', '0',
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

    # ✅ Correct bridge: ROS /cmd_vel -> Gazebo /model/uav_platform/cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/uav/down_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/uav_platform/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            #  pose v for platform
            '/world/mosaic_world/pose/info@gz.msgs.Pose_V@ros_gz_interfaces/msg/Pose_V',
        ],
        remappings=[
            ('/uav/down_camera/image', '/camera/image_raw'),
            ('/model/uav_platform/cmd_vel', '/cmd_vel'),
            # ('/model/uav_platform/pose', '/uav_platform/pose'),
        ],
    )


    # keep only the velocity generator
    '''
    motion_node = Node(
        package='uav_bev_sim',
        executable='motion_controller',
        output='screen',
        parameters=[{
            'topic': '/cmd_vel',
            'forward_speed_mps': 1.0,
            'sideways_speed_mps': 0.0
        }],
    )
    '''

    # planning to circle the map
    '''
    planner_node = Node(
        package='uav_bev_sim',
        executable='coverage_planner',
        output='screen'
    )
    '''

    # Add pubish node 
    
    uav_pose_node = Node(
        package='uav_bev_sim',
        executable='uav_pose_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gz_sim,
        spawn_robot,
        bridge,
        # motion_node,
        # planner_node,
        uav_pose_node,
    ])