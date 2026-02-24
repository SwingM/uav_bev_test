from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bev_drone_project')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(pkg_share, 'worlds', 'bev_world.world')
    model = os.path.join(pkg_share, 'models', 'flying_camera', 'model.sdf')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'flying_camera', '-file', model, '-x', '0', '-y', '0', '-z', '20'],
        output='screen',
    )

    autopilot = Node(
        package='bev_drone_project',
        executable='autopilot_node',
        output='screen',
    )

    image_saver = Node(
        package='bev_drone_project',
        executable='image_saver_node',
        output='screen',
    )

    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        spawn,
        autopilot,
        image_saver,
    ])
