from setuptools import find_packages, setup

package_name = 'uav_bev_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/mosaic_world.sdf']),
        ('share/' + package_name + '/models/uav_platform', ['models/uav_platform/model.sdf', 'models/uav_platform/model.config']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Minimal ROS 2 + Gazebo project for BEV image capture and mosaicing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller = uav_bev_sim.motion_controller:main',
            'cmd_vel_pose_controller = uav_bev_sim.cmd_vel_pose_controller:main',
            'image_capture = uav_bev_sim.image_capture:main',
            'stitching_node = uav_bev_sim.stitching_node:main',
            # add coverage node
            'coverage_planner = uav_bev_sim.coverage_planner:main',
            # add publish pose node
            'uav_pose_publisher = uav_bev_sim.uav_pose_publisher:main',
        ],
    },
)
