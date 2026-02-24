from setuptools import setup
from glob import glob

package_name = 'bev_drone_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models/flying_camera', glob('models/flying_camera/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='BEV drone simulation project with ROS 2 Humble and Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot_node = bev_drone_project.autopilot_node:main',
            'image_saver_node = bev_drone_project.image_saver_node:main',
        ],
    },
)
