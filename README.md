# UAV BEV Test (ROS 2 Humble + Gazebo)

This repository provides a starter ROS 2 Humble simulation project for **top-down BEV image capture** using a simple flying camera drone in Gazebo.

## What is included

- A Gazebo world with a large visible ground plane.
- A simple drone-like model (`flying_camera`) carrying a downward-facing camera.
- An autopilot node that moves the drone in a lawnmower (coverage) pattern.
- An image saver node that stores camera frames as PNG files.

## 1) Prerequisites (Ubuntu 22.04)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cv-bridge \
  python3-opencv
```

> Use Gazebo Classic with ROS 2 Humble (`gazebo_ros`).

## 2) Build

```bash
source /opt/ros/humble/setup.bash
cd /workspace/uav_bev_test
colcon build --symlink-install
source install/setup.bash
```

## 3) Run simulation + autopilot + image saving

```bash
source /opt/ros/humble/setup.bash
source /workspace/uav_bev_test/install/setup.bash
ros2 launch bev_drone_project sim_bev.launch.py
```

Saved BEV images will be created in:

```text
bev_images/
```

## 4) Tune for your scene

You can override autopilot parameters when launching the autopilot separately:

```bash
ros2 run bev_drone_project autopilot_node --ros-args \
  -p altitude_m:=25.0 \
  -p ground_size_m:=80.0 \
  -p stripe_spacing_m:=6.0 \
  -p speed_mps:=3.0
```

And tune save rate:

```bash
ros2 run bev_drone_project image_saver_node --ros-args \
  -p save_every_n_frames:=5 \
  -p save_dir:=/tmp/bev_images
```

## Notes

- This starter uses a **kinematic pose-set autopilot** through `/gazebo/set_entity_state` for simple coverage flight.
- For realistic drone dynamics + PX4/ArduPilot SITL, replace `autopilot_node` with a MAVLink-based stack.
