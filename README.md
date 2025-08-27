# -Development-of-a-Digital-Twin-for-a-Robotic-Manipulator-using-AC-Servo-motors
ROS 2 digital twin + real robot control of a 3-DOF manipulator with Mitsubishi MR-J2S AC servos (MoveIt 2, ros2_control, Gazebo).

# ROS 2 Manipulator Digital Twin (AC Servo – MR-J2S)

Digital twin and real-robot control for a 3-DOF welding manipulator powered by Mitsubishi MR-J2S AC servo drives. The stack includes **ros2_control** hardware interface, **MoveIt 2** motion planning, and **Gazebo/RViz** visualization.

> **Key results (physical ↔ twin)**: avg. feedback latency **15 ms**, twin update rate **100 Hz**, position accuracy **±0.2°**, 4-hour uptime test stable. :contentReference[oaicite:0]{index=0}

## Features
- URDF/Xacro model with `ros2_control` integration (encoders, limits, gear ratios)
- Custom hardware interface for **AC servo drivers (MR-J2S series)** with homing via limit switches
- MoveIt 2 configs + demo launch; trajectory execution to the real robot
- Gazebo/RViz twin for plan-then-execute workflows
- Controller setup (Joint State Broadcaster, JointTrajectoryController)
- GitHub Actions CI (colcon build + lint)

## Hardware Highlights
- **Servos:** Mitsubishi 200 W (×2) + 100 W (×1) with MR-J2-20A / MR-J2S-10A drivers, 131072 ppr encoders, 100:1 gearboxes (torque-boosted)
- **Controller:** Raspberry Pi (Ubuntu 22.04) generating pulse+dir, limit-switch homing; 1 pulse ≈ 0.01°
- **Calibration:** paper dial 360°, verified pulse→angle mapping
- **Twin link:** ROS 2 DDS feeds twin @100 Hz for live overlays and logs :contentReference[oaicite:1]{index=1}

## Quick Start
```bash
# deps
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install
source install/setup.bash

# RViz view
ros2 launch automation_lab_description rviz.launch.py

# Gazebo sim
ros2 launch automation_lab_description automation.launch.py

# MoveIt 2 demo
ros2 launch automation_lab_moveit_config moveit.launch.py
