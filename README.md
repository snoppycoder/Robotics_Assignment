# SAR Rescue Robot 
---
## Project Overview

A ROS2 + Gazebo simulation of a differential-drive rescue robot that navigates a collapsed-building environment, uses a LiDAR sensor for obstacle avoidance, and detects simple victim markers placed in the world.

### Key features
- Differential-drive robot model and Gazebo world representing a collapsed building.
- LiDAR bridged to ROS2 as `/scan` and a PointCloud2 topic `/scan/points`.
- Simple victim markers (colored boxes) and a `victim_detector` node that publishes `/victim_detected` when the robot approaches a marker.
---
## Quick start
1. From the workspace root:
```bash
colcon build
source install/setup.bash
```
2. Launch the simulation:
```bash
ros2 launch sar_robot_description launch_robot.launch.py
```
