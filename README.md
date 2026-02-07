# CERISE Multi-Robot Nav2

Multi-robot autonomous navigation using ROS2 Humble + Nav2.

## Quick Start (2 Robots)
```bash
# Terminal 1
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py \
  use_rviz:=False autostart:=true use_composition:=False

# Terminal 2: Set initial poses
ros2 topic pub --once /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.5}}}}"
ros2 topic pub --once /robot2/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: -0.5}}}}"

# Send navigation goal
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```

## System
- Ubuntu 22.04 Jammy
- ROS2 Humble (native)
- Nav2 + TurtleBot3 Waffle
- CycloneDDS middleware

## Status
- ✅ 2 robots: VALIDATED (autonomous navigation working)
- ⏳ 4 robots: IN PROGRESS
