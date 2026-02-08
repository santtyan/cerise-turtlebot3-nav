# Multi-Robot Autonomous Navigation System

ROS2 Humble + Nav2 + TurtleBot3 multi-robot autonomous navigation for 5G logistics applications.

## Quick Start
```bash
# Clone
git clone git@github.com:santtyan/cerise-turtlebot3-nav.git
cd cerise-turtlebot3-nav

# Build
colcon build --symlink-install
source install/setup.bash

# Run 2 robots (validated)
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py \
  use_rviz:=False autostart:=true use_composition:=False

# Wait 20s, then set initial poses
./set_initialposes.sh

# Navigate robot1
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```

## Status

- ✅ **2 robots**: Fully functional autonomous navigation
- ⏳ **4 robots**: 75% complete (spawn entity conflict)

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- TurtleBot3 packages
- Gazebo 11

## Project Structure
```
├── src/cerise_4robots/     # 4-robot package (WIP)
├── setup.sh                # Environment setup
├── set_initialposes.sh     # Auto initialpose publisher
└── PROGRESS.md             # Detailed progress log
```

## License

Apache 2.0
