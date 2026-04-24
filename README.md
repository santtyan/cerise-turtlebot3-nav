# Multi-Robot Autonomous Navigation System

ROS2 Humble + Nav2 + TurtleBot3 multi-robot autonomous navigation for 5G logistics applications.

## Status

- ✅ **2 robots**: Fully functional autonomous navigation
- ⏳ **4 robots**: 75% complete (spawn entity conflict)

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Nav2
- TurtleBot3 packages
- Gazebo 11

## Installation (WSL Ubuntu 22.04)

### 1. Clone & Build
```bash
git clone https://github.com/santtyan/cerise-turtlebot3-nav.git
cd cerise-turtlebot3-nav

# Build with colcon
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. Source Environment
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 3. Run 2 Robots (Validated ✅)
```bash
# Terminal 1: Launch simulation
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py \
  use_rviz:=False autostart:=true use_composition:=False

# Terminal 2: Wait 20s, then set initial poses
sleep 20
./set_initialposes.sh

# Terminal 2: Send navigation goal
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```

### 4. Run 4 Robots (In Development)
```bash
ros2 launch cerise_4robots cerise_4robots_launch.py \
  use_rviz:=False autostart:=true use_composition:=False
```

## Quick Reference Commands

**Set initial poses (manual)**:
```bash
# Robot 1
ros2 topic pub --once /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.5}}}}"

# Robot 2
ros2 topic pub --once /robot2/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: -0.5}}}}"
```

**Diagnose ROS2**:
```bash
ros2 node list
ros2 topic list
ros2 service list
```

**Kill all processes**:
```bash
pkill -9 gazebo; pkill -9 ros2; pkill -9 gzclient
```

## Project Structure
```
├── src/cerise_4robots/     # 4-robot package (WIP)
├── setup.sh                # Environment setup
├── set_initialposes.sh     # Auto initialpose publisher
└── PROGRESS.md             # Detailed progress log
```

## License

Apache 2.0
