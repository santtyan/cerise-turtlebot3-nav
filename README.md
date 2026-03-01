# CERISE Multi-Robot Nav2
Multi-robot autonomous navigation using ROS2 Humble + Nav2.

## Quick Start (2 Robots)
```bash
# Terminal 1 - Gazebo + Nav2
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup unique_multi_tb3_simulation_launch.py \
  use_rviz:=False autostart:=true use_composition:=False

# Terminal 2 - Initial poses (wait for AMCL ready)
source install/setup.bash && ./set_initialposes.sh

# Terminal 3 - Task allocator
source install/setup.bash && ros2 run cerise_nav task_allocator

# Terminal 4 - Demand generator
source install/setup.bash && ros2 run cerise_nav demand_generator
```

## System
- Ubuntu 22.04 Jammy
- ROS2 Humble (native)
- Nav2 + TurtleBot3 Waffle
- CycloneDDS middleware

## Architecture
```
demand_generator → /demands → task_allocator → Nav2 NavigateToPose
                                    ↑
                          /robot1/odom, /robot2/odom
```

## Status
- ✅ 2 robots: VALIDATED (autonomous navigation working)
- ✅ Task allocation: VALIDATED (nearest-free-robot, queue, latency monitoring)
- ⏳ 4 robots: IN PROGRESS
