# CERISE Multi-Robot Nav2

Multi-robot autonomous navigation for 5G logistics applications (CERISE Lab/UFG).

## Branches
- `main`: Clean 4-robot setup (work in progress)
- `antiga`: 2-robot validated baseline

## Quick Start

Coming soon - 4 robot launch file in development.

See branch `antiga` for working 2-robot setup.

## System
- Ubuntu 22.04 Jammy
- ROS2 Humble (native)
- Nav2 + TurtleBot3 Waffle
- CycloneDDS middleware

## Team
- Researcher: Yan (CERISE Lab/UFG)
- Advisor: Prof. Alisson Assis Cardoso

## Troubleshooting
**Gazebo port conflict**: `killall -9 gzserver gzclient`
**AMCL initialpose**: `./set_initialposes.sh`
**RViz crash**: Use `use_rviz:=False`
