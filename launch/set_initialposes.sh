#!/bin/bash
ros2 topic pub --once /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}" &
ros2 topic pub --once /robot2/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}" &
wait
echo "✅ Initial poses set for robot1 and robot2"
