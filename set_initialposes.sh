#!/bin/bash
ros2 topic pub --once /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.5}}}}" &
ros2 topic pub --once /robot2/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: -0.5}}}}" &
ros2 topic pub --once /robot3/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 1.5}}}}" &
ros2 topic pub --once /robot4/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: -1.5}}}}" &
wait
echo "✅ All initial poses set"
