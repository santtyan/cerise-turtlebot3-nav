#!/usr/bin/env python3
"""
Launch leve: Gazebo + 2 TurtleBot3 + dataset_collector.
Movimento dos robôs é feito via `scripts/calibration_debug/cmd_vel_random.py` (sem Nav2).
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = Path(__file__).parent.parent.resolve()
    world_file = str(pkg_dir / "worlds" / "world_with_camera.world")
    waffle_model = str(pkg_dir / "worlds" / "waffle_nodepth.model")
    pkg_gazebo_ros = "/opt/ros/humble/share/gazebo_ros"

    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg_gazebo_ros}/launch/gzserver.launch.py"),
            launch_arguments={"world": world_file, "pause": "false"}.items(),
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(f"{pkg_gazebo_ros}/launch/gzclient.launch.py")
        )
    )

    ld.add_action(Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "robot1", "-file", waffle_model,
                   "-robot_namespace", "robot1",
                   "-x", "0.0", "-y", "0.5", "-z", "0.01"],
        output="screen",
    ))
    ld.add_action(Node(
        package="gazebo_ros", executable="spawn_entity.py",
        arguments=["-entity", "robot2", "-file", waffle_model,
                   "-robot_namespace", "robot2",
                   "-x", "0.0", "-y", "-0.5", "-z", "0.01"],
        output="screen",
    ))

    ld.add_action(Node(package="cerise_nav", executable="dataset_collector", output="screen"))

    return ld
