#!/usr/bin/env python3
"""
Launch file para simular 2 TurtleBot3 Waffle com câmera overhead em Gazebo.
GUI enabled para visualização.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup dinâmico do launch."""

    pkg_dir = Path(__file__).parent.parent
    world_file = pkg_dir / "world_with_camera.model"

    return [
        # Gazebo com mundo customizado
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py"
            ),
            launch_arguments={
                "world": str(world_file),
                "pause": "false",
                "gui": "true",
            }.items(),
        ),

        # Spawn robot1
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robot1",
                "-file", str(pkg_dir / "waffle_nodepth.model"),
                "-robot_namespace", "robot1",
                "-x", "0.0",
                "-y", "0.5",
                "-z", "0.01",
            ],
            output="screen",
        ),

        # Spawn robot2
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robot2",
                "-file", str(pkg_dir / "waffle_nodepth.model"),
                "-robot_namespace", "robot2",
                "-x", "0.0",
                "-y", "-0.5",
                "-z", "0.01",
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch Gazebo GUI",
        ),
        OpaqueFunction(function=launch_setup),
    ])
