#!/usr/bin/env python3
"""
Launch completo CERISE: Gazebo + 3 robôs + TF estático map→odom + Nav2 (sem AMCL/map).
yolo_detector, task_allocator e demand_generator rodam em terminais separados.

Uso: ros2 launch cerise_nav cerise_full.launch.py
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # resolve() ANTES de navegar — segue o symlink do launch instalado até o repo
    pkg_dir    = Path(__file__).resolve().parent.parent
    world_file = str(pkg_dir / "worlds" / "world_with_camera.world")
    waffle     = str(pkg_dir / "worlds" / "waffle_nodepth.model")
    nav_launch = "/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py"
    gz_dir     = "/opt/ros/humble/share/gazebo_ros"

    robots = [
        {"name": "robot1", "x": "-0.55", "y":  "0.55", "params": str(pkg_dir / "config" / "params_r1.yaml")},
        {"name": "robot2", "x":  "0.55", "y":  "0.55", "params": str(pkg_dir / "config" / "params_r2.yaml")},
        {"name": "robot3", "x":  "0.55", "y": "-0.55", "params": str(pkg_dir / "config" / "params_r3.yaml")},
    ]

    ld = LaunchDescription()

    # Gazebo server + cliente
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gz_dir}/launch/gzserver.launch.py"),
        launch_arguments={"world": world_file, "pause": "false"}.items(),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gz_dir}/launch/gzclient.launch.py")
    ))

    # Spawn 3 robôs
    for r in robots:
        ld.add_action(Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=["-entity", r["name"], "-file", waffle,
                       "-robot_namespace", r["name"],
                       "-x", r["x"], "-y", r["y"], "-z", "0.01"],
            output="screen",
        ))

    # TF estático map→odom e base_footprint→base_link por robô
    for r in robots:
        # map → odom na pose de spawn (odom nasce na pose de spawn)
        ld.add_action(TimerAction(period=6.0, actions=[Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"static_map_odom_{r['name']}",
            arguments=["--x", r["x"], "--y", r["y"], "--z", "0.0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            remappings=[("/tf", f"/{r['name']}/tf"),
                        ("/tf_static", f"/{r['name']}/tf_static")],
            output="screen",
        )]))
        # base_footprint → base_link (TurtleBot3: z=0.010m)
        ld.add_action(TimerAction(period=6.0, actions=[Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"static_fp_bl_{r['name']}",
            arguments=["--x", "0", "--y", "0", "--z", "0.010",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "base_footprint", "--child-frame-id", "base_link"],
            remappings=[("/tf", f"/{r['name']}/tf"),
                        ("/tf_static", f"/{r['name']}/tf_static")],
            output="screen",
        )]))
        # base_link → base_scan (laser, offset do modelo: x=-0.064, z=0.121)
        ld.add_action(TimerAction(period=6.0, actions=[Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"static_bl_bs_{r['name']}",
            arguments=["--x", "-0.064", "--y", "0", "--z", "0.121",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "base_link", "--child-frame-id", "base_scan"],
            remappings=[("/tf", f"/{r['name']}/tf"),
                        ("/tf_static", f"/{r['name']}/tf_static")],
            output="screen",
        )]))

    # Nav2 (navigation_launch — sem AMCL/map_server) namespaced por robô
    for r in robots:
        ld.add_action(TimerAction(period=12.0, actions=[GroupAction([
            PushRosNamespace(r["name"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "namespace":    r["name"],
                    "use_sim_time": "true",
                    "params_file":  r["params"],
                    "use_composition": "False",
                }.items(),
            ),
        ])]))

    return ld
