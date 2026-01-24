from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    
    world = os.path.join(pkg_tb3, 'worlds', 'turtlebot3_world.world')
    urdf = os.path.join(pkg_tb3, 'models', 'turtlebot3_waffle', 'model.sdf')
    map_file = os.path.join(nav2_bringup, 'maps', 'turtlebot3_world.yaml')
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    # Spawn robots
    spawn_r1 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-file', urdf, '-x', '-2.0', '-y', '-0.5', '-z', '0.01'])
    
    spawn_r2 = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot2', '-file', urdf, '-x', '2.0', '-y', '0.5', '-z', '0.01'])
    
    # Nav2 robot1
    nav1 = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'map': map_file, 'use_sim_time': 'true'}.items()
        )
    ])
    
    # Nav2 robot2
    nav2 = GroupAction([
        PushRosNamespace('robot2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={'map': map_file, 'use_sim_time': 'true'}.items()
        )
    ])
    
    return LaunchDescription([gazebo, spawn_r1, spawn_r2, nav1, nav2])

