from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    
    world = os.path.join(pkg_tb3, 'worlds', 'turtlebot3_world.world')
    urdf_path = os.path.join(pkg_tb3, 'models', 'turtlebot3_waffle', 'model.sdf')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )
    
    spawn_r1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot1',
            '-file', urdf_path,
            '-x', '-2.0', '-y', '-0.5', '-z', '0.01',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    spawn_r2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot2',
            '-file', urdf_path,
            '-x', '2.0', '-y', '0.5', '-z', '0.01',
            '-Y', '3.14'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_r1,
        spawn_r2
    ])
