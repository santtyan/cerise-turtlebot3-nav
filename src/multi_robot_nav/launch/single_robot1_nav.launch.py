from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    
    nav2_launch = GroupAction([
        PushRosNamespace('robot1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'tb3_simulation_launch.py')
            ),
            launch_arguments={
                'namespace': 'robot1',
                'use_namespace': 'true',
                'x_pose': '-2.0',
                'y_pose': '-0.5'
            }.items()
        )
    ])
    
    return LaunchDescription([nav2_launch])
