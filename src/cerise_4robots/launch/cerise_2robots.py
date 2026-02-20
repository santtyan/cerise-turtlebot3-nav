import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    world = os.path.join(bringup_dir, 'worlds', 'world_only.model')
    map_yaml = os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml')
    cerise_pkg = get_package_share_directory('cerise_4robots')  	
    robot_sdf_template  = os.path.join(cerise_pkg, 'models', 'robot{}_waffle.model')
    
    # Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        output='screen')
    
    robots = [
        ('robot1', '0.0', '0.5', '1'),
        ('robot2', '0.0', '-0.5', '2'),
    ]
    
    nodes = [start_gazebo]
    
    # Spawn robots com entity único
    for name, x, y, idx in robots:
        nodes.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', f'{name}_waffle',  # UNIQUE ENTITY NAME
                '-file', robot_sdf_template.format(idx),
                '-robot_namespace', name,
                '-x', x, '-y', y, '-z', '0.01'
            ],
            output='screen'))
    
    # Nav2 stacks
    cerise_dir = get_package_share_directory('cerise_4robots')
    for name, x, y, idx in robots:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'tb3_simulation_launch.py')),
            launch_arguments={
                'namespace': name,
                'use_namespace': 'True',
                'map': map_yaml,
                'use_sim_time': 'True',
                'params_file': os.path.join(cerise_dir, 'params', f'nav2_multirobot_params_{idx}.yaml'),
                'autostart': 'true',
                'use_rviz': 'False',
                'use_simulator': 'False',
                'headless': 'False',
                'x_pose': x,
                'y_pose': y,
            }.items()))
    
    return LaunchDescription(nodes)
