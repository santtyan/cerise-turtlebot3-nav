import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cerise = get_package_share_directory('cerise_4robots')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = os.path.join(pkg_nav2_bringup, 'worlds', 'world_only.model')
    
    gzserver_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world],
        output='screen'
    )
    
    robots = [
        ('robot1', '0.0', '0.5', '1'),
        ('robot2', '0.0', '-0.5', '2'),
    ]
    
    spawn_commands = []
    nav2_bringup_commands = []
    
    for robot_name, x, y, node_suffix in robots:
        model_path = os.path.join(pkg_cerise, 'models', f'{robot_name}_waffle.model')
        params_file = os.path.join(pkg_cerise, 'params', f'nav2_multirobot_params_{node_suffix}.yaml')
        
        spawn_commands.append(
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', f'{robot_name}_waffle',
                     '-file', model_path,
                     '-robot_namespace', robot_name,
                     '-x', x, '-y', y, '-z', '0.01'],
                output='screen'
            )
        )
        
        spawn_commands.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot_name,
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[os.path.join(pkg_nav2_bringup, 'worlds', 'waffle.model')]
            )
        )
        
        nav2_bringup_commands.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'True',
                    'use_sim_time': 'true',
                    'params_file': params_file,
                    'autostart': 'true',
                    'use_composition': 'True',
                }.items()
            )
        )
    
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    
    for cmd in spawn_commands:
        ld.add_action(cmd)
    
    for cmd in nav2_bringup_commands:
        ld.add_action(cmd)
    
    return ld
