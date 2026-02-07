import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    cerise_dir = get_package_share_directory('cerise_4robots')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # Params
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    world = LaunchConfiguration('world', default=os.path.join(bringup_dir, 'worlds', 'world_only.model'))
    params_file_1 = os.path.join(cerise_dir, 'params', 'nav2_multirobot_params_1.yaml')
    params_file_2 = os.path.join(cerise_dir, 'params', 'nav2_multirobot_params_2.yaml')
    params_file_3 = os.path.join(cerise_dir, 'params', 'nav2_multirobot_params_3.yaml')
    params_file_4 = os.path.join(cerise_dir, 'params', 'nav2_multirobot_params_4.yaml')
    
    use_rviz = LaunchConfiguration('use_rviz', default='False')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    
    rviz_config_file = LaunchConfiguration('rviz_config', 
        default=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'))
    
    # Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo_launch.py')),
        launch_arguments={'world': world}.items())
    
    # Robot SDFs
    robot_sdf = os.path.join(bringup_dir, 'worlds', 'waffle.model')
    
    # Robot 1
    spawn_robot1_cmd = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot1', '-file', robot_sdf, '-robot_namespace', 'robot1',
                   '-x', '0.0', '-y', '0.5', '-z', '0.01'], output='screen')
    
    robot1_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': 'robot1', 'map': map_yaml_file, 'use_sim_time': 'True',
                          'params_file': params_file_1, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    robot1_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': 'robot1', 'use_sim_time': 'True',
                          'params_file': params_file_1, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    # Robot 2
    spawn_robot2_cmd = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot2', '-file', robot_sdf, '-robot_namespace', 'robot2',
                   '-x', '0.0', '-y', '-0.5', '-z', '0.01'], output='screen')
    
    robot2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': 'robot2', 'map': map_yaml_file, 'use_sim_time': 'True',
                          'params_file': params_file_2, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    robot2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': 'robot2', 'use_sim_time': 'True',
                          'params_file': params_file_2, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    # Robot 3
    spawn_robot3_cmd = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot3', '-file', robot_sdf, '-robot_namespace', 'robot3',
                   '-x', '0.0', '-y', '1.5', '-z', '0.01'], output='screen')
    
    robot3_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': 'robot3', 'map': map_yaml_file, 'use_sim_time': 'True',
                          'params_file': params_file_3, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    robot3_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': 'robot3', 'use_sim_time': 'True',
                          'params_file': params_file_3, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    # Robot 4
    spawn_robot4_cmd = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'robot4', '-file', robot_sdf, '-robot_namespace', 'robot4',
                   '-x', '0.0', '-y', '-1.5', '-z', '0.01'], output='screen')
    
    robot4_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={'namespace': 'robot4', 'map': map_yaml_file, 'use_sim_time': 'True',
                          'params_file': params_file_4, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    robot4_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': 'robot4', 'use_sim_time': 'True',
                          'params_file': params_file_4, 'autostart': autostart,
                          'use_composition': use_composition, 'use_respawn': use_respawn}.items())
    
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml_file),
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('use_respawn', default_value='False'),
        
        start_gazebo_cmd,
        spawn_robot1_cmd, robot1_localization, robot1_navigation,
        spawn_robot2_cmd, robot2_localization, robot2_navigation,
        spawn_robot3_cmd, robot3_localization, robot3_navigation,
        spawn_robot4_cmd, robot4_localization, robot4_navigation,
    ])
