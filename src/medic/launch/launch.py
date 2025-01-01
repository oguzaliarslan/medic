#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Launch Turtlebot3 Gazebo
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # Launch Navigation2
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Launch SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ])
    )

    # Launch waypoint navigator
    waypoint_navigator = Node(
        package='medic',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_x': [1.0, 3.0, 2.0],
            'waypoint_y': [1.0, 1.0, 3.0],
            'waypoint_yaw': [0.0, 0.0, 0.0],
            'waypoint_names': ['Room1', 'Room2', 'Room3']
        }]
    )

    ld = LaunchDescription()
    ld.add_action(turtlebot3_gazebo)
    ld.add_action(slam)
    ld.add_action(navigation)
    ld.add_action(waypoint_navigator)
    
    return ld