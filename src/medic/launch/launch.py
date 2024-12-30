#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='A1_world.world',
        description='Name of the world file to load'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='1.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    
    # Construct the full path to the world file using PathJoinSubstitution
    world_file_path = PathJoinSubstitution([
        FindPackageShare('medic'),
        'worlds',
        LaunchConfiguration('world')
    ])
    
    # Launch Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Include TurtleBot3 robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3
    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'spawn_turtlebot3.launch.py'
            ])
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Launch our robot controller
    robot_controller = Node(
        package='medic',
        executable='robot_controller',
        name='medic_robot_controller',
        output='screen'
    )
    
    perception_node = Node(
        package='medic',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  # Add this line
    )

    obstacle_detection_node = Node(
        package='medic',
        executable='obstacle_detection_node',
        name='obstacle_detection_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  # Add this line
    
    )

    navigation_node = Node(
        package='medic',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]  # Add this line
    )


    # Create and return launch description
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(world_arg)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_turtlebot)
    ld.add_action(robot_controller)

    ld.add_action(perception_node)
    ld.add_action(obstacle_detection_node)
    ld.add_action(navigation_node)
    
    return ld