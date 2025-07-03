#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('lidar_robot_simulation')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_world.world'),
        description='Full path to the world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='lidar_robot',
        description='name of the robot')

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        name='robot_namespace',
        default_value='lidar_robot',
        description='ROS namespace applied to multi-robot systems')

    # Specify the actions
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

    # Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'urdf', 'lidar_robot.urdf.xacro')
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ExecuteProcess(
                cmd=['xacro', urdf_file],
                output='screen'
            ).output
        }]
    )

    # Spawn Robot
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name,
                   '-topic', [robot_namespace, '/robot_description'],
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_namespace_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld