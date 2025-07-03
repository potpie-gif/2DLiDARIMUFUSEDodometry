#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory('lidar_robot_simulation')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_world.world'),
        description='Full path to the world model file to load')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

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
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Spawn Robot
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'lidar_robot',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.0'],
        output='screen')

    # LiDAR Odometry Node
    lidar_odometry_cmd = Node(
        package='lidar_odometry',
        executable='lidar_odometry_node',
        name='lidar_odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic_name': '/ldlidar_node/scan',
            'imu_topic_name': '/imu/mpu6050',
            'odom_topic_name': 'odom'
        }]
    )

    # QR Test Publisher
    qr_test_publisher_cmd = Node(
        package='lidar_odometry',
        executable='qr_test_publisher',
        name='qr_test_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # RViz
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'robot_view.rviz')],
        parameters=[{
            'use_sim_time': True
        }],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(lidar_odometry_cmd)
    ld.add_action(qr_test_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld