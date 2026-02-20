"""Launch file for the RC car self-driving stack with Gazebo simulation.

Starts:
- Gazebo with the rc_car_world scene.
- robot_state_publisher with the xacro-processed URDF.
- The filtering_and_perception node.
- The planning node.
- The driver node.
- (Optional) RViz2 for visualisation.

Usage
-----
ros2 launch robot_bringup robot.launch.py

Optional arguments:
  use_rviz:=true|false   Whether to launch RViz2 (default: true).
  use_sim_time:=true     Use Gazebo simulation clock (default: true).
"""

# Copyright (c) 2024 Maintainer
#
# SPDX-License-Identifier: MIT

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Return the LaunchDescription for the RC car simulation stack."""
    bringup_pkg = get_package_share_directory('robot_bringup')

    urdf_file = os.path.join(bringup_pkg, 'urdf', 'rc_car.urdf.xacro')
    world_file = os.path.join(bringup_pkg, 'worlds', 'rc_car_world.sdf')
    rviz_config = os.path.join(bringup_pkg, 'config', 'rviz_config.rviz')

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualisation.',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # ------------------------------------------------------------------ #
    # Gazebo                                                               #
    # ------------------------------------------------------------------ #
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Robot state publisher                                                #
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time,
        }],
    )

    # ------------------------------------------------------------------ #
    # Spawn the robot in Gazebo                                            #
    # ------------------------------------------------------------------ #
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'rc_car',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Stack nodes                                                          #
    # ------------------------------------------------------------------ #
    filtering_and_perception_node = Node(
        package='filtering_and_perception',
        executable='filtering_and_perception_node',
        name='filtering_and_perception_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    driver_node = Node(
        package='driver',
        executable='driver_node',
        name='driver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------ #
    # RViz2 (optional)                                                     #
    # ------------------------------------------------------------------ #
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        filtering_and_perception_node,
        planning_node,
        driver_node,
        rviz2_node,
    ])
