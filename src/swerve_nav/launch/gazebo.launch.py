# Copyright (c) 2026 Koji Shibata
# SPDX-License-Identifier: MIT
# Adapted from the upstream Gazebo launch; source and full license: ../LICENSE.

"""Spawn the robot in Gazebo and activate its joint controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogError,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('swerve_nav')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    robot_xml = xacro.process_file(
        os.path.join(pkg, 'urdf', 'robot.urdf.xacro'),
        mappings={'controllers_config': controllers_yaml},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                LaunchConfiguration('gui'),
                if_value='-r empty.sdf',
                else_value='-s -r empty.sdf',
            ),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_xml, value_type=str),
            'use_sim_time': True,
        }],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-topic', '/robot_description',
            '-name', 'swerve_robot',
            # base_link is at floor level; leave a small initial clearance.
            '-z', '0.02',
        ],
    )

    # Pass the YAML explicitly so each controller receives its own parameters.
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'steering_controller',
            'drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
            '--controller-manager-timeout', '60',
        ],
    )

    def after_spawn(event, context):
        if event.returncode != 0:
            return [
                LogError(msg='Robot creation failed; stopping simulation.'),
                Shutdown(reason='Robot creation failed'),
            ]
        return [controller_spawner]

    def after_controllers(event, context):
        if event.returncode != 0:
            return [
                LogError(msg='Controller activation failed; stopping simulation.'),
                Shutdown(reason='Controller activation failed'),
            ]
        return []

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Open Gazebo GUI; false runs only the simulation server.',
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot, on_exit=after_spawn,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=controller_spawner, on_exit=after_controllers,
        )),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
    ])
