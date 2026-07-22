#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='ffw_sh5_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('worlds', default_value='warehouse_distribution',
                              description='Gz sim World'),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('worlds')

    ffw_description_path = os.path.join(
        get_package_share_directory('ffw_description'))

    ffw_bringup_path = os.path.join(
        get_package_share_directory('ffw_bringup'))

    library_world_path = os.path.join(
        get_package_share_directory('warehouse_worlds'))

    # ─── Kill any stale processes from previous launches ───────────────────────
    kill_old_bridges = ExecuteProcess(
        cmd=['bash', '-c',
            'pkill -9 -f parameter_bridge || true; '
            'pkill -9 -f robot_state_publisher || true; '
            'pkill -9 -f dual_laser_merger || true; '
            'sleep 2; '
            'echo "KILL DONE - safe to launch"'
        ],
        output='screen'
    )

    # ─── Cleanup on Ctrl+C so next launch is always fresh ──────────────────────
    cleanup_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=[
            ExecuteProcess(
                cmd=['bash', '-c',
                    'pkill -9 -f parameter_bridge || true; '
                    'pkill -9 -f robot_state_publisher || true; '
                    'pkill -9 -f dual_laser_merger || true; '
                    'echo "SHUTDOWN CLEANUP DONE"'
                ],
                output='screen'
            )
        ])
    )

    # ─── Environment ───────────────────────────────────────────────────────────
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(ffw_bringup_path, 'worlds'), ':' +
            os.path.join(library_world_path, 'worlds'), ':' +
            os.path.join(library_world_path, 'models'), ':' +
            str(Path(ffw_description_path).parent.resolve()), ':' +
            str(Path(get_package_share_directory('realsense2_description')).parent.resolve())
        ])

    # ─── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', [world, '.sdf', ' -v 1', ' -r', ' -s'])
        ]
    )

    # ─── Robot description ─────────────────────────────────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'),
                              'urdf',
                              model,
                              'ffw_sh5_follower.urdf.xacro']),
        ' ',
        'model:=', model,
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    # ─── Nodes ─────────────────────────────────────────────────────────────────
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.2',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '3.14',
                   '-name', model,
                   '-allow_renaming', 'true',
                   '-use_sim', 'true'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30'],
        output='screen'
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-ros-args',
            '-r /arm_l_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            '--controller-ros-args',
            '-r /arm_r_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            '--controller-ros-args',
            '-r /hand_l_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory',
            '--controller-ros-args',
            '-r /hand_r_controller/joint_trajectory:='
            '/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory',
            '--controller-ros-args',
            '-r /head_controller/joint_trajectory:='
            '/leader/joystick_controller_left/joint_trajectory',
            '--controller-ros-args',
            '-r /lift_controller/joint_trajectory:='
            '/leader/joystick_controller_right/joint_trajectory',
            'arm_l_controller',
            'arm_r_controller',
            'head_controller',
            'lift_controller',
            'hand_l_controller',
            'hand_r_controller',
            'effort_l_controller',
            'effort_r_controller',
            'swerve_drive_controller',
        ],
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    gz_bridge_params_path = os.path.join(
        ffw_bringup_path, 'config', 'common', 'gz_bridge.yaml'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    dual_laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        output='screen',
        parameters=[{
            'laser_1_topic': '/scan_left',
            'laser_2_topic': '/scan_right',
            'merged_scan_topic': '/scan',
            'target_frame': 'base_link',
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.006544985,
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 20.0,
            'use_inf': True,
            'tolerance': 0.05,
            'queue_size': 10,
            'enable_shadow_filter': True,
            'enable_average_filter': True,
        }, {
            'use_sim_time': True,
        }],
    )

    rviz_config_file = os.path.join(ffw_description_path, 'rviz', 'ffw_sh5.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    tilt_head = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal',
                '/head_controller/follow_joint_trajectory',
                'control_msgs/action/FollowJointTrajectory',
                '{"trajectory": {"joint_names": ["head_joint1"], '
                '"points": [{"positions": [0.3], '
                '"time_from_start": {"sec": 3, "nanosec": 0}}]}}'],
            output='screen'
        )]
    )

    # ─── Controller event chain ────────────────────────────────────────────────
    spawn_jsb_on_entity_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_controllers_on_jsb_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # ─── Launch description ────────────────────────────────────────────────────
    return LaunchDescription([
        *declared_arguments,

        # Register handlers first (they only listen, don't launch anything yet)
        cleanup_on_shutdown,
        spawn_jsb_on_entity_exit,
        spawn_controllers_on_jsb_exit,

        # Environment must be set before gazebo starts
        gazebo_resource_path,

        # Step 1: kill stale processes
        # kill_old_bridges,

        # Step 2: after kill completes, launch everything
        TimerAction(period=4.0, actions=[
            gazebo,
            robot_state_pub_node,
            bridge,
            dual_laser_merger_node,
            gz_spawn_entity,
            rviz,
            tilt_head,
        ]),
    ])
