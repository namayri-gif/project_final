# Copyright 2026 ROBOTIS CO., LTD.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_navigation = get_package_share_directory('ffw_navigation')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='navigation.rviz',
        description='RViz config filename inside ffw_navigation/rviz',
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time',
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Use SLAM Toolbox instead of map server + AMCL',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            pkg_navigation,
            'config',
            'navigation.yaml',
        ),
        description='Full path to the Nav2 parameter file',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_navigation,
            'maps',
            'map.yaml',
        ),
        description='Full path to the map YAML file',
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py',
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py',
    )

    slam_params_path = os.path.join(
        pkg_navigation,
        'config',
        'mapper_params_online_sync.yaml',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'map': LaunchConfiguration('map'),
        }.items(),
        condition=IfCondition(
            NotSubstitution(LaunchConfiguration('use_slam'))
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_navigation,
                'launch',
                'online_sync_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_path,
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                pkg_navigation,
                'rviz',
                LaunchConfiguration('rviz_config'),
            ]),
        ],
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'),
                value_type=bool,
            ),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # RViz works reliably when it starts after Nav2 has begun activating its
    # lifecycle nodes and TF/costmap publishers.
    delayed_rviz = TimerAction(
        period=8.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        sim_time_arg,
        use_slam_arg,
        params_file_arg,
        map_arg,
        slam_launch,
        localization_launch,
        navigation_launch,
        delayed_rviz,
    ])