import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------

    human_detector_share = get_package_share_directory(
        'human_detector'
    )

    ffw_description_share = get_package_share_directory(
        'ffw_description'
    )

    models_dir = os.path.join(
        human_detector_share,
        'models',
    )

    robot_xacro_path = os.path.join(
        ffw_description_share,
        'urdf',
        'ffw_sh5_rev1_follower',
        'ffw_sh5_follower.urdf.xacro',
    )

    # ------------------------------------------------------------
    # Validate required files
    # ------------------------------------------------------------

    required_files = [
        robot_xacro_path,
        os.path.join(models_dir, 'yolov4-tiny.weights'),
        os.path.join(models_dir, 'yolov4-tiny.cfg'),
        os.path.join(models_dir, 'coco.names'),
    ]

    for required_file in required_files:
        if not os.path.exists(required_file):
            raise FileNotFoundError(
                f'Required file does not exist: {required_file}'
            )

    # ------------------------------------------------------------
    # Process the real robot Xacro
    # ------------------------------------------------------------

    robot_description_xml = xacro.process_file(
        robot_xacro_path
    ).toxml()

    robot_description = {
        'robot_description': robot_description_xml,
    }

    # ------------------------------------------------------------
    # MoveIt configuration
    # ------------------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ffw',
            package_name='ffw_moveit_config',
        )
        .robot_description_semantic(
            file_path='config/ffw.srdf',
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml',
        )
        .joint_limits(
            file_path='config/joint_limits.yaml',
        )
        .planning_pipelines(
            pipelines=['ompl'],
            default_planning_pipeline='ompl',
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
        )
        .to_moveit_configs()
    )

    # ------------------------------------------------------------
    # Wave interaction node
    # ------------------------------------------------------------

    wave_interact = Node(
        package='human_detector',
        executable='wave_interact',
        name='wave_interact',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {
                'use_sim_time': True,
            },
        ],
    )

    # ------------------------------------------------------------
    # Person detector
    # ------------------------------------------------------------

    person_detector = Node(
        package='human_detector',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'weights_path': os.path.join(
                models_dir,
                'yolov4-tiny.weights',
            ),
            'config_path': os.path.join(
                models_dir,
                'yolov4-tiny.cfg',
            ),
            'names_path': os.path.join(
                models_dir,
                'coco.names',
            ),
            'camera_topic': '/zedm/image',
        }],
    )

    # Start the detector after the MoveIt interaction node.
    # This does not replace the requirement to launch simulation,
    # controllers, and Nav2 before this launch file.
    delayed_person_detector = TimerAction(
        period=5.0,
        actions=[
            person_detector,
        ],
    )

    return LaunchDescription([
        wave_interact,
        delayed_person_detector,
    ])