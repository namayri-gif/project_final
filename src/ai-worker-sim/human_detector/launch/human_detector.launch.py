import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    human_detector_share = get_package_share_directory(
        'human_detector'
    )

    models_dir = os.path.join(
        human_detector_share,
        'models',
    )

    required_model_files = {
        'weights': os.path.join(models_dir, 'yolov4-tiny.weights'),
        'config': os.path.join(models_dir, 'yolov4-tiny.cfg'),
        'names': os.path.join(models_dir, 'coco.names'),
    }

    for description, file_path in required_model_files.items():
        if not os.path.exists(file_path):
            raise FileNotFoundError(
                f'Missing detector {description} file: {file_path}'
            )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ffw',
            package_name='ffw_moveit_config',
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(
            pipelines=['ompl'],
            default_planning_pipeline='ompl',
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
        )
        .moveit_cpp(
            file_path='config/moveit_cpp.yaml',
        )
        .to_moveit_configs()
    )

    # use_sim_time is supplied here only.
    wave_interact = Node(
        package='human_detector',
        executable='wave_interact',
        name='wave_interact',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': True,
            },
        ],
    )

    person_detector = Node(
        package='human_detector',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'weights_path': required_model_files['weights'],
            'config_path': required_model_files['config'],
            'names_path': required_model_files['names'],
            'camera_topic': '/zedm/image',
        }],
    )

    delayed_person_detector = TimerAction(
        period=10.0,
        actions=[person_detector],
    )

    return LaunchDescription([
        wave_interact,
        delayed_person_detector,
    ])
