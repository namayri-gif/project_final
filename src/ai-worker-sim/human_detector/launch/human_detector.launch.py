import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


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

    # MoveIt configuration is intentionally built inside wave_interact.py.
    # MoveItPy creates an internal rclcpp node, so passing MoveIt parameters
    # only through this launch Node does not configure that internal node.
    wave_interact = Node(
        package='human_detector',
        executable='wave_interact',
        name='wave_interact',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
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

    # Give MoveItPy time to load the robot model and planning pipeline before
    # the detector begins publishing events.
    delayed_person_detector = TimerAction(
        period=10.0,
        actions=[person_detector],
    )

    return LaunchDescription([
        wave_interact,
        delayed_person_detector,
    ])