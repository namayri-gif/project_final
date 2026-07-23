from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    models_dir = (
        '/root/workspaces/ros2_ws/src/'
        'ai-worker-sim/human_detector/models'
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ffw',
            package_name='ffw_moveit_config'
        )
        .robot_description_semantic(
            file_path='config/ffw.srdf'
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml'
        )
        .joint_limits(
            file_path='config/joint_limits.yaml'
        )
        .planning_pipelines(
            pipelines=['ompl'],
            default_planning_pipeline='ompl'
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml'
        )
        .to_moveit_configs()
    )

    person_detector = Node(
        package='human_detector',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[{
            'weights_path': f'{models_dir}/yolov4-tiny.weights',
            'config_path': f'{models_dir}/yolov4-tiny.cfg',
            'names_path': f'{models_dir}/coco.names',
            'camera_topic': '/zedm/image',
        }]
    )

    wave_interact = Node(
        package='human_detector',
        executable='wave_interact',
        name='wave_interact',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'use_sim_time': False,
            }
        ]
    )

    return LaunchDescription([
        person_detector,
        wave_interact,
    ])