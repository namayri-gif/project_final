from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    models_dir = '/root/workspaces/ros2_ws/src/ai-worker-sim/human_detector/models'

    return LaunchDescription([
        Node(
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
    ])