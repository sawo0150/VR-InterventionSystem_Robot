from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vris_vr_bridge',
            executable='map_visualizer_node',  # setup.py에서 등록한 이름
            name='map_visualizer',
            output='screen',
            parameters=[
                {
                    'map_topic': '/map',
                    'global_frame': 'map',
                    'base_frame': 'base_link',
                    'image_topic': '/vr_localization_image',
                    'publish_rate': 1.0,  # Hz
                }
            ],
        ),
    ])
