#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # VR Bridge (WebRTC + 상태/맵 전송)
        Node(
            package='vris_vr_bridge',
            executable='vr_bridge_nodeManager',   # setup.py에서 등록한 executable 이름
            name='vr_bridge_nodeManager',
            output='screen',
            parameters=[
                {
                    'signaling_ip': '192.168.0.7',   # 필요시 launch 인자로 빼도 됨
                    'signaling_port': 8080,
                    'camera_type': 'generic',
                    'camera_index': 0,
                    'resolution': '640x360',
                }
            ],
        ),

        # SLAM 맵 + TF → 로컬라이제이션 이미지 생성
        Node(
            package='vris_vr_bridge',
            executable='map_visualizer_node',  # 앞에서 만든 노드
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
