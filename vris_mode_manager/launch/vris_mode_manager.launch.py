from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vris_mode_manager',
            executable='mode_manager',   # setup.py / CMakeLists에 등록한 이름
            name='vris_mode_manager',
            output='screen',
            parameters=[
                # 나중에 필요하면 파라미터 추가
            ],
        ),
    ])
