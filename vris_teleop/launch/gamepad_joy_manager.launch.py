from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vris_teleop',
            executable='gamepad_joy_manager',  # 또는 설치한 executable 이름
            name='gamepad_joy_manager',
            output='screen',
            parameters=[
                # 필요하면 여기 나중에 파라미터 추가
            ],
        ),
    ])
