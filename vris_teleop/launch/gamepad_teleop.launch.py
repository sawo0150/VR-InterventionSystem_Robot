from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
    #     Node(
    #         package='vris_teleop',
    #         executable='gamepad_teleop',
    #         name='gamepad_teleop',
    #         output='screen',
    #         parameters=[{
    #             'device_path': '/dev/input/event17',  # 필요시 수정
    #             'linear_axis': 1,
    #             'angular_axis': 0,
    #             'max_linear_vel': 0.4,
    #             'max_angular_vel': 1.0,
    #         }]
    #     )

        Node(
            package='vris_teleop',
            executable='gamepad_joy',
            name='gamepad_joy',
            output='screen'
        )
    ])
