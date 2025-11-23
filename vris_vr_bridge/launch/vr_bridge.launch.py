import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Launch Arguments 선언 (터미널 입력값)
    # 예: ros2 launch vris_vr_bridge vr_bridge.launch.py camera_type:=insta360 resolution:=1920x1080
    
    signaling_ip_arg = DeclareLaunchArgument(
        'signaling_ip',
        default_value='192.168.0.7',
        description='IP address of the Signaling Server (Unity PC)'
    )

    signaling_port_arg = DeclareLaunchArgument(
        'signaling_port',
        default_value='8080',
        description='Port of the Signaling Server'
    )

    camera_type_arg = DeclareLaunchArgument(
        'camera_type',
        default_value='insta360',
        description='Camera Type: generic or insta360'
    )

    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='2',
        description='Video device index (e.g. /dev/video0 -> 0)'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='1920x1080',
        description='Camera Resolution (e.g., 640x360, 1280x720, 1920x1080)'
    )

    # 2. 노드 설정
    vr_bridge_node = Node(
        package='vris_vr_bridge',
        executable='vr_bridge',
        name='vr_bridge_node',
        output='screen',
        parameters=[{
            'signaling_ip': LaunchConfiguration('signaling_ip'),
            'signaling_port': LaunchConfiguration('signaling_port'),
            'camera_type': LaunchConfiguration('camera_type'),
            'camera_index': LaunchConfiguration('camera_index'),
            'resolution': LaunchConfiguration('resolution'),
        }]
    )

    return LaunchDescription([
        signaling_ip_arg,
        signaling_port_arg,
        camera_type_arg,
        camera_index_arg,
        resolution_arg,
        vr_bridge_node
    ])