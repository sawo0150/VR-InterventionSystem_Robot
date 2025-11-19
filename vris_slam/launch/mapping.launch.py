#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # 1) 센서 + rf2o + EKF + (옵션) RViz 런치 포함
    #    -> 네가 만든 odom/ekf launch 파일 이름으로 바꿔줘야 함
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vris_sensors'),
                'launch',
                'odom_with_ekf.launch.py'   # 여기를 실제 파일명으로 변경
            ])
        ),
        # 필요하면 여기서 use_rviz:=true/false 등 인자 넘겨도 됨
        # launch_arguments={'use_rviz': 'true'}.items()
    )

    # 2) slam_toolbox 파라미터 파일 경로
    slam_params = PathJoinSubstitution([
        FindPackageShare('vris_slam'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # 3) slam_toolbox 노드 (Async Mapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',   # 실시간 SLAM 노드
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    return LaunchDescription([
        sensors_launch,
        slam_node,
    ])
