#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory # Import 추가


def generate_launch_description():
    # 0) 경로 설정
    slam_pkg_path = get_package_share_directory('vris_slam')

    # 1) 센서 + rf2o + EKF + (옵션) RViz 런치 포함
    #    -> 네가 만든 odom/ekf launch 파일 이름으로 바꿔줘야 함
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vris_sensors'),
                'launch',
                'sensors_with_odom.launch.py'   # 여기를 실제 파일명으로 변경
            ])
        ),
        # 필요하면 여기서 use_rviz:=true/false 등 인자 넘겨도 됨
        launch_arguments={'use_rviz': 'false'}.items()
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

    # 4) SLAM 전용 RViz2 실행 (추가됨)
    rviz_config_file = os.path.join(slam_pkg_path, 'rviz', 'slam_toolbox_default.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    return LaunchDescription([
        sensors_launch,
        slam_node,
        rviz_node,
    ])
