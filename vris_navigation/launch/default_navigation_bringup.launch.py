#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. Argument Definition
    # ---------------------------------------------------------
    # 맵 파일 기본값 (Phase 2에서 만든 office_map 사용)
    map_file_default = PathJoinSubstitution([
        FindPackageShare('vris_slam'),
        'maps',
        'office_map.yaml' 
    ])

    # Nav2 파라미터 파일 (방금 수정한 nav2_params_test.yaml)
    nav2_params_default = PathJoinSubstitution([
        FindPackageShare('vris_navigation'),
        'config',
        'nav2_params_test.yaml'
    ])

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_default,
        description='Full path to map yaml file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_default,
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    # 실제 로봇이므로 기본값 false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ---------------------------------------------------------
    # 2. Include Launches
    # ---------------------------------------------------------
    
    # (A) 센서 + Odom + EKF 실행
    # Phase 2에서 검증된 'sensors_with_odom.launch.py'를 재사용
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vris_sensors'),
                'launch',
                'sensors_with_odom.launch.py'
            ])
        ),
        # Nav2와 함께 쓸 때는 센서 쪽 RViz는 끄는 게 좋음 (Nav2용 RViz 사용)
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # (B) Nav2 Bringup (Map Server + AMCL + Planner + Controller + BT + Lifecycle)
    # nav2_bringup 패키지의 표준 런치를 사용하여 복잡한 설정 없이 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            # 중요: autostart가 true여야 lifecycle manager가 노드들을 active로 만듦
            'autostart': 'true', 
        }.items()
    )

    # (C) RViz2 (Nav2 전용 뷰)
    # Nav2가 제공하는 기본 RViz 설정을 사용하여 실행
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    ])
    
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ),
        launch_arguments={
            'rviz_config': rviz_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        use_sim_time_arg,
        sensors_launch,
        nav2_launch,
        rviz_node
    ])