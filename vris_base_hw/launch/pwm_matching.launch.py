#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 경로 (launch 내에서 substitution용)
    base_hw_share = FindPackageShare('vris_base_hw')

    # PWMmatching 디렉토리
    pwm_matching_dir = PathJoinSubstitution([
        base_hw_share,
        'PWMmatching'
    ])

    # 설정 파일 기본값
    config_default = PathJoinSubstitution([
        pwm_matching_dir,
        'pwm_matching_config.yaml'
    ])


    # ---------------- IMU 런치 설정 ----------------
    # vris_sensors/config/imu_taobotics.yaml을 읽어서
    # mrpt_sensor_imu_taobotics.launch.py에 파라미터로 넘겨준다.

    # vris_sensors 패키지의 실제 파일 경로
    vris_sensors_share_dir = get_package_share_directory('vris_sensors')
    sensors_config_dir = os.path.join(vris_sensors_share_dir, 'config')

    imu_config_path = os.path.join(sensors_config_dir, 'imu_taobotics.yaml')
    with open(imu_config_path, 'r') as f:
        imu_yaml = yaml.safe_load(f)
        # YAML 구조: imu_taobotics -> {serial_port, sensor_model, sensor_frame_id, robot_frame_id, ...}
        i_params = imu_yaml['imu_taobotics']

    # mrpt_sensor_imu_taobotics 패키지의 런치 파일 포함
    imu_pkg_dir = get_package_share_directory('mrpt_sensor_imu_taobotics')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_pkg_dir, 'launch', 'mrpt_sensor_imu_taobotics.launch.py')
        ),
        launch_arguments={
            'serial_port': i_params['serial_port'],
            'sensor_model': i_params['sensor_model'],
            'sensor_frame_id': i_params['sensor_frame_id'],
            'robot_frame_id': i_params['robot_frame_id'],
        }.items()
    )

    # (옵션) imu 토픽 이름을 바꾸고 싶으면 여기서 arg/param 추가

    pwm_tool_node = Node(
        package='vris_base_hw',
        executable='pwm_matching_tool',  # setup.py에서 entry_points로 등록해야 함
        name='pwm_matching_tool',
        output='screen',
        # 필요하면 환경변수/param로 포트/설정 전달도 가능 (지금 코드는 내부 기본값 사용)
        # parameters=[ ...]
    )

    return LaunchDescription([
        imu_launch,
        pwm_tool_node,
    ])
