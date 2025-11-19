import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ---------------------------------------------------------
    # 1. Launch Arguments & Config Setup
    # ---------------------------------------------------------
    pkg_vris_sensors = get_package_share_directory('vris_sensors')
    
    # RViz 실행 여부를 결정하는 인자 (Default: True)
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start Rviz2'
    )
    
    # Path Configurations
    config_dir = os.path.join(pkg_vris_sensors, 'config')
    rviz_config_file = os.path.join(pkg_vris_sensors, 'rviz2', 'odom_test.rviz')
    
    # YAML Paths
    lidar_yaml_path = os.path.join(config_dir, 'lidar_a3.yaml')
    imu_yaml_path = os.path.join(config_dir, 'imu_taobotics.yaml')
    rf2o_config_path = os.path.join(config_dir, 'rf2o_odom.yaml')
    ekf_config_path = os.path.join(config_dir, 'ekf_odom.yaml')

    # ---------------------------------------------------------
    # 2. Load Parameters manually for "IncludeLaunchDescription"
    # ---------------------------------------------------------
    # Lidar Params Load
    with open(lidar_yaml_path, 'r') as f:
        lidar_yaml = yaml.safe_load(f)
        # [주의] 이전 단계에서 수정한 Simplified YAML 구조('sllidar')를 따른다고 가정
        # 만약 'sllidar_node' -> 'ros__parameters' 구조라면 아래 주석 해제 후 사용
        # l_params = lidar_yaml['sllidar_node']['ros__parameters']
        l_params = lidar_yaml['sllidar']

    # IMU Params Load
    with open(imu_yaml_path, 'r') as f:
        imu_yaml = yaml.safe_load(f)
        i_params = imu_yaml['imu_taobotics']

    # ---------------------------------------------------------
    # 3. Nodes & Includes
    # ---------------------------------------------------------
    
    # (1) Lidar Driver
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch', 'sllidar_a3_launch.py'
            ])
        ),
        launch_arguments={
            'channel_type': l_params['channel_type'],
            'serial_port': l_params['serial_port'],
            'serial_baudrate': str(l_params['serial_baudrate']),
            'frame_id': l_params['frame_id'],
            'inverted': str(l_params['inverted']).lower(),
            'angle_compensate': str(l_params['angle_compensate']).lower(),
            'scan_mode': l_params['scan_mode'],
        }.items(),
    )

    # (2) IMU Driver
    # [중요] EKF가 '/taobotics/sensor' 토픽을 듣도록 설정되어 있으므로, 
    # IMU 드라이버의 출력이 그 토픽인지 확인하거나 Remap 해야 합니다.
    # 보통 mrpt driver는 /imu/data 등을 내보내므로 안전하게 Remapping을 추가합니다.
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mrpt_sensor_imu_taobotics'),
                'launch', 'mrpt_sensor_imu_taobotics.launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': i_params['serial_port'],
            'sensor_model': i_params['sensor_model'],
            'sensor_frame_id': i_params['sensor_frame_id'],
            'robot_frame_id': i_params['robot_frame_id'],
        }.items(),
    )

    # (3) Static TF Publishers
    # base_link -> laser (Lidar 위치)
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', l_params['frame_id']]
    )
    
    # base_link -> imu_link (IMU 위치)
    # mrpt 드라이버가 TF를 안 쏠 경우를 대비해 추가 (0.1m 높이 가정)
    tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', i_params['sensor_frame_id']]
    )

    # (4) RF2O Laser Odometry (Lidar -> Odom 계산)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[rf2o_config_path],
    )

    # (5) Robot Localization (EKF)
    # /odom_rf2o + IMU -> /odometry/filtered (TF: odom->base_link)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        # [중요] EKF Config에서 imu0: /taobotics/sensor 로 되어 있다면 
        # 실제 IMU 토픽과 맞추기 위해 Remap이 필요할 수 있음. 
        # 만약 IMU 드라이버가 /imu/data를 낸다면 아래 주석 해제하여 매칭.
        # remappings=[('/taobotics/sensor', '/imu/data')] 
    )

    # (6) RViz2 (Conditioned)
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        use_rviz_arg,
        sllidar_launch,
        imu_launch,
        tf_base_laser,
        tf_base_imu,
        rf2o_node,
        ekf_node,
        rviz_node
    ])