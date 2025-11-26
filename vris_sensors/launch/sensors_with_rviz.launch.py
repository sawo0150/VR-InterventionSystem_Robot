import os
import yaml  # YAML 파싱을 위해 추가
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Load Configuration from YAML
    # 패키지에 설치된 config 폴더 경로를 찾습니다.
    pkg_share = get_package_share_directory('vris_sensors')
    config_dir = os.path.join(get_package_share_directory('vris_sensors'), 'config')

    # [RViz Config Path] - 추가된 부분
    rviz_config_file = os.path.join(pkg_share, 'rviz2', 'sensor_test.rviz')

    # [LIDAR CONFIG LOAD]
    lidar_config_path = os.path.join(config_dir, 'lidar_a3.yaml')
    with open(lidar_config_path, 'r') as f:
        lidar_yaml = yaml.safe_load(f)
        # YAML 구조: sllidar -> [params...]
        l_params = lidar_yaml['sllidar']

    # [IMU CONFIG LOAD]
    imu_config_path = os.path.join(config_dir, 'imu_taobotics.yaml')
    with open(imu_config_path, 'r') as f:
        imu_yaml = yaml.safe_load(f)
        # YAML 구조: imu_taobotics -> [params...]
        i_params = imu_yaml['imu_taobotics']

    # 2. Lidar Launch Include
    # 설치된 sllidar_ros2 패키지의 런치 파일을 가져옵니다.
    lidar_pkg_dir = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_pkg_dir, 'launch', 'sllidar_a3_launch.py')),
        launch_arguments={
            'channel_type': l_params['channel_type'],
            'serial_port': l_params['serial_port'],
            'serial_baudrate': str(l_params['serial_baudrate']), # 숫자는 문자로 변환
            'frame_id': l_params['frame_id'],
            'inverted': str(l_params['inverted']).lower(),       # Bool -> 'true'/'false'
            'angle_compensate': str(l_params['angle_compensate']).lower(),
            'scan_mode': l_params['scan_mode']
        }.items()
    )

    # 3. IMU Launch Include
    # 설치된 mrpt_sensor_imu_taobotics 패키지의 런치 파일을 가져옵니다.
    imu_pkg_dir = get_package_share_directory('mrpt_sensor_imu_taobotics')
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_pkg_dir, 'launch', 'mrpt_sensor_imu_taobotics.launch.py')),
        launch_arguments={
            'serial_port': i_params['serial_port'],
            'sensor_model': i_params['sensor_model'],
            'sensor_frame_id': i_params['sensor_frame_id'],
            'robot_frame_id': i_params['robot_frame_id']
        }.items()
    )

    # 4. RViz2 Node
    # 단순히 RViz 창만 띄웁니다. (설정 파일이 있다면 arguments=['-d', 'path/to/config.rviz'] 추가)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 5. Static TF Publisher (Optional but Recommended for Visualization)
    # 아직 로봇 모델(URDF)이나 robot_localization이 없으므로, 
    # RViz에서 에러 없이 보려면 임시로 base_link와 센서들을 이어줘야 합니다.
    # (나중에 vris_robot_description이 생기면 삭제해야 함)
    tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.2', '3.141592', '0', '0', 'base_link', l_params['frame_id']]
    )

    tf_base_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.3', '0', '0', '0', 'base_link', i_params['sensor_frame_id']]
    )

    return LaunchDescription([
        lidar_launch,
        imu_launch,
        tf_base_laser, # 임시 TF
        tf_base_imu,   # 임시 TF
        rviz_node
    ])