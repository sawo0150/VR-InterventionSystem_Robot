import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 패키지 경로 찾기
    # (주의: 각 패키지가 빌드되어 install 폴더에 launch 파일이 존재해야 합니다)
    pkg_mode_manager = get_package_share_directory('vris_mode_manager')
    pkg_teleop = get_package_share_directory('vris_teleop')
    pkg_base_hw = get_package_share_directory('vris_base_hw')
    pkg_slam = get_package_share_directory('vris_slam')

    return LaunchDescription([
        
        # 1. Mode Manager 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_mode_manager, 'launch', 'vris_mode_manager.launch.py')
            )
        ),

        # 2. Gamepad Joy Manager 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_teleop, 'launch', 'gamepad_joy_manager.launch.py')
            )
        ),

        # 3. Cmd Vel to Serial 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_base_hw, 'launch', 'cmd_vel_to_serial.launch.py')
            )
        ),

        # 4. mapping 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'mapping.launch.py')
            )
        ),
    ])