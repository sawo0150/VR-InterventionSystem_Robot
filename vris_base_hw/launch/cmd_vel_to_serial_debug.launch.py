from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ----- Launch Arguments -----
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port of Arduino'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.3',
        description='Wheel base (distance between wheels) in meters'
    )

    max_wheel_speed_arg = DeclareLaunchArgument(
        'max_wheel_speed',
        default_value='0.5',
        description='Max wheel speed [m/s] corresponding to max PWM'
    )

    max_pwm_arg = DeclareLaunchArgument(
        'max_pwm',
        default_value='255',
        description='Max PWM magnitude'
    )

    status_interval_arg = DeclareLaunchArgument(
        'status_interval',
        default_value='1.0',
        description='Interval [s] to print status logs'
    )

    # ----- Node -----
    cmd_vel_to_serial_node = Node(
        package='vris_base_hw',
        executable='cmd_vel_to_serial',
        name='cmd_vel_to_serial',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'max_wheel_speed': LaunchConfiguration('max_wheel_speed'),
            'max_pwm': LaunchConfiguration('max_pwm'),
            'status_interval': LaunchConfiguration('status_interval'),
        }],
        # 디버그 로그 레벨
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        wheel_base_arg,
        max_wheel_speed_arg,
        max_pwm_arg,
        status_interval_arg,
        cmd_vel_to_serial_node,
    ])
