#!/usr/bin/env python3

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
        default_value='0.2',  # 200mm
        description='Wheel base (distance between wheels) in meters'
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

    send_interval_arg = DeclareLaunchArgument(
        'send_interval',
        default_value='0.05',
        description='Interval [s] to send PWM to Arduino (e.g., 0.05s = 20Hz)'
    )

    # Angular calibration parameters (from PWM–angular_vel experiment)
    ang_slope_arg = DeclareLaunchArgument(
        'ang_slope',
        default_value='0.006',
        description='Slope in |omega_z| ≈ ang_slope * |pwm| + ang_intercept'
    )

    ang_intercept_arg = DeclareLaunchArgument(
        'ang_intercept',
        default_value='0.0',
        description='Intercept in |omega_z| ≈ ang_slope * |pwm| + ang_intercept'
    )

    # ----- Node -----
    cmd_vel_to_serial_node = Node(
        package='vris_base_hw',
        executable='cmd_vel_to_serial_calibrated',
        name='cmd_vel_to_serial_calibrated',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'max_pwm': LaunchConfiguration('max_pwm'),
            'status_interval': LaunchConfiguration('status_interval'),
            'send_interval': LaunchConfiguration('send_interval'),
            'ang_slope': LaunchConfiguration('ang_slope'),
            'ang_intercept': LaunchConfiguration('ang_intercept'),
        }],
        # 디버그 로그 보고 싶으면 아래 주석 풀기
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        wheel_base_arg,
        max_pwm_arg,
        status_interval_arg,
        send_interval_arg,
        ang_slope_arg,
        ang_intercept_arg,
        cmd_vel_to_serial_node,
    ])
