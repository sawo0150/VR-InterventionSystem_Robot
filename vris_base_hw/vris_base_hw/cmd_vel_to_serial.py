#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import threading


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # ----- 파라미터 -----
        # ros2 run 할 때, 또는 yaml로 override 가능
        self.declare_parameter('port', '/dev/ttyACM0')   # 아두이노 포트
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.3)        # 바퀴 간 거리 [m] (대략값, 나중에 조정)
        
        self.declare_parameter('max_wheel_speed', 0.5)   # 각 바퀴 최대 속도 [m/s], 나중에 조정
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('status_interval', 1.0)   # 상태 로그 주기 [s]
        self.declare_parameter('send_interval', 0.05)    # 아두이노로 명령 보내는 주기 [s] (0.05s = 20Hz)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.status_interval = self.get_parameter('status_interval').get_parameter_value().double_value
        self.send_interval = self.get_parameter('send_interval').get_parameter_value().double_value


        # ----- 시리얼 오픈 -----
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        # 시리얼은 쓰기만 할 거지만, multi-thread 안전하게 lock 사용
        self.serial_lock = threading.Lock()

        # 디버깅용 변수
        self.last_twist_time = self.get_clock().now()
        self.last_pwm_l = 0
        self.last_pwm_r = 0

        # 주기적으로 상태 출력
        if self.status_interval > 0.0:
            self.status_timer = self.create_timer(
                self.status_interval, self.status_timer_callback
            )

        # 주기적으로 마지막 PWM을 아두이노로 전송
        if self.send_interval > 0.0:
            self.send_timer = self.create_timer(
                self.send_interval, self.send_timer_callback
            )

        # ----- /cmd_vel_robot 서브스크라이버 -----
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_robot',
            self.cmd_vel_callback,
            10
        )

    def reopen_serial(self):
        """시리얼 에러 발생 시 포트 재연결 시도"""
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Re-opened serial port {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Re-open failed: {e}")
            self.ser = None

    def status_timer_callback(self):
        """ROS 쪽 상태를 주기적으로 출력"""
        now = self.get_clock().now()
        dt = (now - self.last_twist_time).nanoseconds / 1e9
        self.get_logger().info(
            f"[STATUS] dt_since_last_twist={dt:.3f}s, last_pwm_l={self.last_pwm_l}, last_pwm_r={self.last_pwm_r}"
        )

    def send_timer_callback(self):
        """마지막 PWM 값을 주기적으로 아두이노로 전송"""
        if self.ser is None:
            return

        cmd_str = f"{self.last_pwm_l} {self.last_pwm_r}\n"

        try:
            with self.serial_lock:
                self.ser.write(cmd_str.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error in send_timer: {e}")
            self.reopen_serial()

    def cmd_vel_callback(self, msg: Twist):

        v = msg.linear.x     # [m/s]
        w = msg.angular.z    # [rad/s]

        # ----- 좌/우 바퀴 속도 계산 -----
        # differential drive
        v_r = v + (self.wheel_base / 2.0) * w
        v_l = v - (self.wheel_base / 2.0) * w

        # ----- 속도 → PWM 변환 -----
        # max_wheel_speed 기준으로 -max_pwm ~ max_pwm 으로 스케일
        def vel_to_pwm(v_wheel: float) -> int:
            if self.max_wheel_speed <= 0.0:
                return 0
            ratio = v_wheel / self.max_wheel_speed
            # -1.0 ~ 1.0로 클램프
            if ratio > 1.0:
                ratio = 1.0
            elif ratio < -1.0:
                ratio = -1.0
            pwm = int(ratio * self.max_pwm)
            # 안전 범위 클램프
            if pwm > self.max_pwm:
                pwm = self.max_pwm
            if pwm < -self.max_pwm:
                pwm = -self.max_pwm
            return pwm

        pwm_l = vel_to_pwm(v_l)
        pwm_r = vel_to_pwm(v_r)

        # 디버깅용 상태 저장 (마지막 명령 갱신만 하고, 실제 전송은 타이머에서 수행)
        self.last_twist_time = self.get_clock().now()
        self.last_pwm_l = pwm_l
        self.last_pwm_r = pwm_r

        # 디버그
        self.get_logger().debug(
            f"cmd_vel v={v:.3f}, w={w:.3f} -> "
            f"v_l={v_l:.3f}, v_r={v_r:.3f}, "
            f"pwm_l={pwm_l}, pwm_r={pwm_r}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
