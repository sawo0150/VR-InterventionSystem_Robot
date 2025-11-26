#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class CmdVelToSerial(Node):
    """
    /cmd_vel_robot (Twist) → 좌/우 PWM 변환 후 시리얼로 전송하는 노드.
    - 로봇: 2-wheel differential drive
    - 입력: 선속도 v [m/s], 각속도 w [rad/s]
    - 출력: 왼/오 PWM (-max_pwm ~ max_pwm)
    - PWM ↔ wheel velocity 관계는 'angular calibration'으로부터 유도:
        회전 실험에서 얻은 관계:
            |omega_z| ≈ ang_slope * |pwm| + ang_intercept
        그리고 pure rotation에서:
            |omega_z| = 2 * |v_wheel| / wheel_base
        => |v_wheel| ≈ (wheel_base / 2) * (ang_slope * |pwm| + ang_intercept)
        이를 역으로 풀어서:
            |pwm| ≈ (2 * |v_wheel| / wheel_base - ang_intercept) / ang_slope
    """

    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # ----- 파라미터 선언 -----
        # 시리얼 관련
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # 로봇 기구학
        # 사용자가 알려준 값: 바퀴 사이 거리 200mm = 0.2 m
        self.declare_parameter('wheel_base', 0.2)  # [m]

        # PWM 한계
        self.declare_parameter('max_pwm', 255)

        # Angular calibration (|omega_z| vs |pwm|)
        # 기본값은 대충 넣어두고, 실제 값은 launch에서 override해서 사용
        self.declare_parameter('ang_slope', 0.006)      # [rad/s] / PWM
        self.declare_parameter('ang_intercept', 0.0)    # [rad/s]

        # 상태/전송 주기
        self.declare_parameter('status_interval', 1.0)  # [s]
        self.declare_parameter('send_interval', 0.05)   # [s]  (20 Hz)

        # ----- 파라미터 읽기 -----
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.ang_slope = self.get_parameter('ang_slope').get_parameter_value().double_value
        self.ang_intercept = self.get_parameter('ang_intercept').get_parameter_value().double_value
        self.status_interval = self.get_parameter('status_interval').get_parameter_value().double_value
        self.send_interval = self.get_parameter('send_interval').get_parameter_value().double_value

        # ----- 시리얼 오픈 -----
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        # 시리얼은 쓰기만 하지만, multi-thread 안전하게 lock 사용
        self.serial_lock = threading.Lock()

        # 디버깅용 상태 변수
        self.last_twist_time = self.get_clock().now()
        self.last_pwm_l = 0
        self.last_pwm_r = 0

        # 주기적으로 상태 출력
        if self.status_interval > 0.0:
            self.status_timer = self.create_timer(
                self.status_interval, self.status_timer_callback
            )

        # 주기적으로 마지막 PWM 값을 아두이노로 전송
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

        self.get_logger().info(
            f"wheel_base={self.wheel_base:.3f} m, "
            f"max_pwm={self.max_pwm}, "
            f"ang_slope={self.ang_slope:.6f}, ang_intercept={self.ang_intercept:.6f}"
        )

    # ------------------------------------------------------------------
    # 유틸 함수들
    # ------------------------------------------------------------------

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
            f"[STATUS] dt_since_last_twist={dt:.3f}s, "
            f"last_pwm_l={self.last_pwm_l}, last_pwm_r={self.last_pwm_r}"
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

    # ------------------------------------------------------------------
    # 핵심: wheel velocity → PWM 매핑
    # ------------------------------------------------------------------

    def wheel_vel_to_pwm(self, v_wheel: float) -> int:
        """
        바퀴 선속도 [m/s] -> PWM.
        angular calibration 관계식만을 이용해서 역변환한다.

        |omega_z| ≈ ang_slope * |p| + ang_intercept
        pure rotation에서 |omega_z| = 2 * |v_w| / wheel_base

        => 2 * |v_w| / wheel_base ≈ ang_slope * |p| + ang_intercept
           |p| ≈ (2 * |v_w| / wheel_base - ang_intercept) / ang_slope
        """
        if self.ang_slope <= 0.0 or self.wheel_base <= 0.0:
            # 잘못된 파라미터면 그냥 0
            return 0

        sign = 1 if v_wheel >= 0.0 else -1
        v_abs = abs(v_wheel)

        # 역변환 식
        numerator = 2.0 * v_abs / self.wheel_base - self.ang_intercept

        if numerator <= 0.0:
            pwm_mag = 0.0
        else:
            pwm_mag = numerator / self.ang_slope

        # 클램프
        if pwm_mag > float(self.max_pwm):
            pwm_mag = float(self.max_pwm)

        pwm = int(round(pwm_mag)) * sign

        # 안전 범위 한 번 더 클램프
        if pwm > self.max_pwm:
            pwm = self.max_pwm
        if pwm < -self.max_pwm:
            pwm = -self.max_pwm

        return pwm

    # ------------------------------------------------------------------
    # 콜백: /cmd_vel_robot
    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg: Twist):
        
        v = msg.linear.x     # [m/s]
        w = msg.angular.z    # [rad/s]

        # ----- 좌/우 바퀴 속도 계산 (2-wheel kinematics) -----
        # Lmotor_vel, Rmotor_vel
        Lmotor_vel = v - (self.wheel_base / 2.0) * w
        Rmotor_vel = v + (self.wheel_base / 2.0) * w

        # ----- 속도 → PWM 변환 -----
        # 요구 스펙:
        #  - |vel| <= 0.2      -> PWM = 0
        #  - vel =  0.2        -> PWM =  100
        #  - vel =  0.75       -> PWM =  250
        #  - vel >=  0.75      -> PWM =  250 고정
        #  - 음수 방향은 대칭 (부호만 반대)
        def vel_to_pwm(v_wheel: float) -> int:
            # deadband
            if -0.2 <= v_wheel <= 0.2:
                return 0

            # 양수 방향
            if v_wheel > 0.0:
                # 상한 클램프
                if v_wheel >= 0.75:
                    return 250
                # 0.2 ~ 0.75 사이 선형 보간
                # 0.2 -> 100, 0.75 -> 250
                pwm = 100 + (v_wheel - 0.2) * (250 - 100) / (0.75 - 0.2)
                return int(pwm)

            # 음수 방향 (대칭 적용)
            # 여기까지 왔으면 v_wheel < 0.0 이고 deadband는 이미 제외됨
            if v_wheel <= -0.75:
                return -250
            abs_v = abs(v_wheel)
            pwm = 100 + (abs_v - 0.2) * (250 - 100) / (0.75 - 0.2)
            return -int(pwm)

        pwm_l = vel_to_pwm(Lmotor_vel)
        pwm_r = vel_to_pwm(Rmotor_vel)

        self.get_logger().debug(
            f"cmd_vel v={v:.3f}, w={w:.3f} -> "
            f"Lmotor_vel={Lmotor_vel:.3f}, Rmotor_vel={Rmotor_vel:.3f}, "
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
