#!/usr/bin/env python3
import time
import threading
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import evdev
from evdev import InputDevice, ecodes

class GamepadTeleopNode(Node):
    """
    evdev 기반 게임패드 입력을 읽어서 /cmd_vel_robot 으로 Twist를 발행하는 노드.

    - 기본 매핑:
        * Left Stick Y (ABS_Y, code=1)  -> linear.x
        * Left Stick X (ABS_X, code=0)  -> angular.z
      (axis 번호는 파라미터로 바꿀 수 있음)

    - 파라미터:
        * device_path (string): /dev/input/eventX 경로
        * linear_axis (int):   linear.x에 쓸 축 코드 (기본 1)
        * angular_axis (int):  angular.z에 쓸 축 코드 (기본 0)
        * max_linear_vel (double):  최대 선속도 [m/s]
        * max_angular_vel (double): 최대 각속도 [rad/s]
    """

    def __init__(self):
        super().__init__('gamepad_teleop_node')

        # --- ROS 파라미터 선언 ---
        self.declare_parameter('device_path', '/dev/input/event17')
        self.declare_parameter('linear_axis', 1)    # 보통 ABS_Y
        self.declare_parameter('angular_axis', 0)   # 보통 ABS_X
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_angular_vel', 1.0)

        self.device_path = self.get_parameter('device_path').value
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # --- 환경 디버깅: 어떤 파이썬/evdev를 쓰고 있는지 로그 ---
        self.get_logger().info(f"Python executable: {sys.executable}")
        self.get_logger().info(f"evdev module: {evdev.__file__}")
        self.get_logger().info(
            f"evdev version: {getattr(evdev, '__version__', 'unknown')}"
        )

        # --- Gamepad open ---
        # 디버깅용: 현재 연결된 input 디바이스 목록 먼저 출력
        devices = [InputDevice(path) for path in evdev.list_devices()]
        self.get_logger().info("=== Devices ===")
        for d in devices:
            self.get_logger().info(f"{d.path} {d.name} {d.phys}")

        try:
            self.gamepad = InputDevice(self.device_path)
        except FileNotFoundError:
            self.get_logger().error(f"Gamepad device not found: {self.device_path}")
            raise

        # read_one() 자체가 non-blocking 이라 set_nonblocking 불필요(구버전 evdev에는 없음)
        self.get_logger().info(
            f"Using gamepad: {self.gamepad.path} ({self.gamepad.name})"
        )

        # 버튼/축 매핑 (gamepad_test.py 그대로 가져옴 - 디버깅 메시지용)
        self.buttons_map = {
            314: 'Select',
            315: 'Start',
            316: 'Mode',
            312: 'L2',
            313: 'R2',
            310: 'L1',
            311: 'R1',
            307: 'X',
            308: 'Y',
            304: 'A',
            305: 'B',
        }
        self.axes_map = {
            0:  'Left Joystick X',
            1:  'Left Joystick Y',
            2:  'Right Joystick X',
            5:  'Right Joystick Y',
            16: 'D-pad X',
            17: 'D-pad Y',
        }

        # 축 정보 / 현재 값 저장용
        self.axes_info = {}
        self.axis_states = {}
        self.button_states = {}

        self._setup_axes()

        # cmd_vel_robot publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_robot', 10)

        # gamepad_test.py 스타일: 별도 스레드에서 이벤트 루프 실행
        self.get_logger().info("Starting gamepad read thread (read_loop 기반)...")
        self.gamepad_thread = threading.Thread(
            target=self.gamepad_loop, daemon=True
        )
        self.gamepad_thread.start()

    def _setup_axes(self):
        """
        게임패드의 ABS 축 정보(최소/최대/중앙값)를 읽어서 정규화에 사용.
        """
        caps = self.gamepad.capabilities().get(ecodes.EV_ABS, [])
        for code, absinfo in caps:
            self.axes_info[code] = absinfo
            self.axis_states[code] = 0.0

        # 디버그용 로그
        axis_names = []
        for code in self.axes_info.keys():
            if code in ecodes.ABS:
                axis_names.append(f"{ecodes.ABS[code]}({code})")
            else:
                axis_names.append(f"code={code}")
        self.get_logger().info("Detected ABS axes: " + ", ".join(axis_names))

    def _normalize_axis(self, code: int, value: int) -> float:
        """
        EV_ABS 값(value)을 [-1.0, 1.0] 범위로 정규화.
        """
        info = self.axes_info.get(code, None)
        if info is None:
            return 0.0

        min_v = info.min
        max_v = info.max
        # 혹시라도 min/max가 동일하면 0 반환
        if max_v == min_v:
            return 0.0

        # AbsInfo.value 는 "현재 값"이라 초기에는 0일 수 있음.
        # 물리적인 중앙값(예: 0~255 -> 127.5)을 기준으로 사용하는 편이 더 안정적.
        center = (min_v + max_v) / 2.0

        if value >= center:
            denom = (max_v - center) or 1
            norm = (value - center) / float(denom)
        else:
            denom = (center - min_v) or 1
            norm = (value - center) / float(denom)

        # 혹시 범위를 조금 넘으면 클램핑
        if norm > 1.0:
            norm = 1.0
        if norm < -1.0:
            norm = -1.0

        return norm


    def gamepad_loop(self):
        """
        evdev이 제공하는 read_loop()를 그대로 사용:
        - 내부에서 select()로 블록
        - 이벤트가 들어오면 하나씩 yield
        """
        self.get_logger().info("Gamepad loop started (using read_loop).")

        try:
            for event in self.gamepad.read_loop():
                if not rclpy.ok():
                    break

                # 버튼 이벤트
                if event.type == ecodes.EV_KEY:
                    self.button_states[event.code] = event.value

                    name = self.buttons_map.get(event.code, f'Unknown({event.code})')
                    if event.value == 1:
                        state = 'pressed'
                    elif event.value == 0:
                        state = 'released'
                    else:
                        state = f'other({event.value})'

                    self.get_logger().info(
                        f"[KEY] Button {name:14s} (code={event.code}) {state}"
                    )

                # 축(조이스틱, D-Pad) 이벤트
                elif event.type == ecodes.EV_ABS:
                    axis_name = self.axes_map.get(event.code, f'Unknown({event.code})')
                    value = event.value
                    self.get_logger().info(
                        f"[ABS] Axis   {axis_name:14s} (code={event.code}) value={value}"
                    )

                    # 정규화된 값은 /cmd_vel_robot 계산에 사용
                    norm = self._normalize_axis(event.code, value)
                    self.axis_states[event.code] = norm

                    # 현재 축 상태로 Twist 생성
                    lin_raw = self.axis_states.get(self.linear_axis, 0.0)
                    ang_raw = self.axis_states.get(self.angular_axis, 0.0)

                    linear_x = -lin_raw * self.max_linear_vel
                    angular_z = -ang_raw * self.max_angular_vel

                    twist = Twist()
                    twist.linear.x = float(linear_x)
                    twist.angular.z = float(angular_z)

                    if abs(linear_x) > 1e-3 or abs(angular_z) > 1e-3:
                        self.get_logger().info(
                            f"Publishing Twist: linear_x={linear_x:.3f}, angular_z={angular_z:.3f}"
                        )
                    self.cmd_pub.publish(twist)
        except OSError as e:
            self.get_logger().error(f"Error in gamepad read_loop: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 쪽에서 shutdown을 이미 호출하는 경우가 있어서,
        # 여기서 나는 RCLError는 무시해도 됨.
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
