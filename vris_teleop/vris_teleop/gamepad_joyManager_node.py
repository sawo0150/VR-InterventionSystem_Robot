#!/usr/bin/env python3

# gamepad_test.py (ROS2 /cmd_vel_robot 퍼블리셔 버전)

import evdev
from evdev import InputDevice, ecodes
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

def select_gamepad_device():
    """phys에 'input0' 포함된 디바이스를 우선 선택, 없으면 첫 번째 디바이스."""
    devices = [InputDevice(path) for path in evdev.list_devices()]
    # print("=== Devices ===")
    # for d in devices:
    #     print(d.path, d.name, d.phys)

    if not devices:
        raise RuntimeError("No input devices found.")

    # 1순위: phys에 'input0'이 들어간 디바이스
    for d in devices:
        if d.phys and 'input0' in d.phys:
            # print(f"Selected (phys contains 'input0'): {d.path} {d.name} {d.phys}")
            return d

    # 없으면 걍 첫 번째
    d = devices[0]
    # print(f"No 'input0' in phys. Fallback to: {d.path} {d.name} {d.phys}")
    return d

# 1) 연결된 input 디바이스 목록 출력
devices = [InputDevice(path) for path in evdev.list_devices()]
# print("=== Devices ===")
for d in devices:
    print(d.path, d.name, d.phys)


gamepad = select_gamepad_device()

# 2-1) ROS2 초기화 및 퍼블리셔 생성
rclpy.init()
node = rclpy.create_node('gamepad_joy_manager')

# 왼쪽 스틱 → cmd_vel_joy, 오른쪽 스틱 → cmd_vel_vr
joy_pub = node.create_publisher(Twist, '/cmd_vel_joy', 10)
vr_pub = node.create_publisher(Twist, '/cmd_vel_vr', 10)

# 오류 상태 / 모드 상태 토픽
error_pub = node.create_publisher(Bool, '/system_error', 10)
mode_pub = node.create_publisher(String, '/operation_mode', 10)

# 내부 상태
error_state = False       # False: 정상, True: 오류
mode_state = "AUTO"       # "AUTO" 또는 "VR"

# 초기 상태 한 번 전송
err_msg = Bool()
err_msg.data = error_state
error_pub.publish(err_msg)

mode_msg = String()
mode_msg.data = mode_state
mode_pub.publish(mode_msg)

# 현재 조이스틱 축 상태 저장용 (raw 값)
axis_state = {
    0: 128,  # Left X
    1: 128,  # Left Y
    2: 128,  # Right X
    5: 128,  # Right Y
    16: 0,   # D-pad X (-1,0,1)
    17: 0,   # D-pad Y (-1,0,1)
}

# 속도 스케일
MAX_LINEAR_VEL = 0.4   # m/s
MAX_ANGULAR_VEL = 3.0  # rad/s

# 3) 버튼 매핑
buttons = {
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

# 4) 축(조이스틱/십자키) 매핑
axes = {
    0:  'Left Joystick X',
    1:  'Left Joystick Y',
    2:  'Right Joystick X',
    5:  'Right Joystick Y',
    16: 'D-pad X',
    17: 'D-pad Y',
}


# print("버튼/조이스틱 입력 대기 중... (Ctrl+C 로 종료)\n")

try:
    while rclpy.ok():
        event = gamepad.read_one()

        # 이벤트가 없으면 잠깐 쉬고 다시 루프
        if event is None:
            time.sleep(0.001)
            continue

        # 키(버튼) 이벤트
        if event.type == ecodes.EV_KEY:
            name = buttons.get(event.code, f'Unknown({event.code})')

            if event.value == 1:
                state = 'pressed'
            elif event.value == 0:
                state = 'released'
            else:
                # 2(autorepeat) 같은 값이 올 수도 있어서 표시만 해줌
                state = f'other({event.value})'

            # 버튼 눌림(pressed)에 따라 상태 토픽 업데이트
            if event.value == 1:
                # X 버튼: 오류 발생
                if name == 'X':
                    error_state = True
                    msg = Bool()
                    msg.data = True
                    error_pub.publish(msg)

                # Y 버튼: 정상 복귀
                elif name == 'Y':
                    error_state = False
                    msg = Bool()
                    msg.data = False
                    error_pub.publish(msg)

                # A 버튼: 자율주행(AUTO 모드)
                elif name == 'A':
                    mode_state = "AUTO"
                    m = String()
                    m.data = mode_state
                    mode_pub.publish(m)

                # B 버튼: VR 원격 개입 모드
                elif name == 'B':
                    mode_state = "VR"
                    m = String()
                    m.data = mode_state
                    mode_pub.publish(m)

        # 조이스틱/축(ABS) 이벤트
        elif event.type == ecodes.EV_ABS:
            axis_name = axes.get(event.code, f'Unknown({event.code})')
            value = event.value
            # print(f"[ABS] Axis   {axis_name:14s} (code={event.code}) value={value}")

            # 현재 축 상태 갱신
            if event.code in axis_state:
                axis_state[event.code] = value

            # Left stick → /cmd_vel_joy, Right stick → /cmd_vel_vr
            lx_raw = axis_state.get(0, 128)  # Left X
            ly_raw = axis_state.get(1, 128)  # Left Y
            rx_raw = axis_state.get(2, 128)  # Right X
            ry_raw = axis_state.get(5, 128)  # Right Y

            # 0~255 -> -1.0 ~ 1.0 으로 정규화 (중앙 128 기준)
            def norm(v):
                return (float(v) - 128.0) / 128.0

            norm_x = norm(lx_raw)  # 좌우 (Left)
            norm_y = norm(ly_raw)  # 앞뒤 (Left)
            norm_x_r = norm(rx_raw)  # 좌우 (Right)
            norm_y_r = norm(ry_raw)  # 앞뒤 (Right)

            # 왼쪽 스틱 → 조이스틱 주행용 cmd_vel_joy
            linear_x_joy = -norm_y * MAX_LINEAR_VEL
            angular_z_joy = -norm_x * MAX_ANGULAR_VEL

            twist_joy = Twist()
            twist_joy.linear.x = float(linear_x_joy)
            twist_joy.angular.z = float(angular_z_joy)
            joy_pub.publish(twist_joy)

            # 오른쪽 스틱 → VR 원격 조작용 cmd_vel_vr
            linear_x_vr = -norm_y_r * MAX_LINEAR_VEL
            angular_z_vr = -norm_x_r * MAX_ANGULAR_VEL

            twist_vr = Twist()
            twist_vr.linear.x = float(linear_x_vr)
            twist_vr.angular.z = float(angular_z_vr)
            vr_pub.publish(twist_vr)


# except KeyboardInterrupt:
#     # print("\n종료합니다.")
finally:
    node.destroy_node()
    rclpy.shutdown()
