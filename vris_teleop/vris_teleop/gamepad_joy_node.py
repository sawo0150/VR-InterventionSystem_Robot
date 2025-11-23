#!/usr/bin/env python3

# gamepad_test.py (ROS2 /cmd_vel_robot 퍼블리셔 버전)

import evdev
from evdev import InputDevice, ecodes
import time

import rclpy
from geometry_msgs.msg import Twist

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

# 2) 사용할 gamepad 경로 입력
# device_path = input("사용할 gamepad 경로를 입력하세요 (예: /dev/input/event17): ").strip()
gamepad = select_gamepad_device()
# print(f"\nUsing device: {gamepad.path} ({gamepad.name})\n")

# 2-1) ROS2 초기화 및 퍼블리셔 생성
rclpy.init()
node = rclpy.create_node('gamepad_teleop_simple')
cmd_pub = node.create_publisher(Twist, '/cmd_vel_robot', 10)

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
MAX_ANGULAR_VEL = 1.0  # rad/s

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

            # print(f"[KEY] Button {name:14s} (code={event.code}) {state}")

        # 조이스틱/축(ABS) 이벤트
        elif event.type == ecodes.EV_ABS:
            axis_name = axes.get(event.code, f'Unknown({event.code})')
            value = event.value
            # print(f"[ABS] Axis   {axis_name:14s} (code={event.code}) value={value}")

            # 현재 축 상태 갱신
            if event.code in axis_state:
                axis_state[event.code] = value

            # Left stick 기준으로 /cmd_vel_robot 계산
            lx_raw = axis_state.get(0, 128)  # Left X
            ly_raw = axis_state.get(1, 128)  # Left Y
            # 0~255 -> -1.0 ~ 1.0 으로 정규화 (중앙 128 기준)
            def norm(v):
                return (float(v) - 128.0) / 128.0

            norm_x = norm(lx_raw)  # 좌우
            norm_y = norm(ly_raw)  # 앞뒤

            # 일반적으로 조이스틱은 위로 밀면 값이 감소(-1)라서 부호 반전
            linear_x = -norm_y * MAX_LINEAR_VEL
            angular_z = -norm_x * MAX_ANGULAR_VEL

            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)

            cmd_pub.publish(twist)

# except KeyboardInterrupt:
#     # print("\n종료합니다.")
finally:
    node.destroy_node()
    rclpy.shutdown()
