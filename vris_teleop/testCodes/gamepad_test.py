# gamepad_test.py
import evdev
from evdev import InputDevice, ecodes
import time

# 1) 연결된 input 디바이스 목록 출력
devices = [InputDevice(path) for path in evdev.list_devices()]
print("=== Devices ===")
for d in devices:
    print(d.path, d.name, d.phys)

# 2) 사용할 gamepad 경로 입력
device_path = input("사용할 gamepad 경로를 입력하세요 (예: /dev/input/event17): ").strip()
gamepad = InputDevice(device_path)
print(f"\nUsing device: {gamepad.path} ({gamepad.name})\n")

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

print("버튼/조이스틱 입력 대기 중... (Ctrl+C 로 종료)\n")

try:
    while True:
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

            print(f"[KEY] Button {name:14s} (code={event.code}) {state}")

        # 조이스틱/축(ABS) 이벤트
        elif event.type == ecodes.EV_ABS:
            axis_name = axes.get(event.code, f'Unknown({event.code})')
            value = event.value
            print(f"[ABS] Axis   {axis_name:14s} (code={event.code}) value={value}")

except KeyboardInterrupt:
    print("\n종료합니다.")