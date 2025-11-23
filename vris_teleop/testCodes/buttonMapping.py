import evdev
from evdev import InputDevice, ecodes
import time

# 1) 장치 목록 먼저 보여주기
devices = [InputDevice(path) for path in evdev.list_devices()]
print("=== Devices ===")
for d in devices:
    print(d.path, d.name, d.phys)

# 2) 여기에서 직접 골라 넣기
device_path = input("사용할 gamepad 경로를 입력하세요 (예: /dev/input/event4): ").strip()
gamepad = InputDevice(device_path)
print(f"Using device: {gamepad.path} {gamepad.name}")

# 3) 논리적인 버튼 이름 목록 (원래 쓰던 이름들)
logical_buttons = [
    'Select', 'Start', 'Mode',
    'L2', 'R2', 'L1', 'R1',
    'X', 'Y', 'A', 'B'
]

code_map = {}   # 버튼: event.code -> logical name

# 4) 논리적인 조이스틱 축 이름 목록
#   필요에 따라 이름은 바꿔도 됨 (순서는 "어떤 축을 먼저 물어볼지" 정도 의미)
logical_axes = [
    'Left Joystick X',
    'Left Joystick Y',
    'Right Joystick X',
    'Right Joystick Y',
    'D-pad X',
    'D-pad Y',
]

axis_map = {}   # 축: event.code -> logical axis name
 

print("\n이제 안내에 따라 각 버튼을 한 번씩 눌러주세요.\n")

for name in logical_buttons:
    print(f"--> 지금 '{name}' 버튼을 눌러주세요 (누를 때까지 대기합니다)...")
    while True:
        event = gamepad.read_one()
        if event is None:
            continue
        if event.type == ecodes.EV_KEY and event.value == 1:  # 눌림(press)만 기록
            print(f"   감지: code={event.code}")
            code_map[event.code] = name
            break


print("\n이제 안내에 따라 각 조이스릭 축을 천천히 끝까지 움직여주세요.\n")

for name in logical_axes:
    print(f"--> 지금 '{name}' 축을 크게 한 방향으로 끝까지 움직였다가 천천히 돌려주세요 "
          f"(이 축의 EV_ABS 이벤트를 몇 초 동안 기록합니다)...")

    # 코드별 발생 횟수 카운트
    counts = {}
    start_time = time.time()

    # 약 3초 동안 EV_ABS 이벤트를 모읍니다.
    while time.time() - start_time < 3.0:
        event = gamepad.read_one()
        if event is None:
            time.sleep(0.001)
            continue

        if event.type == ecodes.EV_ABS:
            counts[event.code] = counts.get(event.code, 0) + 1
            # 필요하면 디버그용으로 풀어서 확인 가능
            # print(f"   감지: code={event.code}, value={event.value}")

    if not counts:
        print("   EV_ABS 이벤트를 감지하지 못했습니다. 이 축은 건너뜁니다.")
        continue

    # 가장 많이 등장한 code를 이 논리 축에 매핑
    best_code = max(counts, key=counts.get)

    if best_code in axis_map:
        print(f"   주의: code={best_code}는 이미 '{axis_map[best_code]}'에 매핑되어 있습니다.")

    axis_map[best_code] = name
    print(f"   최종 선택: code={best_code} (count={counts[best_code]}) -> '{name}'")

print("\n=== 완성된 매핑 (code -> 이름) ===")
print("buttons = {")
for code, name in code_map.items():
    print(f"    {code}: '{name}',")
print("}")

print("\naxes = {")
for code, name in axis_map.items():
    print(f"    {code}: '{name}',")
print("}")