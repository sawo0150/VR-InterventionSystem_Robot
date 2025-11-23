import cv2
import time

DEVICE = "/dev/video0"   # Insta360 X3가 붙어 있는 장치
TARGET_RESOLUTIONS = [
    (1920, 1080),   # 우선 1080p 시도
    (1280, 720),    # 안 되면 720p 시도
    (640, 360),     # 최종 fallback
]

print("Insta360 X3 웹캠 테스트 (Jetson)")
print(f"장치: {DEVICE}")
print("-" * 40)

# 1. V4L2 백엔드로 카메라 열기
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
if not cap.isOpened():
    print("[에러] 카메라를 열 수 없습니다.")
    exit(1)

# 2. 포맷을 MJPG로 설정 (고해상도 + 프레임레이트를 위해 중요)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
cap.set(cv2.CAP_PROP_FOURCC, fourcc)

# 3. FPS도 30으로 요청
cap.set(cv2.CAP_PROP_FPS, 30)

# 4. 지원 가능한 최고 해상도부터 순서대로 시도
selected_w, selected_h = None, None
for (w, h) in TARGET_RESOLUTIONS:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    time.sleep(0.1)  # 드라이버가 적용할 시간 약간 대기

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"요청: {w}x{h}  ->  실제 적용: {real_w}x{real_h}")

    if real_w == w and real_h == h:
        selected_w, selected_h = real_w, real_h
        print(f"✅ {real_w}x{real_h} 해상도로 사용합니다.")
        break

if selected_w is None:
    print("⚠ 요청한 해상도가 그대로 적용되진 않았습니다. "
          "드라이버가 허용하는 값 중 가장 가까운 해상도를 사용합니다.")
    selected_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    selected_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fps = cap.get(cv2.CAP_PROP_FPS)
print(f"최종 해상도: {selected_w}x{selected_h}, FPS: {fps}")
print("-" * 40)

window_name = f"Insta360 X3 ({selected_w}x{selected_h}) - Press 'q' to quit"

frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("[에러] 프레임을 읽어오지 못했습니다.")
        break

    frame_count += 1

    text = f"{selected_w}x{selected_h} @ {fps:.1f} FPS (q to quit)"
    cv2.putText(frame, text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow(window_name, frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

elapsed = time.time() - start_time
if elapsed > 0:
    print(f"실측 FPS: {frame_count / elapsed:.2f}")

cap.release()
cv2.destroyAllWindows()
