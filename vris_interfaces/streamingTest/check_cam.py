import cv2
import time

print("사용 가능한 웹캠을 확인하고 실시간 피드를 엽니다...")
print("각 카메라의 창이 뜨면 'q' 키를 눌러 닫고 다음 카메라를 확인하세요.")
print("-" * 30)

available_indices = []

for i in range(10):  # 0번부터 9번 인덱스까지 확인
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        # 열리지 않으면 조용히 넘어감
        continue

    # 카메라가 열렸다면 정보 출력
    print(f"\n[ 📷 웹캠 인덱스 {i} 발견 ]")
    available_indices.append(i)

    # 1. 기본 정보 (화질, FPS) 가져오기
    # .get() 메서드는 종종 float 값을 반환하므로 int로 변환
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"  - 해상도 (화질): {width} x {height}")
    print(f"  - 설정된 FPS: {fps:.2f}") # .2f는 소수점 둘째 자리까지 표시
    
    # 2. 실시간 작동 확인 (창 띄우기)
    window_name = f"Webcam Index {i} (Press 'q' to close)"
    print(f"  -> 실시간 피드를 엽니다. ('{window_name}' 창)")
    
    # 프레임 계산용 변수 (실측 FPS 확인)
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print(f"  ! 인덱스 {i}: 프레임을 읽어오는 데 실패했습니다.")
            break
            
        frame_count += 1
        
        # 실시간 화면에 텍스트 오버레이 (옵션)
        text = f"Index: {i} | {width}x{height} | Press 'q' to quit"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow(window_name, frame)
        
        # 1ms 대기하며 'q' 키 입력 확인
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"  -> 인덱스 {i} 창을 닫습니다.")
            
            # 실측 FPS 계산
            end_time = time.time()
            elapsed = end_time - start_time
            if elapsed > 0:
                measured_fps = frame_count / elapsed
                print(f"  - 실측 FPS: {measured_fps:.2f} (약 {elapsed:.1f}초 동안)")
            
            break
    
    # 해당 카메라 자원 해제 및 창 닫기
    cap.release()
    cv2.destroyWindow(window_name)

print("-" * 30)
if not available_indices:
    print("\n[ 결과 ] 연결된 웹캠을 찾을 수 없습니다. (0-9 인덱스 확인 완료)")
else:
    print(f"\n[ 결과 ] 확인된 웹캠 인덱스: {available_indices}")

# 혹시 모를 모든 창 닫기
cv2.destroyAllWindows()