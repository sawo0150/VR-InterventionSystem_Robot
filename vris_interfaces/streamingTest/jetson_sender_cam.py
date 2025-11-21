import asyncio, json, logging, cv2
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aioice.candidate import Candidate as AioIceCandidate
from aiortc.rtcicetransport import candidate_from_aioice

# --- 시그널링 서버 주소 ---
# SIGNALING_SERVER_URL = "ws://192.168.0.29:8080"  # ◀◀◀ Unity와 동일한 주소 입력
SIGNALING_SERVER_URL = "ws://147.47.128.72:8080"  # ◀◀◀ Unity와 동일한 주소 입력
# SIGNALING_SERVER_URL = "ws://192.168.247.247:8080"  # ◀◀◀ 휴대폰 핫스팟
# SIGNALING_SERVER_URL = "ws://192.168.137.1:8080"  # ◀◀◀ 노트북 핫스팟

# --- 로깅 설정 ---
logging.basicConfig(level=logging.INFO)

# --- 로컬 미리보기 설정 ---
SHOW_LOCAL_PREVIEW = True   # 필요 없으면 False로 바꾸면 됨

# --- OpenCV 비디오 캡처 ---
# Jetson CSI 카메라 GStreamer 파이프라인 (CSI 카메라 사용 시)
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=1"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# 0번 USB 웹캠 사용 시:
cap = cv2.VideoCapture(0)

# CSI 카메라 사용 시:
# cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

class OpenCVVideoTrack(VideoStreamTrack):
    """
    OpenCV 프레임을 WebRTC 트랙으로 변환하는 클래스
    """
    def __init__(self):
        super().__init__()

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # 재귀 대신 루프 재시도로 스택 누적 방지
        while True:
            pts, time_base = await self.next_timestamp()
            ret, frame = cap.read()
            if ret:
                break
            logging.warning("웹캠 프레임 읽기 실패")
            await asyncio.sleep(0.01)
 
        # Jetson 로컬에서 실시간 미리보기
        if SHOW_LOCAL_PREVIEW:
            cv2.imshow("Jetson Local Preview", frame)
            # GUI 이벤트 처리를 위해 1ms 대기 (키 입력 무시는 0xFF & ...)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("로컬 미리보기 창 종료 요청(q).")
                # q 누르면 그냥 창만 닫고, 스트리밍은 계속 진행
                cv2.destroyWindow("Jetson Local Preview")
                # 필요하면 여기서 SHOW_LOCAL_PREVIEW를 False로 꺼도 됨

        # aiortc가 요구하는 VideoFrame 형식으로 변환
        from av import VideoFrame
        frame = VideoFrame.from_ndarray(frame, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        
        return frame

# Unity에서 온 ICE JSON을 aiortc.RTCIceCandidate로 변환하는 헬퍼
def to_aiortc_candidate(msg):
    """
    msg: {
      "candidate": "candidate:...",
      "sdpMid": "0",
      "sdpMLineIndex": 0
    }
    """
    cand_str = msg.get("candidate", "") or ""

    # "candidate:" 프리픽스 제거
    prefix = "candidate:"
    if cand_str.startswith(prefix):
        cand_str = cand_str[len(prefix):]

    # 1) SDP 문자열 -> aioice.Candidate
    aio_cand = AioIceCandidate.from_sdp(cand_str)

    # 2) aioice.Candidate -> aiortc.RTCIceCandidate
    rtc_cand = candidate_from_aioice(aio_cand)

    # 3) sdpMid / sdpMLineIndex 설정
    rtc_cand.sdpMid = msg.get("sdpMid")
    rtc_cand.sdpMLineIndex = msg.get("sdpMLineIndex")

    return rtc_cand

async def run():
    logging.info(f"시그널링 서버에 접속 시도: {SIGNALING_SERVER_URL}")
    async with websockets.connect(SIGNALING_SERVER_URL) as ws:
        logging.info("시그널링 서버 접속 성공.")
        # 역할 등록 (필수)
        await ws.send(json.dumps({"role": "sender"}))
        
        pc = RTCPeerConnection()

        # 카메라 오픈 확인
        if not cap.isOpened():
            logging.error("카메라를 열 수 없습니다. 장치 인덱스/권한/파이프라인을 확인하세요.")
            return

        # (A) ICE 후보가 생성되면 WebSocket으로 Unity에 전송
        @pc.on("icecandidate")
        async def on_icecandidate(candidate):
            if candidate:
                logging.info(f"[WebRTC] Jetson ICE 후보 생성됨. Unity로 전송.")
                await ws.send(json.dumps({
                    'type': 'candidate',
                    'candidate': candidate.candidate,
                    'sdpMid': candidate.sdpMid,
                    'sdpMLineIndex': candidate.sdpMLineIndex,
                }))

        # (B) 웹캠 트랙을 PeerConnection에 추가
        pc.addTrack(OpenCVVideoTrack())

        # (C) Offer(연결 제안) 생성 및 전송
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        
        logging.info("[WebRTC] 'offer' 생성 완료. Unity로 전송.")
        await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp}))

        # (D) WebSocket 메시지 수신 (Unity의 Answer/Candidate)
        async for message in ws:
            data = json.loads(message)

            if data["type"] == "answer":
                logging.info("[WebRTC] 'answer' 수신.")
                answer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(answer)
                logging.info("[WebRTC] SetRemoteDescription(answer) 성공. 스트리밍 시작됨.")

            elif data["type"] == "candidate":
                
                logging.info("[WebRTC] 'candidate' 수신.")

            #     # Unity JSON -> aiortc용 ICE 후보로 변환
            #     candidate = to_aiortc_candidate({
            #         "candidate": data.get("candidate"),
            #         "sdpMid": data.get("sdpMid"),
            #         "sdpMLineIndex": data.get("sdpMLineIndex"),
            #     })

            # elif data["type"] == "control":

                # Unity JSON -> aiortc용 ICE 후보로 변환
                candidate = to_aiortc_candidate({
                    "candidate": data.get("candidate"),
                    "sdpMid": data.get("sdpMid"),
                    "sdpMLineIndex": data.get("sdpMLineIndex"),
                })

                await pc.addIceCandidate(candidate)
                logging.info("[WebRTC] Unity ICE 후보 추가 완료.")

            elif data["type"] == "control":
                # Unity에서 온 로봇 제어 명령 (데모용: 일단 로그만)
                linear = data.get("linear", 0.0)
                angular = data.get("angular", 0.0)
                logging.info(f"[CONTROL] linear={linear:.2f}, angular={angular:.2f}")
                # TODO: 여기에서 실제 로봇 제어 코드 호출 (예: ROS publish 등)

            elif data["type"] == "offer":
                # 발신자는 offer를 받을 일이 없음
                pass

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logging.info("스크립트 종료.")
    except websockets.exceptions.ConnectionClosedError as e:
        logging.error(f"시그널링 서버 연결 실패 또는 끊김: {e}")
    finally:
        cap.release()
        logging.info("웹캠 리소스 해제.")