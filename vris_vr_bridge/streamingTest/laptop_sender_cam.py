import asyncio, json, logging, cv2
import numpy as np  # ◀◀◀ (중요) 누락된 numpy 라이브러리 추가
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aioice.candidate import Candidate as AioIceCandidate
from aiortc.rtcicetransport import candidate_from_aioice
from av import VideoFrame

# --- 시그널링 서버 주소 ---
# SIGNALING_SERVER_URL = "ws://192.168.0.29:8080"
SIGNALING_SERVER_URL = "ws://192.168.0.7:8080"  # ◀◀◀ Unity와 동일한 IP 확인 필수!

# --- 로깅 설정 ---
logging.basicConfig(level=logging.INFO)

# --- 로컬 미리보기 설정 ---
SHOW_LOCAL_PREVIEW = True

# 0번 USB 웹캠 사용
# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2) # ◀◀◀ (중요) GStreamer 대신 V4L2 강제 사용 (호환성
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # 해상도를 낮춰서 대역폭 문제 배제
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

class OpenCVVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frameCount = 0

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # 카메라 읽기 시도
        ret, frame = cap.read()
        
        if not ret:
            logging.warning("웹캠 프레임 읽기 실패 (카메라 연결 확인 필요)")
            # 실패 시 검은 화면이라도 보냄 (멈춤 방지)
            await asyncio.sleep(0.1)
            return VideoFrame.from_ndarray(np.zeros((480, 640, 3), dtype=np.uint8), format="bgr24")

        self.frameCount += 1
        if self.frameCount % 60 == 0:
            logging.info(f"[Camera] {self.frameCount}번째 프레임 캡처 중... (크기: {frame.shape})")

        # Jetson 로컬에서 실시간 미리보기
        if SHOW_LOCAL_PREVIEW:
            cv2.imshow("Jetson Local Preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass 

        # aiortc용 프레임 변환
        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

# Unity ICE JSON -> aiortc 변환 헬퍼
def to_aiortc_candidate(msg):
    cand_str = msg.get("candidate", "") or ""
    prefix = "candidate:"
    if cand_str.startswith(prefix):
        cand_str = cand_str[len(prefix):]
    aio_cand = AioIceCandidate.from_sdp(cand_str)
    rtc_cand = candidate_from_aioice(aio_cand)
    rtc_cand.sdpMid = msg.get("sdpMid")
    rtc_cand.sdpMLineIndex = msg.get("sdpMLineIndex")
    return rtc_cand

async def run():
    logging.info(f"시그널링 서버에 접속 시도: {SIGNALING_SERVER_URL}")
    async with websockets.connect(SIGNALING_SERVER_URL) as ws:
        logging.info("시그널링 서버 접속 성공.")
        await ws.send(json.dumps({"role": "sender"}))
        
        pc = RTCPeerConnection()

        # (중요) 연결 상태 변화 로그 출력
        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logging.info(f"!!! ICE 연결 상태 변경: {pc.iceConnectionState} !!!")
            if pc.iceConnectionState == "failed":
                logging.error("ICE 연결 실패. 방화벽이나 네트워크 대역을 확인하세요.")

        if not cap.isOpened():
            logging.error("카메라를 열 수 없습니다.")
            return

        pc.addTrack(OpenCVVideoTrack())

        @pc.on("icecandidate")
        async def on_icecandidate(candidate):
            if candidate:
                # logging.info(f"ICE 후보 전송") # 로그 너무 많으면 주석 처리
                await ws.send(json.dumps({
                    'type': 'candidate',
                    'candidate': candidate.candidate,
                    'sdpMid': candidate.sdpMid,
                    'sdpMLineIndex': candidate.sdpMLineIndex,
                }))

        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        logging.info("[WebRTC] 'offer' 전송.")
        await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp}))

        async for message in ws:
            data = json.loads(message)

            if data["type"] == "answer":
                logging.info("[WebRTC] 'answer' 수신.")
                answer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(answer)

            elif data["type"] == "candidate":
                candidate = to_aiortc_candidate(data)
                await pc.addIceCandidate(candidate)
                logging.info("Unity ICE 후보 추가됨.")
                
            elif data["type"] == "control":
                pass

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()