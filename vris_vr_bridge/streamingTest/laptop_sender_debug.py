import asyncio
import json
import logging
import cv2
import numpy as np
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceServer, RTCConfiguration
from aioice.candidate import Candidate as AioIceCandidate
from aiortc.rtcicetransport import candidate_from_aioice
from av import VideoFrame

# ==========================================
# [설정 영역]
# ==========================================
# Unity PC의 IP 주소가 정확한지 다시 한번 확인하세요!
SIGNALING_SERVER_URL = "ws://192.168.0.7:8080" 

# 디버그 레벨 설정 (INFO -> DEBUG로 바꾸면 패킷 단위 로그가 쏟아짐)
# 문제 해결을 위해 aioice(네트워크) 로그를 활성화합니다.
logging.basicConfig(level=logging.INFO) 
logging.getLogger("aioice").setLevel(logging.INFO) # 너무 많으면 WARNING으로 변경
logging.getLogger("aiortc").setLevel(logging.INFO)

SHOW_LOCAL_PREVIEW = True

# ==========================================
# [카메라 설정]
# ==========================================
# V4L2 강제 사용
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

class OpenCVVideoTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.frameCount = 0
        logging.info("[Track] 비디오 트랙 초기화됨.")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        # 카메라 읽기
        ret, frame = cap.read()
        
        if not ret:
            logging.warning("[Camera] 프레임 읽기 실패!")
            await asyncio.sleep(0.1)
            # 검은 화면 전송
            return VideoFrame.from_ndarray(np.zeros((480, 640, 3), dtype=np.uint8), format="bgr24")

        self.frameCount += 1
        # 로그가 너무 자주 뜨지 않게 60프레임마다 한 번만 출력
        if self.frameCount % 60 == 1:
            logging.info(f"[Camera] 스트리밍 중... Frame: {self.frameCount}")

        # Jetson 로컬 미리보기
        if SHOW_LOCAL_PREVIEW:
            cv2.imshow("Jetson Sender Preview", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass 

        # aiortc용 프레임 변환
        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

def to_aiortc_candidate(msg):
    cand_str = msg.get("candidate", "") or ""
    prefix = "candidate:"
    if cand_str.startswith(prefix):
        cand_str = cand_str[len(prefix):]
    try:
        aio_cand = AioIceCandidate.from_sdp(cand_str)
        rtc_cand = candidate_from_aioice(aio_cand)
        rtc_cand.sdpMid = msg.get("sdpMid")
        rtc_cand.sdpMLineIndex = msg.get("sdpMLineIndex")
        return rtc_cand
    except Exception as e:
        logging.error(f"Candidate 파싱 에러: {e}")
        return None

async def run():
    # 1. 시작 전 카메라 테스트 (WebRTC 연결 전에도 창이 뜨는지 확인)
    if not cap.isOpened():
        logging.error("❌ 카메라를 열 수 없습니다! (Index 0, V4L2)")
        return
    else:
        logging.info("✅ 카메라 열기 성공. 초기 프레임 테스트 중...")
        ret, frame = cap.read()
        if ret:
            logging.info(f"✅ 초기 프레임 획득 성공 ({frame.shape}). WebRTC 연결을 시작합니다.")
        else:
            logging.error("❌ 초기 프레임 획득 실패.")
            return

    logging.info(f"🌐 시그널링 서버 접속 시도: {SIGNALING_SERVER_URL}")
    
    try:
        async with websockets.connect(SIGNALING_SERVER_URL) as ws:
            logging.info("✅ 시그널링 서버 접속 성공.")
            await ws.send(json.dumps({"role": "sender"}))
            
            # STUN 서버 추가 (네트워크 연결 성공률 비약적 상승)
            config = RTCConfiguration(iceServers=[
                RTCIceServer(urls="stun:stun.l.google.com:19302")
            ])
            pc = RTCPeerConnection(configuration=config)

            @pc.on("iceconnectionstatechange")
            async def on_iceconnectionstatechange():
                logging.info(f"🔄 ICE 상태 변경: {pc.iceConnectionState.upper()}")
                if pc.iceConnectionState == "connected":
                    logging.info("🎉 P2P 연결 성공! 비디오 전송이 시작됩니다.")
                elif pc.iceConnectionState == "failed":
                    logging.error("⛔ P2P 연결 실패. 방화벽(UDP 1024-65535)을 확인하세요.")

            pc.addTrack(OpenCVVideoTrack())

            @pc.on("icecandidate")
            async def on_icecandidate(candidate):
                if candidate:
                    await ws.send(json.dumps({
                        'type': 'candidate',
                        'candidate': candidate.candidate,
                        'sdpMid': candidate.sdpMid,
                        'sdpMLineIndex': candidate.sdpMLineIndex,
                    }))

            offer = await pc.createOffer()
            await pc.setLocalDescription(offer)
            logging.info("📤 Offer 전송함.")
            await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp}))

            async for message in ws:
                data = json.loads(message)

                if data["type"] == "answer":
                    logging.info("📥 Answer 수신함.")
                    answer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                    await pc.setRemoteDescription(answer)

                elif data["type"] == "candidate":
                    logging.info("📥 Unity Candidate 수신.")
                    candidate = to_aiortc_candidate(data)
                    if candidate:
                        await pc.addIceCandidate(candidate)
                    
                elif data["type"] == "control":
                    pass

    except Exception as e:
        logging.error(f"치명적 에러 발생: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        logging.info("프로그램 종료.")