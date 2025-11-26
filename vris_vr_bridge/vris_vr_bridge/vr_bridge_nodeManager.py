import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import asyncio
import json
import logging
import cv2
import numpy as np  # [중요] 누락된 numpy 추가
import websockets
import time
import base64
import contextlib
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from aiortc.contrib.media import MediaBlackhole
from aioice.candidate import Candidate as AioIceCandidate  # [중요] candidate 변환용
from aiortc.rtcicetransport import candidate_from_aioice   # [중요] candidate 변환용
from av import VideoFrame

# [중요] Unity ICE JSON -> aiortc 변환 헬퍼 함수 추가
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

# --- 기존 Legacy 코드의 OpenCVVideoTrack 클래스 유지 ---
class OpenCVVideoTrack(VideoStreamTrack):
    def __init__(self, capture_index=0, width=640, height=360):
        super().__init__()
        self.frameCount = 0
        # [수정] 노트북 환경 호환성을 위해 CAP_V4L2 강제 사용
        logging.info(f"[Camera] Opening Camera Index {capture_index} with {width}x{height}")
        self.cap = cv2.VideoCapture(capture_index, cv2.CAP_V4L2)

        # 해상도 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # 카메라 오픈 확인
        if not self.cap.isOpened():
            logging.error(f"[Camera] Failed to open camera index {capture_index}")
        else:
            real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            logging.info(f"[Camera] Initialized. Actual Resolution: {real_w}x{real_h}")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret:
            # 읽기 실패 시 로그 출력 및 검은 화면
            logging.warning("[Camera] Frame read failed!")
            await asyncio.sleep(0.1)
            # 해상도는 고정이 아니므로 기본값으로 처리하거나 self 멤버변수로 관리 필요하나, 
            # 여기서는 안전하게 640x360 블랙으로 처리
            black_frame = np.zeros((360, 640, 3), dtype=np.uint8)
            frame = VideoFrame.from_ndarray(black_frame, format="bgr24")
            frame.pts = pts
            frame.time_base = time_base
            return frame
        
        self.frameCount += 1
        if self.frameCount % 300 == 1:
             logging.info(f"[Camera] Streaming... Frame {self.frameCount}")

        # aiortc용 프레임 변환
        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

    def release(self):
        if self.cap.isOpened():
            self.cap.release()

class VRBridgeNode(Node):
    def __init__(self):
        super().__init__('vr_bridge_node')
        
        # 1. 파라미터 설정 (IP 주소 등을 런치 파일에서 변경 가능하게)
        self.declare_parameter('signaling_ip', '192.168.0.7')
        self.declare_parameter('signaling_port', 8080)
                
        # [추가] 카메라 관련 파라미터
        self.declare_parameter('camera_type', 'generic')   # generic or insta360
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('resolution', '640x360')    # 640x360, 1280x720, 1920x1080

        self.signaling_ip = self.get_parameter('signaling_ip').value
        self.signaling_port = self.get_parameter('signaling_port').value
        
        # 파라미터 읽기
        self.cam_type = self.get_parameter('camera_type').value
        self.cam_idx = self.get_parameter('camera_index').value
        self.res_str = self.get_parameter('resolution').value

        self.uri = f"ws://{self.signaling_ip}:{self.signaling_port}"

        # 2. Publisher 설정 (/cmd_vel_vr, /operation_mode)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_vr', 10)
        self.mode_publisher_ = self.create_publisher(String, '/operation_mode', 10)

        # 3. 상태/이미지 구독 (/system_error, /operation_mode, /vr_localization_image)
        self.error_state = False
        self.mode_state = "AUTO"
        self.bridge = CvBridge()
        self.localization_jpeg = None   # 최신 로컬라이제이션 이미지를 JPEG로 인코딩한 바이트

        self.create_subscription(Bool, '/system_error', self.system_error_callback, 10)
        self.create_subscription(String, '/operation_mode', self.mode_state_callback, 10)
        self.create_subscription(Image, '/vr_localization_image', self.localization_image_callback, 10)

        self.get_logger().info(f"VR Bridge Node Started. Connecting to {self.uri}")

        # 4. WebRTC용 별도 스레드 시작
        self.webrtc_thread = threading.Thread(target=self.start_asyncio_loop)
        self.webrtc_thread.daemon = True
        self.webrtc_thread.start()

    # ---- ROS 콜백들 (상태/이미지) ----

    def system_error_callback(self, msg: Bool):
        self.error_state = msg.data

    def mode_state_callback(self, msg: String):
        if msg.data:
            self.mode_state = msg.data

    def localization_image_callback(self, msg: Image):
        """map_visualizer_node가 퍼블리시한 /vr_localization_image를 JPEG로 인코딩해 저장."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, enc = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.localization_jpeg = enc.tobytes()
        except Exception as e:
            self.get_logger().warn(f"Failed to convert localization image: {e}")

    def start_asyncio_loop(self):
        # 별도 스레드에서 asyncio 이벤트 루프 생성 및 실행
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        self.loop = loop
        loop.run_until_complete(self.run_webrtc())

    def destroy_node(self):
        # 종료 시 asyncio 루프 중단 요청
        try:
            if hasattr(self, "loop") and self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
        except Exception as e:
            self.get_logger().warn(f"Error stopping asyncio loop: {e}")
        super().destroy_node()

    def parse_resolution(self, res_str):
        try:
            w, h = map(int, res_str.split('x'))
            return w, h
        except:
            self.get_logger().warn(f"Invalid resolution string: {res_str}. Using 640x360.")
            return 640, 360


    async def run_webrtc(self):
        # 해상도 파싱
        target_w, target_h = self.parse_resolution(self.res_str)
        pc = None
        video_track = None
        status_task = None
        map_task = None

        try:
            async with websockets.connect(self.uri) as ws:
                self.get_logger().info("Connected to Signaling Server")
                await ws.send(json.dumps({"role": "sender"}))

                # STUN 서버 설정
                config = RTCConfiguration(iceServers=[
                    RTCIceServer(urls="stun:stun.l.google.com:19302")
                ])
                pc = RTCPeerConnection(configuration=config)

                @pc.on("iceconnectionstatechange")
                async def on_iceconnectionstatechange():
                    self.get_logger().info(f"ICE State: {pc.iceConnectionState.upper()}")

                # 카메라 트랙 생성 (기존 Video 스트림 유지)
                video_track = OpenCVVideoTrack(
                    capture_index=self.cam_idx,
                    width=target_w,
                    height=target_h
                )
                pc.addTrack(video_track)

                @pc.on("icecandidate")
                async def on_icecandidate(candidate):
                    if candidate:
                        msg = {
                            'type': 'candidate',
                            'candidate': candidate.candidate,
                            'sdpMid': candidate.sdpMid,
                            'sdpMLineIndex': candidate.sdpMLineIndex
                        }
                        await ws.send(json.dumps(msg))

                # Offer 생성 및 전송
                offer = await pc.createOffer()
                await pc.setLocalDescription(offer)
                await ws.send(json.dumps({"type": "offer", "sdp": offer.sdp}))

                # ---- 로봇 → VR 상태/맵 전송용 태스크들 ----

                async def status_sender():
                    """현재 오류 상태 / 모드 상태를 VR에 주기적으로 전송."""
                    last_err = None
                    last_mode = None
                    try:
                        while True:
                            if last_err != self.error_state or last_mode != self.mode_state:
                                msg = {
                                    "type": "status",
                                    "error": self.error_state,
                                    "mode": self.mode_state,
                                }
                                await ws.send(json.dumps(msg))
                                last_err = self.error_state
                                last_mode = self.mode_state
                            await asyncio.sleep(0.2)
                    except Exception as e:
                        self.get_logger().warn(f"status_sender stopped: {e}")

                async def map_sender():
                    """map_visualizer_node에서 만든 로컬라이제이션 이미지를 주기적으로 전송."""
                    try:
                        while True:
                            if self.localization_jpeg is not None:
                                b64 = base64.b64encode(self.localization_jpeg).decode('ascii')
                                msg = {
                                    "type": "localization_image",
                                    "format": "jpeg",
                                    "data": b64,
                                }
                                await ws.send(json.dumps(msg))
                            await asyncio.sleep(1.0)  # 1 Hz 정도면 충분
                    except Exception as e:
                        self.get_logger().warn(f"map_sender stopped: {e}")

                status_task = asyncio.create_task(status_sender())
                map_task = asyncio.create_task(map_sender())

                # 메시지 수신 루프 (VR → 로봇)
                async for message in ws:
                    data = json.loads(message)

                    if data["type"] == "answer":
                        answer = RTCSessionDescription(sdp=data["sdp"], type="answer")
                        await pc.setRemoteDescription(answer)
                        self.get_logger().info("WebRTC Connection Established!")

                    elif data["type"] == "candidate":
                        candidate = to_aiortc_candidate(data)
                        if candidate:
                            await pc.addIceCandidate(candidate)
                            self.get_logger().info("Added Remote ICE Candidate")

                    elif data["type"] == "control":
                        # VR에서 온 속도 명령 → /cmd_vel_vr
                        linear_x = float(data.get("linear", 0.0))
                        angular_z = float(data.get("angular", 0.0))
                        twist = Twist()
                        twist.linear.x = linear_x
                        twist.angular.z = angular_z
                        self.publisher_.publish(twist)

                    elif data["type"] == "mode":
                        # VR에서 모드 변경 요청 → /operation_mode
                        mode = data.get("mode", "")
                        if mode in ("AUTO", "VR"):
                            msg = String()
                            msg.data = mode
                            self.mode_publisher_.publish(msg)
                            # 로컬 상태도 업데이트 (status_sender가 반영)
                            self.mode_state = mode

        except Exception as e:
            self.get_logger().error(f"WebRTC Error: {e}")
        finally:
            # 보조 태스크 정리
            for t in (status_task, map_task):
                if t is not None:
                    t.cancel()
                    with contextlib.suppress(Exception):
                        await t

            # ✅ 자원 정리 (중요)
            if pc is not None:
                await pc.close()
            if video_track is not None:
                video_track.release()
            self.get_logger().info("✅ WebRTC resources cleaned up.")


def main(args=None):
    rclpy.init(args=args)
    node = VRBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()