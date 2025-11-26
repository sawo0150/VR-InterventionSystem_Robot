#!/usr/bin/env python3
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf2_ros


class MapVisualizerNode(Node):
    def __init__(self):
        super().__init__('map_visualizer_node')

        # === 파라미터 ===
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('image_topic', '/vr_localization_image')
        self.declare_parameter('publish_rate', 2.0)  # Hz

        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # === Publisher / Subscriber ===
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            self.image_topic,
            10
        )

        # === TF ===
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # === 기타 상태 ===
        self.bridge = CvBridge()
        self.latest_map = None          # OccupancyGrid
        self.latest_map_image = None    # OpenCV BGR 이미지 (배경만)
        self.map_info = None            # (resolution, origin_x, origin_y, width, height)

        # 타이머: 주기적으로 이미지 생성 & 퍼블리시
        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"[MapVisualizerNode] Subscribing map: {self.map_topic}, "
            f"frames: {self.global_frame}->{self.base_frame}, "
            f"publishing image: {self.image_topic} @ {self.publish_rate} Hz"
        )

    # -----------------------
    # Map 콜백: OccupancyGrid → 기본 배경 이미지로 변환
    # -----------------------
    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.map_info = (res, origin_x, origin_y, width, height)

        # data: 1D array, row-major, (0,0) = left-bottom
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # occupancy 값 → 그레이스케일
        # -1: unknown → 회색(128)
        # 0: free     → 흰색(255)
        # 100: occ    → 검정(0)
        img = np.zeros((height, width), dtype=np.uint8)
        img[:] = 128  # unknown

        img[data == 0] = 255
        img[data > 50] = 0  # 50 이상은 장애물로 간주

        # OpenCV 이미지는 (0,0)이 좌상단, map은 (0,0)이 좌하단이므로 위아래 뒤집기
        img = np.flipud(img)

        # BGR로 변환 (나중에 컬러 overlay 위해)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        self.latest_map_image = img_bgr

    # -----------------------
    # 주기 타이머: TF 조회 + 로봇 위치 표시 + 이미지 퍼블리시
    # -----------------------
    def timer_callback(self):
        if self.latest_map is None or self.latest_map_image is None or self.map_info is None:
            # 아직 map 수신 전
            return

        # TF에서 로봇 pose 가져오기
        try:
            # 현재 시간 기준 transform (최근 값)
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            # TF 없을 때는 그냥 스킵
            self.get_logger().debug(f"TF lookup failed: {e}")
            return

        # pose 추출
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        q = transform.transform.rotation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        res, origin_x, origin_y, width, height = self.map_info

        # 맵 좌표 → 픽셀 좌표로 변환
        # map_x, map_y: 맵 셀 좌표 (0 ~ width-1, 0 ~ height-1), 원점은 좌하단
        map_x = (tx - origin_x) / res
        map_y = (ty - origin_y) / res

        # 이미지 좌표: 좌상단이 (0,0), y는 아래로 증가
        px = int(map_x)
        py = int(height - 1 - map_y)  # flip 때문에

        # 이미지 범위 체크
        if px < 0 or px >= width or py < 0 or py >= height:
            # 맵 밖이면 그냥 로봇 표시 없이 배경만 퍼블리시
            vis_img = self.latest_map_image.copy()
        else:
            vis_img = self.latest_map_image.copy()

            # 로봇 위치 원 (파란색)
            radius = max(2, int(0.15 / res))  # 15cm 정도를 픽셀로 환산
            cv2.circle(vis_img, (px, py), radius, (255, 0, 0), thickness=2)

            # 로봇 진행 방향 화살표 (빨간색)
            arrow_len = radius * 2
            end_x = int(px + arrow_len * math.cos(yaw+math.pi))
            end_y = int(py - arrow_len * math.sin(yaw+math.pi))  # y축 반대

            cv2.arrowedLine(
                vis_img,
                (px, py),
                (end_x, end_y),
                (0, 0, 255),
                thickness=2,
                tipLength=0.4
            )

        # ROS Image로 변환 후 publish
        img_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.global_frame

        self.image_pub.publish(img_msg)

    # -----------------------
    # Helper: Quaternion → Yaw
    # -----------------------
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """ZYX 순서 yaw만 추출."""
        # 참고: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
