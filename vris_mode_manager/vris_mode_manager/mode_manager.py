#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

import pygame
import sys
import math


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__('vris_mode_manager')

        # 상태 토픽 구독
        self.create_subscription(Twist, '/cmd_vel_joy', self.joy_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_vr', self.vr_callback, 10)
        self.create_subscription(Bool, '/system_error', self.error_callback, 10)
        self.create_subscription(String, '/operation_mode', self.mode_callback, 10)

        # 최종 로봇 명령 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_robot', 10)

        # 내부 상태
        self.joy_cmd = Twist()
        self.vr_cmd = Twist()
        self.robot_cmd = Twist()

        self.error_state = False      # False: 정상, True: 오류
        self.mode_state = "AUTO"      # "AUTO" or "VR"

        self.get_logger().info("ModeManagerNode initialized")

    # --- 콜백들 ---

    def joy_callback(self, msg: Twist):
        self.joy_cmd = msg

    def vr_callback(self, msg: Twist):
        self.vr_cmd = msg

    def error_callback(self, msg: Bool):
        self.error_state = msg.data

    def mode_callback(self, msg: String):
        # 혹시 이상한 값이 들어와도 일단 그대로 저장
        self.mode_state = msg.data if msg.data else "AUTO"

    # --- 출력 Twist 계산 로직 ---

    def compute_cmd(self) -> Twist:
        """현재 상태에 따라 /cmd_vel_robot 로 보낼 Twist 계산."""
        out = Twist()

        if self.error_state:
            # 오류면 무조건 정지
            # VR 모드 → VR 입력 사용
            out.linear.x = self.vr_cmd.linear.x
            out.linear.y = self.vr_cmd.linear.y
            out.linear.z = self.vr_cmd.linear.z
            out.angular.x = self.vr_cmd.angular.x
            out.angular.y = self.vr_cmd.angular.y
            out.angular.z = self.vr_cmd.angular.z
            return out

        # 오류가 아니라면 모드에 따라 선택
        if self.mode_state == "VR":
            # VR 모드 → VR 입력 사용
            out.linear.x = self.vr_cmd.linear.x
            out.linear.y = self.vr_cmd.linear.y
            out.linear.z = self.vr_cmd.linear.z
            out.angular.x = self.vr_cmd.angular.x
            out.angular.y = self.vr_cmd.angular.y
            out.angular.z = self.vr_cmd.angular.z
        else:
            # 기본은 AUTO (조이스틱 모드) → joy 입력 사용
            out.linear.x = self.joy_cmd.linear.x
            out.linear.y = self.joy_cmd.linear.y
            out.linear.z = self.joy_cmd.linear.z
            out.angular.x = self.joy_cmd.angular.x
            out.angular.y = self.joy_cmd.angular.y
            out.angular.z = self.joy_cmd.angular.z

        return out


def draw_text(surface, font, text, pos, color):
    img = font.render(text, True, color)
    surface.blit(img, pos)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()

    # --- pygame 초기화 ---
    pygame.init()
    width, height = 800, 480
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("VRIS Mode Manager")
    clock = pygame.time.Clock()

    # 폰트 (한글까지 잘 나오면 좋지만, 없으면 기본 폰트 사용)
    font_big = pygame.font.SysFont(None, 48)
    font_mid = pygame.font.SysFont(None, 32)
    font_small = pygame.font.SysFont(None, 24)

    running = True

    try:
        while rclpy.ok() and running:
            # ROS 콜백 한 번 실행
            rclpy.spin_once(node, timeout_sec=0.01)

            # 현재 상태 기반으로 로봇 명령 계산 및 publish
            cmd = node.compute_cmd()
            node.robot_cmd = cmd
            node.cmd_pub.publish(cmd)

            # pygame 이벤트 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # 화면 그리기
            screen.fill((20, 20, 20))

            # 1) 상단 상태바: 시스템 상태 + 모드
            # 시스템 오류 상태
            if node.error_state:
                status_text = "SYSTEM STATUS: ERROR"
                status_color = (255, 80, 80)
            else:
                status_text = "SYSTEM STATUS: NORMAL"
                status_color = (80, 255, 80)

            draw_text(screen, font_big, status_text, (40, 20), status_color)

            # 모드 상태
            if node.mode_state == "VR":
                mode_text = "MODE: VR REMOTE CONTROL"
                mode_color = (80, 160, 255)
            else:
                mode_text = "MODE: AUTO (JOYSTICK)"
                mode_color = (255, 220, 80)

            draw_text(screen, font_mid, mode_text, (40, 80), mode_color)

            # 2) 중앙: 각 cmd_vel 값들
            # helper: 값 포맷
            def fmt_twist(t: Twist):
                return f"lin.x={t.linear.x:+.2f}, ang.z={t.angular.z:+.2f}"

            y_base = 150
            line_gap = 50

            draw_text(screen, font_mid, "[/cmd_vel_joy]", (40, y_base), (200, 200, 200))
            draw_text(
                screen, font_small,
                fmt_twist(node.joy_cmd),
                (60, y_base + 30),
                (220, 220, 220)
            )

            draw_text(screen, font_mid, "[/cmd_vel_vr]", (40, y_base + line_gap), (200, 200, 200))
            draw_text(
                screen, font_small,
                fmt_twist(node.vr_cmd),
                (60, y_base + line_gap + 30),
                (220, 220, 220)
            )

            draw_text(screen, font_mid, "[/cmd_vel_robot]", (40, y_base + 2 * line_gap), (255, 255, 0))
            draw_text(
                screen, font_small,
                fmt_twist(node.robot_cmd),
                (60, y_base + 2 * line_gap + 30),
                (255, 255, 0)
            )

            # 3) 오른쪽: 간단한 설명 / 도움말
            right_x = 430
            help_y = 150
            draw_text(screen, font_mid, "Buttons (from gamepad_joy_manager):", (right_x, help_y), (180, 180, 180))
            help_y += 30
            draw_text(screen, font_small, "X : Set ERROR", (right_x + 20, help_y), (200, 200, 200))
            help_y += 25
            draw_text(screen, font_small, "Y : Clear ERROR", (right_x + 20, help_y), (200, 200, 200))
            help_y += 25
            draw_text(screen, font_small, "A : MODE = AUTO", (right_x + 20, help_y), (200, 200, 200))
            help_y += 25
            draw_text(screen, font_small, "B : MODE = VR", (right_x + 20, help_y), (200, 200, 200))

            help_y += 40
            draw_text(screen, font_small, "AUTO  : /cmd_vel_robot = /cmd_vel_joy", (right_x + 20, help_y), (180, 255, 180))
            help_y += 25
            draw_text(screen, font_small, "VR    : /cmd_vel_robot = /cmd_vel_vr", (right_x + 20, help_y), (180, 255, 180))
            help_y += 25
            draw_text(screen, font_small, "ERROR : /cmd_vel_robot = /cmd_vel_vr", (right_x + 20, help_y), (255, 180, 180))

            pygame.display.flip()
            clock.tick(30)  # 30 FPS 정도면 충분

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down ModeManagerNode & pygame")
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()
        sys.exit(0)


if __name__ == '__main__':
    main()
