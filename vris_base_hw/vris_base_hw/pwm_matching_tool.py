#!/usr/bin/env python3
import os
import time
import math
import yaml
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import pygame


class ImuCollector(Node):
    """IMU 데이터를 모으기 위한 ROS2 노드"""

    def __init__(self, imu_topic: str):
        super().__init__('pwm_matching_imu_collector')

        self.sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            50
        )

        # (timestamp, ang_vel_z) 리스트
        self.samples = []
        self.recording = False
        self.start_time = None

    def start_recording(self):
        self.samples = []
        self.start_time = time.time()
        self.recording = True

    def stop_recording(self):
        self.recording = False

    def imu_callback(self, msg: Imu):
        if not self.recording or self.start_time is None:
            return
        t = time.time() - self.start_time
        ang_z = msg.angular_velocity.z
        self.samples.append((t, ang_z))


def least_squares_fit(xs, ys):
    """간단한 최소제곱 1차식 피팅 (y = a*x + b)"""
    n = len(xs)
    if n < 2:
        return 0.0, 0.0  # 데이터가 너무 적으면 0 리턴

    sum_x = sum(xs)
    sum_y = sum(ys)
    sum_x2 = sum(x * x for x in xs)
    sum_xy = sum(x * y for x, y in zip(xs, ys))

    denom = n * sum_x2 - sum_x * sum_x
    if abs(denom) < 1e-6:
        return 0.0, sum_y / n  # 거의 수직선에 가까운 경우

    a = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_y - a * sum_x) / n
    return a, b


def main():
    # ---------------- ROS2 초기화 ----------------
    rclpy.init()

    # 기본 파라미터 (필요하면 launch에서 override 가능)
    imu_topic = '/imu/data'
    serial_port = '/dev/ttyACM0'
    baudrate = 115200
    config_path = os.path.join(
        os.path.dirname(__file__),
        'pwm_matching_config.yaml'
    )
    output_dir = os.path.dirname(__file__)  # 기본: PWMmatching 폴더

    # 환경변수/CLI로 바꾸고 싶으면 여기서 처리해도 됨
    # ex) os.getenv('PWM_MATCHING_CONFIG', default=config_path)

    node = ImuCollector(imu_topic)

    # ---------------- 시리얼 오픈 ----------------
    try:
        ser = serial.Serial(serial_port, baudrate, timeout=0.01)
        node.get_logger().info(f"Opened serial port {serial_port} @ {baudrate}")
    except serial.SerialException as e:
        node.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
        return

    # ---------------- 설정 YAML 로드 ----------------
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    tests = cfg.get('tests', [])
    segments_cfg = cfg.get('segments', [])

    if not tests:
        node.get_logger().error("No tests defined in config.")
        return

    # ---------------- pygame 초기화 ----------------
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("PWM–IMU Matching Tool")
    font = pygame.font.SysFont(None, 28)

    clock = pygame.time.Clock()

    # ---------------- 실험 결과 저장용 ----------------
    # 각 항목: {name, left_pwm, right_pwm, avg_ang_vel_z}
    results = []

    current_test_idx = 0
    phase = 'idle'  # 'idle', 'running', 'done'
    trial_start_time = None

    running = True

    def draw_screen():
        screen.fill((30, 30, 30))
        if current_test_idx < len(tests):
            test = tests[current_test_idx]
            text_lines = [
                f"Test {current_test_idx+1} / {len(tests)} : {test['name']}",
                f"  left_pwm = {test['left_pwm']}, right_pwm = {test['right_pwm']}",
                "",
                f"Phase: {phase}",
                "",
                "Controls:",
                "  SPACE : start/run current test",
                "  ESC   : quit",
                "",
                "Procedure:",
                "  - Set robot in safe area.",
                "  - Press SPACE to start this test.",
                "  - Robot will move for 'duration' seconds.",
                "  - 2~4s 구간의 IMU angular.z를 평균 내서 속도로 사용.",
            ]
        else:
            text_lines = [
                "All tests finished.",
                "Press ESC to exit.",
            ]

        y = 50
        for line in text_lines:
            surf = font.render(line, True, (220, 220, 220))
            screen.blit(surf, (40, y))
            y += 30

        # 최근 결과 몇 개 보여주기
        y += 20
        surf = font.render("Recent results (up to 5):", True, (180, 180, 180))
        screen.blit(surf, (40, y))
        y += 30

        for r in results[-5:]:
            line = f"{r['name']}: L={r['left_pwm']}, R={r['right_pwm']}, avg ang.z={r['avg_ang_vel_z']:.3f}"
            surf = font.render(line, True, (180, 180, 180))
            screen.blit(surf, (40, y))
            y += 25

        pygame.display.flip()

    # ---------------- 메인 루프 ----------------
    while running:
        # ROS 콜백 처리
        rclpy.spin_once(node, timeout_sec=0.01)

        # pygame 이벤트 처리
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    # SPACE 누르면: idle 상태에서 현재 테스트 시작
                    if phase == 'idle' and current_test_idx < len(tests):
                        test = tests[current_test_idx]
                        duration = float(test.get('duration', 5.0))

                        node.get_logger().info(
                            f"Starting test {current_test_idx+1}/{len(tests)}: "
                            f"{test['name']} (L={test['left_pwm']}, R={test['right_pwm']}, dur={duration}s)"
                        )

                        # IMU 기록 시작
                        node.start_recording()

                        # PWM 명령 보내기 시작
                        trial_start_time = time.time()
                        phase = 'running'

        # running phase면 PWM 계속 전송
        if phase == 'running' and current_test_idx < len(tests):
            test = tests[current_test_idx]
            duration = float(test.get('duration', 5.0))
            now = time.time()
            t_elapsed = now - trial_start_time

            # 현재 PWM 유지
            cmd_str = f"{test['left_pwm']} {test['right_pwm']}\n"
            try:
                ser.write(cmd_str.encode('ascii'))
            except serial.SerialException as e:
                node.get_logger().error(f"Serial write error: {e}")
                running = False
                break

            if t_elapsed >= duration:
                # 테스트 종료
                node.stop_recording()

                # 로봇 정지
                try:
                    ser.write(b"0 0\n")
                except serial.SerialException as e:
                    node.get_logger().error(f"Serial write error on stop: {e}")

                # 2~4초 사이의 IMU angular.z 평균
                valid_samples = [
                    ang_z for (t, ang_z) in node.samples
                    if 2.0 <= t <= 4.0
                ]
                if valid_samples:
                    avg_ang_z = sum(valid_samples) / len(valid_samples)
                else:
                    avg_ang_z = 0.0
                    node.get_logger().warn(
                        f"No IMU samples in 2~4s window for test {test['name']}"
                    )

                node.get_logger().info(
                    f"Test {test['name']} done. avg angular.z = {avg_ang_z:.4f} rad/s "
                    f"({len(valid_samples)} samples)"
                )

                results.append({
                    'name': test['name'],
                    'left_pwm': test['left_pwm'],
                    'right_pwm': test['right_pwm'],
                    'avg_ang_vel_z': float(avg_ang_z),
                })

                # 다음 테스트로
                current_test_idx += 1
                phase = 'idle'
                trial_start_time = None

                if current_test_idx >= len(tests):
                    phase = 'done'

        draw_screen()
        clock.tick(30)  # 30 FPS

    # 루프 종료 → 시리얼 정리
    try:
        ser.write(b"0 0\n")
    except Exception:
        pass
    ser.close()

    # ---------------- 결과 처리: 구간별 1차 피팅 ----------------
    # 여기서는 "PWM의 절댓값 vs |avg_ang_vel_z|" 로 피팅
    abs_data = []
    for r in results:
        pwm_mag = max(abs(r['left_pwm']), abs(r['right_pwm']))
        vel_mag = abs(r['avg_ang_vel_z'])
        abs_data.append((pwm_mag, vel_mag))

    segment_results = []
    for seg in segments_cfg:
        name = seg['name']
        pmin = seg['pwm_min']
        pmax = seg['pwm_max']
        xs = []
        ys = []

        for pwm_mag, vel_mag in abs_data:
            if pmin <= pwm_mag <= pmax:
                xs.append(pwm_mag)
                ys.append(vel_mag)

        if len(xs) < 2:
            a, b = 0.0, 0.0
        else:
            a, b = least_squares_fit(xs, ys)

        segment_results.append({
            'name': name,
            'pwm_min': pmin,
            'pwm_max': pmax,
            'slope': float(a),
            'intercept': float(b),
            'num_points': len(xs),
        })

        node.get_logger().info(
            f"Segment {name} [{pmin}, {pmax}] : y ≈ {a:.6f} * pwm + {b:.6f} "
            f"(n={len(xs)})"
        )

    # ---------------- 결과 YAML 저장 ----------------
    os.makedirs(output_dir, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(output_dir, f"pwm_matching_result_{timestamp}.yaml")

    out_data = {
        'raw_results': results,
        'segments': segment_results,
        'note': 'vel = slope * |pwm| + intercept, where vel is |angular_velocity_z| [rad/s]',
    }

    with open(out_path, 'w') as f:
        yaml.dump(out_data, f, sort_keys=False)

    node.get_logger().info(f"Saved PWM matching result to: {out_path}")

    # ROS2 종료
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()


if __name__ == '__main__':
    main()
