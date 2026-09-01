#!/usr/bin/env python3
"""直接通过 Wi-Fi 对 ESP32 发送开环 PWM，用于电机硬件排障。

本脚本不依赖 ROS 2，不读取编码器，也不运行 PID：
它只重复发送 ESP32 固件支持的 ``M <left_pwm> <right_pwm>`` UDP 指令。

示例（车轮必须悬空）：
  python3 esp32_pwm_test.py --left 80 --right 80 --duration 1
  python3 esp32_pwm_test.py --left 100 --right -100 --duration 0.8

无论正常结束、Ctrl+C 或异常，都会连续发送停车指令。
"""

from __future__ import annotations

import argparse
import socket
import sys
import time


PWM_LIMIT = 255


def pwm_value(value: str) -> int:
    """Parse and limit a signed ESP32 PWM value."""
    try:
        pwm = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('PWM 必须是整数') from exc
    if not -PWM_LIMIT <= pwm <= PWM_LIMIT:
        raise argparse.ArgumentTypeError(f'PWM 必须在 {-PWM_LIMIT} 到 {PWM_LIMIT} 之间')
    return pwm


def positive_float(value: str) -> float:
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('必须是数字') from exc
    if number <= 0:
        raise argparse.ArgumentTypeError('必须大于 0')
    return number


def send_command(sock: socket.socket, address: tuple[str, int], left: int, right: int) -> None:
    sock.sendto(f'M {left} {right}\n'.encode('ascii'), address)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='通过 UDP 直接给 ESP32 发送开环电机 PWM；不经过 ROS/PID/编码器。')
    parser.add_argument('--host', default='192.168.4.1', help='ESP32 IP（默认：192.168.4.1）')
    parser.add_argument('--port', type=int, default=8888, help='ESP32 UDP 端口（默认：8888）')
    parser.add_argument('--left', required=True, type=pwm_value, help='左轮 PWM，范围 -255 到 255')
    parser.add_argument('--right', required=True, type=pwm_value, help='右轮 PWM，范围 -255 到 255')
    parser.add_argument('--duration', type=positive_float, default=1.0,
                        help='持续时间（秒，默认：1.0）')
    parser.add_argument('--rate', type=positive_float, default=20.0,
                        help='重复发送频率 Hz（默认：20；必须高于 ESP32 的 500ms 失联保护）')
    parser.add_argument('--dry-run', action='store_true', help='只显示将发送的内容，不向 ESP32 发包')
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 1 <= args.port <= 65535:
        print('端口必须在 1 到 65535 之间', file=sys.stderr)
        return 2

    command = f'M {args.left} {args.right}'
    print(f'目标：{args.host}:{args.port}，开环指令：{command}，持续 {args.duration:.2f}s')
    if args.dry_run:
        print('dry-run：未发送任何 UDP 数据包。')
        return 0

    address = (args.host, args.port)
    interval = 1.0 / args.rate
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)
    deadline = time.monotonic() + args.duration

    try:
        while time.monotonic() < deadline:
            send_command(sock, address, args.left, args.right)
            time.sleep(interval)
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，正在停车。')
    finally:
        # 重复发送，避免单个 UDP 包丢失；固件自身也有 500ms 失联停车保护。
        for _ in range(3):
            try:
                send_command(sock, address, 0, 0)
            except OSError as exc:
                print(f'发送停车指令失败：{exc}', file=sys.stderr)
                break
            time.sleep(0.05)
        sock.close()

    print('已发送 M 0 0，测试结束。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
