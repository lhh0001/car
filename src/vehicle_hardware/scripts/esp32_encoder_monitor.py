#!/usr/bin/env python3
"""显示 ESP32 经 Wi-Fi 回传的左右编码器累计计数。

只发送 ``M 0 0`` 保持 ESP32 的遥测回传地址并确保停车；不驱动电机。
运行前关闭 wifi_bridge 和其他电机测试程序，因为 ESP32 一次只向最后
注册的 UDP 客户端回传数据。
"""

from __future__ import annotations

import argparse
import select
import socket
import time


def positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('必须是数字') from exc
    if result <= 0:
        raise argparse.ArgumentTypeError('必须大于 0')
    return result


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='只读显示 ESP32 编码器遥测（兼容新旧 E 报文）。')
    parser.add_argument('--host', default='192.168.4.1', help='ESP32 IP（默认：192.168.4.1）')
    parser.add_argument('--port', type=int, default=8888, help='ESP32 UDP 端口（默认：8888）')
    parser.add_argument('--duration', type=positive_float, default=15.0,
                        help='监听秒数（默认：15）')
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 1 <= args.port <= 65535:
        raise SystemExit('端口必须在 1 到 65535 之间')

    address = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)

    print('正在监听编码器。请手转左轮或右轮；本工具只发送停车 M 0 0。')
    print(' time(s)   left   right      d_left  d_right')

    started = time.monotonic()
    deadline = started + args.duration
    next_heartbeat = 0.0
    previous: tuple[int, int] | None = None
    received = 0

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                sock.sendto(b'M 0 0\n', address)
                next_heartbeat = now + 0.1

            ready, _, _ = select.select([sock], [], [], 0.05)
            if not ready:
                continue

            packet, _ = sock.recvfrom(256)
            for line in packet.decode(errors='ignore').splitlines():
                fields = line.split()
                if not fields or fields[0] != 'E':
                    continue
                try:
                    if len(fields) == 6:
                        current = (int(fields[4]), int(fields[5]))
                    elif len(fields) == 3:
                        current = (int(fields[1]), int(fields[2]))
                    else:
                        continue
                except ValueError:
                    continue

                delta = (0, 0) if previous is None else (
                    current[0] - previous[0], current[1] - previous[1])
                # 每秒约有 50 个回传；只在首次或计数变化时打印，避免
                # 静止状态淹没真正的编码器脉冲。
                if previous is None or current != previous:
                    elapsed = time.monotonic() - started
                    print(f'{elapsed:7.2f} {current[0]:7d} {current[1]:7d}'
                          f' {delta[0]:11d} {delta[1]:8d}')
                previous = current
                received += 1
    except KeyboardInterrupt:
        print('\n已停止监听。')
    finally:
        for _ in range(3):
            try:
                sock.sendto(b'M 0 0\n', address)
            except OSError:
                break
            time.sleep(0.05)
        sock.close()

    if received == 0:
        print('没有收到 E 编码器包：检查是否已连接 car-esp32、ESP32 是否在线。')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
