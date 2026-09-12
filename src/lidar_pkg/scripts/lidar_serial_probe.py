#!/usr/bin/env python3
"""
Probe the LiDAR through its original USB-to-UART adapter.

The tool sends the binary scan command (A5 60), captures the raw byte stream,
and validates the intensity packet format documented for this LiDAR:

    AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) |
    [intensity(1) | distance(2)] * LSN
"""

from __future__ import annotations

import argparse
import time

import serial


HEADER = b"\xAA\x55"
SCAN_COMMAND = b"\xA5\x60"


def positive_float(value: str) -> float:
    result = float(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return result


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="发送雷达扫描命令并以 HEX 显示、校验串口数据。"
    )
    parser.add_argument("--port", required=True,
                        help="串口，例如 /dev/ttyACM0 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=150000,
                        help="波特率（默认：150000）")
    parser.add_argument("--duration", type=positive_float, default=6.0,
                        help="采集秒数（默认：6）")
    parser.add_argument("--warmup", type=positive_float, default=1.0,
                        help="打开串口后等待秒数（默认：1）")
    parser.add_argument("--retries", type=int, default=3,
                        help="发送 A5 60 的次数（默认：3）")
    parser.add_argument("--hex-bytes", type=int, default=256,
                        help="最多显示多少个原始字节（默认：256）")
    return parser.parse_args()


def uint16_le(data: bytes | bytearray, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def packet_checksum_valid(packet: bytes | bytearray) -> bool:
    if len(packet) < 13 or packet[:2] != HEADER:
        return False

    ct = packet[2]
    lsn = packet[3]
    if lsn == 0 or len(packet) != 10 + 3 * lsn:
        return False

    checksum = 0x55AA
    checksum ^= uint16_le(packet, 4)
    checksum ^= (lsn << 8) | ct
    checksum ^= uint16_le(packet, 6)
    for sample in range(lsn):
        offset = 10 + 3 * sample
        checksum ^= packet[offset]
        checksum ^= uint16_le(packet, offset + 1)
    return checksum == uint16_le(packet, 8)


def inspect_packets(data: bytes) -> tuple[int, int]:
    headers = 0
    valid_packets = 0
    offset = 0

    while True:
        offset = data.find(HEADER, offset)
        if offset < 0:
            break
        headers += 1
        if offset + 4 > len(data):
            break

        packet_size = 10 + 3 * data[offset + 3]
        packet_end = offset + packet_size
        if packet_end <= len(data) and packet_checksum_valid(data[offset:packet_end]):
            valid_packets += 1
            offset = packet_end
        else:
            offset += 1

    return headers, valid_packets


def print_hex(data: bytes, limit: int) -> None:
    shown = data[:max(0, limit)]
    if not shown:
        print("（没有收到字节）")
        return
    for offset in range(0, len(shown), 16):
        chunk = shown[offset:offset + 16]
        print(f"{offset:04X}: " + " ".join(f"{byte:02X}" for byte in chunk))
    if len(data) > len(shown):
        print(f"……其余 {len(data) - len(shown)} 字节未显示")


def main() -> int:
    args = parse_arguments()
    if args.baud <= 0:
        raise SystemExit("波特率必须大于 0")
    if args.retries <= 0:
        raise SystemExit("--retries 必须大于 0")

    print(f"打开 {args.port}：{args.baud} baud, 8N1，无流控")
    try:
        lidar = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as exc:
        print(f"无法打开串口：{exc}")
        return 2

    captured = bytearray()
    try:
        lidar.reset_input_buffer()
        print(f"等待雷达稳定 {args.warmup:.1f} 秒……")
        time.sleep(args.warmup)

        for attempt in range(args.retries):
            lidar.write(SCAN_COMMAND)
            lidar.flush()
            print(f"已发送 A5 60（{attempt + 1}/{args.retries}）")
            time.sleep(0.20)

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            chunk = lidar.read(lidar.in_waiting or 1)
            if chunk:
                captured.extend(chunk)
    except KeyboardInterrupt:
        print("\n已停止采集。")
    except serial.SerialException as exc:
        print(f"串口通信失败：{exc}")
        return 2
    finally:
        lidar.close()

    headers, valid_packets = inspect_packets(bytes(captured))
    print("\n收到的前部原始数据：")
    print_hex(bytes(captured), args.hex_bytes)
    print("\n诊断统计：")
    print(f"  总字节数：       {len(captured)}")
    print(f"  AA 55 包头数：   {headers}")
    print(f"  校验正确数据包： {valid_packets}")

    if valid_packets:
        print("结论：雷达、USB 转换器、150000 波特率和数据协议均正常。")
        return 0
    if headers:
        print("结论：看到包头但校验失败；重点检查波特率、丢字节或帧格式。")
        return 1
    if captured:
        print("结论：串口有电平变化但没有雷达包头；重点检查端口、波特率和接线。")
        return 1

    print("结论：完全没有数据；尝试另一个 COM/ttyACM 端口并检查 TX、GND。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
