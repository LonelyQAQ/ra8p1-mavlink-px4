#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
假飞控（Fake Flight Controller）—— 在 PC 上模拟 MAVLink 飞控，用于和 RA8P1 主控对通。

不需要真飞控。流程：
    RA8P1 (SCI1/H2) ── TX/RX 交叉 ── USB-TTL ── PC ── 本脚本

用法：
    # 串口
    python fake_fc.py --master COM7 --baud 57600
    # UDP（方便本机自测）
    python fake_fc.py --master udpin:0.0.0.0:14550

本脚本会：
    1. 以 1 Hz 发 HEARTBEAT（扮演飞控）
    2. 等待并打印来自对端（RA8P1）的所有消息
    3. 对主控常发的几类消息做解码显示（HEARTBEAT / STATUSTEXT / LANDING_TARGET /
       OBSTACLE_DISTANCE / DISTANCE_SENSOR）
"""
import argparse
import sys
import time

from pymavlink import mavutil


# 主控（companion computer）常发的消息，做友好解码
INTERESTING = {
    'HEARTBEAT', 'STATUSTEXT', 'LANDING_TARGET', 'OBSTACLE_DISTANCE',
    'DISTANCE_SENSOR', 'SYSTEM_TIME', 'PING', 'COMMAND_ACK', 'TIMESYNC',
}


def describe(msg, mav):
    """把一个 MAVLink 消息压成一行可读文本。"""
    t = msg.get_type()
    if t == 'HEARTBEAT':
        return (f"HEARTBEAT from sys={msg.get_srcSystem()} comp={msg.get_srcComponent()} "
                f"type={mavutil.mavlink.enums['MAV_TYPE'][msg.type].name} "
                f"autopilot={mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name} "
                f"status={mavutil.mavlink.enums['MAV_STATE'][msg.system_status].name}")
    if t == 'STATUSTEXT':
        return f"STATUSTEXT [{msg.severity}] {msg.text}"
    if t == 'LANDING_TARGET':
        return (f"LANDING_TARGET angle_x={msg.angle_x:.4f} angle_y={msg.angle_y:.4f} "
                f"dist={msg.distance:.3f} pos_valid={msg.position_valid}")
    if t == 'OBSTACLE_DISTANCE':
        d = list(msg.distances)
        return f"OBSTACLE_DISTANCE {len(d)} cells, min={min(d)}cm max={max(d)}cm"
    if t == 'DISTANCE_SENSOR':
        return (f"DISTANCE_SENSOR id={msg.id} cur={msg.current_distance}cm "
                f"min={msg.min_distance} max={msg.max_distance} orient={msg.orientation}")
    return t


def main():
    ap = argparse.ArgumentParser(description='假飞控：模拟 MAVLink 飞控与主控对通')
    ap.add_argument('--master', required=True,
                    help='pymavlink 连接串，如 COM7 或 udpin:0.0.0.0:14550')
    ap.add_argument('--baud', type=int, default=57600,
                    help='串口波特率（默认 57600，MAVLink 常用值）')
    ap.add_argument('--system-id', type=int, default=1,
                    help='本端 system id（假飞控 = 1）')
    ap.add_argument('--component-id', type=int, default=1,
                    help='本端 component id（1 = MAV_COMP_ID_AUTOPILOT1）')
    ap.add_argument('--heartbeat-hz', type=float, default=1.0,
                    help='心跳频率（默认 1 Hz）')
    ap.add_argument('--wait', type=float, default=30.0,
                    help='等待对端心跳的超时秒数（默认 30）')
    ap.add_argument('--no-wait', action='store_true',
                    help='不等待对端心跳，直接开始')
    args = ap.parse_args()

    print(f"[fake-fc] 打开 {args.master} @ {args.baud}")
    mav = mavutil.mavlink_connection(
        args.master, baud=args.baud,
        source_system=args.system_id, source_component=args.component_id,
    )

    if not args.no_wait:
        print(f"[fake-fc] 等待对端（RA8P1）HEARTBEAT，最多 {args.wait:.0f}s ...")
        hb = mav.wait_heartbeat(timeout=args.wait)
        if hb is None:
            print("  [!] 超时：没收到对端心跳。逐项检查：")
            print("      1) 接线：RA8P1 的 TX → USB-TTL 的 RX，RX → TX，GND 共地")
            print("      2) 波特率是否一致（本端 %d）" % args.baud)
            print("      3) RA8P1 是否真的在以 1Hz 发心跳")
            print("      4) 串口没被别的程序占用（串口助手/QGC）")
            print("  —— 继续运行，后续若收到消息仍会打印 ——")
        else:
            print(f"  [OK] 收到对端心跳：sys={hb.get_srcSystem()} "
                  f"comp={hb.get_srcComponent()}")

    period = 1.0 / args.heartbeat_hz if args.heartbeat_hz > 0 else 1.0
    next_hb = 0.0
    rx_count = 0
    t0 = time.time()

    print("[fake-fc] 开始收发（Ctrl+C 退出）")
    try:
        while True:
            now = time.time()

            # --- 发心跳 ---
            if now >= next_hb:
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                    0,  # base_mode
                    0,  # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_hb = now + period

            # --- 收消息 ---
            msg = mav.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.005)
                continue

            t = msg.get_type()
            if t == 'BAD_DATA':
                continue

            rx_count += 1
            elapsed = now - t0
            if t in INTERESTING or rx_count <= 20:
                print(f"[{elapsed:7.2f}s] {describe(msg, mav)}")
            elif rx_count % 50 == 0:
                print(f"[{elapsed:7.2f}s] ... 已收 {rx_count} 条消息")

    except KeyboardInterrupt:
        print(f"\n[fake-fc] 退出。共收到 {rx_count} 条消息。")


if __name__ == '__main__':
    sys.exit(main())
