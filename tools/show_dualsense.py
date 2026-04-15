#!/usr/bin/env python3
"""实时显示 DualSense (event6) 按键/轴信号。"""

from __future__ import annotations

import argparse
import datetime as dt
import select
import shutil
import signal
import sys
import unicodedata
from dataclasses import dataclass

from evdev import InputDevice, ecodes


@dataclass(frozen=True)
class SignalMeta:
    physical_name: str
    event_name: str
    event_type: int


KEY_SIGNAL_MAP: dict[int, SignalMeta] = {
    ecodes.BTN_SOUTH: SignalMeta("X (交叉键)", "BTN_SOUTH", ecodes.EV_KEY),
    ecodes.BTN_EAST: SignalMeta("O (圆圈键)", "BTN_EAST", ecodes.EV_KEY),
    ecodes.BTN_NORTH: SignalMeta("△ (三角键)", "BTN_NORTH", ecodes.EV_KEY),
    ecodes.BTN_WEST: SignalMeta("□ (方块键)", "BTN_WEST", ecodes.EV_KEY),
    ecodes.BTN_TL: SignalMeta("L1", "BTN_TL", ecodes.EV_KEY),
    ecodes.BTN_TR: SignalMeta("R1", "BTN_TR", ecodes.EV_KEY),
    ecodes.BTN_TL2: SignalMeta("L2 (数字开关)", "BTN_TL2", ecodes.EV_KEY),
    ecodes.BTN_TR2: SignalMeta("R2 (数字开关)", "BTN_TR2", ecodes.EV_KEY),
    ecodes.BTN_SELECT: SignalMeta("Create", "BTN_SELECT", ecodes.EV_KEY),
    ecodes.BTN_START: SignalMeta("Options", "BTN_START", ecodes.EV_KEY),
    ecodes.BTN_MODE: SignalMeta("PS", "BTN_MODE", ecodes.EV_KEY),
    ecodes.BTN_THUMBL: SignalMeta("L3 (左摇杆下压)", "BTN_THUMBL", ecodes.EV_KEY),
    ecodes.BTN_THUMBR: SignalMeta("R3 (右摇杆下压)", "BTN_THUMBR", ecodes.EV_KEY),
}

ABS_SIGNAL_MAP: dict[int, SignalMeta] = {
    ecodes.ABS_X: SignalMeta("左摇杆 (左右)", "ABS_X", ecodes.EV_ABS),
    ecodes.ABS_Y: SignalMeta("左摇杆 (上下)", "ABS_Y", ecodes.EV_ABS),
    ecodes.ABS_RX: SignalMeta("右摇杆 (左右)", "ABS_RX", ecodes.EV_ABS),
    ecodes.ABS_RY: SignalMeta("右摇杆 (上下)", "ABS_RY", ecodes.EV_ABS),
    ecodes.ABS_Z: SignalMeta("L2 模拟值", "ABS_Z", ecodes.EV_ABS),
    ecodes.ABS_RZ: SignalMeta("R2 模拟值", "ABS_RZ", ecodes.EV_ABS),
    ecodes.ABS_HAT0X: SignalMeta("方向键 (左右)", "ABS_HAT0X", ecodes.EV_ABS),
    ecodes.ABS_HAT0Y: SignalMeta("方向键 (上下)", "ABS_HAT0Y", ecodes.EV_ABS),
}

ALL_SIGNALS: list[tuple[int, SignalMeta]] = (
    list(KEY_SIGNAL_MAP.items()) + list(ABS_SIGNAL_MAP.items())
)


def display_width(text: str) -> int:
    width = 0
    for ch in text:
        # CJK 宽字符在终端一般占 2 列，其他字符占 1 列。
        width += 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
    return width


def pad_to_width(text: str, target_width: int) -> str:
    padding = max(0, target_width - display_width(text))
    return text + (" " * padding)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="实时显示 DualSense 输入信号（刷新面板模式）"
    )
    parser.add_argument(
        "--device",
        default="/dev/input/event6",
        help="输入设备路径，默认: /dev/input/event6",
    )
    return parser.parse_args()


def draw_screen(
    device: InputDevice,
    signal_values: dict[tuple[int, int], int],
    last_event_text: str,
) -> None:
    term_size = shutil.get_terminal_size(fallback=(120, 40))
    w = term_size.columns
    lines: list[str] = []
    header = f"DualSense 实时输入监控 | 设备: {device.path} ({device.name})"
    lines.append(header[: max(0, w - 1)])
    lines.append("按 Ctrl+C 退出。显示值会持续刷新，不使用逐行 print。"[: max(0, w - 1)])
    lines.append("-" * max(0, w - 1))
    col1_width = 22
    col2_width = 16
    col3_width = 8

    header_line = (
        f"{pad_to_width('物理按键名称', col1_width)} "
        f"{pad_to_width('evtest事件代码', col2_width)} "
        f"{'当前值':>{col3_width}}"
    )
    lines.append(header_line[: max(0, w - 1)])
    lines.append("-" * max(0, w - 1))

    for code, meta in ALL_SIGNALS:
        value = signal_values.get((meta.event_type, code), 0)
        row_line = (
            f"{pad_to_width(meta.physical_name, col1_width)} "
            f"{pad_to_width(meta.event_name, col2_width)} "
            f"{value:>{col3_width}}"
        )
        lines.append(row_line[: max(0, w - 1)])

    lines.append("-" * max(0, w - 1))
    lines.append(f"最近事件: {last_event_text}"[: max(0, w - 1)])

    # 用 ANSI 控制码实现“清屏+回到左上角”，避免逐行 print 滚屏。
    screen_text = "\x1b[2J\x1b[H" + "\n".join(lines)
    sys.stdout.write(screen_text)
    sys.stdout.flush()


def monitor(device_path: str) -> None:
    running = True

    def _handle_stop(_: int, __) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    device = InputDevice(device_path)
    signal_values: dict[tuple[int, int], int] = {}
    last_event_text = "尚无事件"

    # 记录已知信号初始值，让 UI 启动时有完整行
    for code in KEY_SIGNAL_MAP:
        signal_values[(ecodes.EV_KEY, code)] = 0
    for code in ABS_SIGNAL_MAP:
        signal_values[(ecodes.EV_ABS, code)] = 0

    while running:
        ready, _, _ = select.select([device.fd], [], [], 0.02)
        if ready:
            for event in device.read():
                if event.type not in (ecodes.EV_KEY, ecodes.EV_ABS):
                    continue
                signal_values[(event.type, event.code)] = event.value

                event_name = ecodes.bytype[event.type].get(event.code, str(event.code))
                timestamp = dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                last_event_text = (
                    f"{timestamp} | {event_name} (code={event.code}) -> {event.value}"
                )

        draw_screen(device, signal_values, last_event_text)

    sys.stdout.write("\n已退出 DualSense 监控。\n")
    sys.stdout.flush()


def main() -> None:
    args = parse_args()
    monitor(args.device)


if __name__ == "__main__":
    main()
