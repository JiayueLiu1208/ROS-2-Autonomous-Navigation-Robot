#!/usr/bin/env python3

import json
import math
import os
import socket
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def _read_proc_stat() -> tuple[int, int] | None:
    try:
        with Path('/proc/stat').open('r', encoding='utf-8') as proc_stat:
            first_line = proc_stat.readline().strip().split()
    except OSError:
        return None

    if len(first_line) < 5 or first_line[0] != 'cpu':
        return None

    values = [int(value) for value in first_line[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    total = sum(values)
    return total, idle


def _read_memory_status() -> tuple[float, float, float] | None:
    meminfo: dict[str, float] = {}
    try:
        with Path('/proc/meminfo').open('r', encoding='utf-8') as proc_meminfo:
            for line in proc_meminfo:
                parts = line.replace(':', '').split()
                if len(parts) >= 2:
                    meminfo[parts[0]] = float(parts[1])
    except OSError:
        return None

    total_kb = meminfo.get('MemTotal')
    available_kb = meminfo.get('MemAvailable')
    if not total_kb or available_kb is None or total_kb <= 0.0:
        return None

    used_kb = max(0.0, total_kb - available_kb)
    used_percent = 100.0 * used_kb / total_kb
    return used_percent, used_kb / 1024.0, total_kb / 1024.0


def _read_temperature_c() -> float | None:
    values: list[float] = []
    for temp_path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            raw_value = temp_path.read_text(encoding='utf-8').strip()
            temp_c = float(raw_value) / 1000.0
        except (OSError, TypeError, UnicodeDecodeError, ValueError):
            continue
        if 0.0 <= temp_c <= 125.0:
            values.append(temp_c)
    if not values:
        return None
    return max(values)


def _read_disk_percent(path: str) -> float | None:
    try:
        usage = os.statvfs(path)
    except OSError:
        return None

    total = usage.f_blocks * usage.f_frsize
    available = usage.f_bavail * usage.f_frsize
    if total <= 0:
        return None
    return 100.0 * (total - available) / total


def _finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return value


class SystemStatusPublisher(Node):
    def __init__(self) -> None:
        super().__init__('jetson_status_publisher')

        self.declare_parameter('topic', 'jetson_status')
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('disk_path', '/')

        self.topic = str(self.get_parameter('topic').value)
        self.publish_period_sec = max(
            0.2,
            float(self.get_parameter('publish_period_sec').value),
        )
        self.disk_path = str(self.get_parameter('disk_path').value)

        self.publisher = self.create_publisher(String, self.topic, 10)
        self.previous_cpu_sample = _read_proc_stat()
        self.hostname = socket.gethostname()

        self.create_timer(self.publish_period_sec, self.publish_status)
        self.get_logger().info(
            '[JETSON_STATUS] publishing %s every %.2f s'
            % (self.topic, self.publish_period_sec)
        )

    def publish_status(self) -> None:
        now = self.get_clock().now()
        cpu_percent = self.cpu_percent()
        memory = _read_memory_status()
        temp_c = _read_temperature_c()
        disk_percent = _read_disk_percent(self.disk_path)

        try:
            load_1min, load_5min, load_15min = os.getloadavg()
        except OSError:
            load_1min = load_5min = load_15min = None

        status = {
            'stamp_sec': now.nanoseconds * 1.0e-9,
            'hostname': self.hostname,
            'cpu_percent': _finite_or_none(cpu_percent),
            'memory_percent': _finite_or_none(memory[0] if memory else None),
            'memory_used_mb': _finite_or_none(memory[1] if memory else None),
            'memory_total_mb': _finite_or_none(memory[2] if memory else None),
            'temperature_c': _finite_or_none(temp_c),
            'load_1min': _finite_or_none(load_1min),
            'load_5min': _finite_or_none(load_5min),
            'load_15min': _finite_or_none(load_15min),
            'disk_percent': _finite_or_none(disk_percent),
        }

        msg = String()
        msg.data = json.dumps(status, separators=(',', ':'))
        self.publisher.publish(msg)

    def cpu_percent(self) -> float | None:
        sample = _read_proc_stat()
        if sample is None:
            return None
        if self.previous_cpu_sample is None:
            self.previous_cpu_sample = sample
            return None

        total, idle = sample
        previous_total, previous_idle = self.previous_cpu_sample
        self.previous_cpu_sample = sample

        total_delta = total - previous_total
        idle_delta = idle - previous_idle
        if total_delta <= 0:
            return None
        busy_fraction = 1.0 - (idle_delta / total_delta)
        return max(0.0, min(100.0, 100.0 * busy_fraction))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemStatusPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
