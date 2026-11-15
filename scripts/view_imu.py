#!/usr/bin/env python3
"""View raw /ouster/imu using bag-header time; run in the SLY ROS environment."""

from collections import deque
import math
import time

import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuViewer(Node):
    def __init__(self):
        super().__init__("sly_imu_viewer")
        self.times = deque(maxlen=2000)
        self.values = [deque(maxlen=2000) for _ in range(6)]
        self.origin_ns = self.last_ns = None
        self.changed = False
        self.create_subscription(Imu, "/ouster/imu", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        a, b = msg.angular_velocity, msg.linear_acceleration
        values = (a.x, a.y, a.z, b.x, b.y, b.z)
        if not all(math.isfinite(v) for v in values):
            return
        if self.origin_ns is None or stamp < self.last_ns:
            self.origin_ns = stamp
            self.times.clear()
            for series in self.values:
                series.clear()
        self.last_ns = stamp
        elapsed = (stamp - self.origin_ns) / 1e9
        self.times.append(elapsed)
        for series, value in zip(self.values, values):
            series.append(value)
        while self.times and self.times[0] < elapsed - 10.0:
            self.times.popleft()
            for series in self.values:
                series.popleft()
        self.changed = True


def main():
    rclpy.init()
    node = None
    try:
        node = ImuViewer()
        plt.ion()
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        fig.canvas.manager.set_window_title("SLY | Ouster IMU")
        title = fig.suptitle("Waiting for /ouster/imu")
        lines = []
        for ax, label in zip(axes, ("Angular velocity (rad/s)", "Acceleration (m/s²)")):
            lines.extend(ax.plot([], [], label=component)[0] for component in "xyz")
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")
        axes[-1].set_xlabel("Header time since first message (s)")
        axes[-1].set_xlim(0, 10)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        plt.show(block=False)
        last_draw = 0.0
        while rclpy.ok() and plt.fignum_exists(fig.number):
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if node.changed and now - last_draw >= 0.1:
                xs = list(node.times)
                for line, series in zip(lines, node.values):
                    line.set_data(xs, list(series))
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)
                axes[-1].set_xlim(max(0.0, xs[-1] - 10.0), max(10.0, xs[-1]))
                title.set_text("Ouster IMU | raw angular velocity and acceleration")
                fig.canvas.draw_idle()
                node.changed, last_draw = False, now
            plt.pause(0.01)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        plt.close("all")
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
