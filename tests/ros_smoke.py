"""Synthetic ROS adapter test, isolated from Gazebo and hardware.

Run after sourcing the built workspace: ROS_DOMAIN_ID=84 python3 tests/ros_smoke.py
The driver owns a synthetic /clock and stops the controller process on exit.
"""

import json
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class Driver(Node):
    def __init__(self):
        super().__init__("sly_synthetic_smoke_driver")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.path_pub = self.create_publisher(Path, "/sly/local_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "/sly/odom", 10)
        self.perception_pub = self.create_publisher(DiagnosticArray, "/sly/perception", 10)
        self.manual_pub = self.create_publisher(Bool, "/sly/manual_takeover", 10)
        self.create_subscription(AckermannDrive, "/sly/sim/ackermann_cmd", self.command, 10)
        self.create_subscription(DiagnosticArray, "/sly/state", self.state, 10)
        self.enable_client = self.create_client(SetBool, "/sly/enable")
        self.sim_ns = 1_000_000_000
        self.last_command = None
        self.last_state = None
        self.command_count = 0

    def command(self, message):
        self.last_command = message
        self.command_count += 1

    def state(self, message):
        for status in message.status:
            for value in status.values:
                if value.key == "state":
                    self.last_state = value.value

    def advance(self, seconds, *, path=True, odom=True, perception=True,
                measured_speed=0.0, obstacle=False, sign=False, frame="base_link"):
        for _ in range(round(seconds / 0.05)):
            self.sim_ns += 50_000_000
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(self.sim_ns, 1_000_000_000)
            self.clock_pub.publish(clock)
            if path:
                message = Path()
                message.header.stamp = clock.clock
                message.header.frame_id = frame
                for x in (0.0, 2.0, 5.0):
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.orientation.w = 1.0
                    message.poses.append(pose)
                self.path_pub.publish(message)
            if odom:
                message = Odometry()
                message.header.stamp = clock.clock
                message.header.frame_id = "odom"
                message.child_frame_id = "base_link"
                message.pose.pose.orientation.w = 1.0
                message.twist.twist.linear.x = measured_speed
                self.odom_pub.publish(message)
            if perception:
                message = DiagnosticArray()
                message.header.stamp = clock.clock
                message.header.frame_id = "base_link"
                status = DiagnosticStatus()
                status.name = "sly/perception"
                status.level = DiagnosticStatus.OK
                status.values = [KeyValue(key="valid", value="true"),
                                 KeyValue(key="obstacle_present", value=str(obstacle).lower()),
                                 KeyValue(key="stop_sign_stop_required", value=str(sign).lower())]
                message.status = [status]
                self.perception_pub.publish(message)
            # Allow the external controller, its timer and DDS to service a
            # complete synthetic tick before advancing the clock again.
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.005)
                time.sleep(0.005)

    def enable(self, value):
        request = SetBool.Request()
        request.data = value
        future = self.enable_client.call_async(request)
        for _ in range(20):
            self.advance(0.05)
            if future.done():
                response = future.result()
                assert response.success, response.message
                return
        raise AssertionError("enable service timed out")

    def assert_stopped(self, state):
        assert self.last_command is not None, "no command received"
        assert self.last_command.speed == 0.0, self.last_command
        assert self.last_state == state, (self.last_state, state)

    def assert_moving(self):
        assert self.last_command is not None, "no command received"
        assert 0.0 < self.last_command.speed <= 1.0, self.last_command
        assert abs(self.last_command.steering_angle) <= 0.45, self.last_command
        assert self.last_state == "CRUISE", self.last_state


def main():
    if os.environ.get("ROS_DOMAIN_ID") != "84":
        raise RuntimeError("Use ROS_DOMAIN_ID=84 to isolate this synthetic test")
    rclpy.init()
    driver = Driver()
    process = None
    checks = []
    try:
        # Refuse to join an already used domain rather than publish test data
        # onto someone else's clock/control interfaces.
        for _ in range(10):
            rclpy.spin_once(driver, timeout_sec=0.05)
        others = [name for name in driver.get_node_names()
                  if name != "sly_synthetic_smoke_driver"]
        if others:
            raise RuntimeError(f"ROS domain 84 is already in use: {others}")
        with tempfile.TemporaryFile(mode="w+t") as output:
            process = subprocess.Popen(
                [sys.executable, "-m", "sly_autonomy.sim_node", "--ros-args",
                 "-p", "use_sim_time:=true"], stdout=output, stderr=subprocess.STDOUT)
            try:
                assert driver.enable_client.wait_for_service(timeout_sec=10.0), "controller service absent"
                driver.advance(0.6)
                driver.assert_stopped("DISABLED")
                checks.append("default disabled with fresh inputs")
                assert not driver.get_publishers_info_by_topic("/ackermann_cmd")
                checks.append("output isolated from /ackermann_cmd")

                driver.enable(True)
                driver.advance(0.5)
                driver.assert_moving()
                checks.append("explicit enable produces bounded Ackermann command")

                driver.advance(0.8, perception=False)
                driver.assert_stopped("FAULT_STOP")
                driver.advance(0.3)
                driver.assert_stopped("FAULT_STOP")
                checks.append("stale perception stops and latches fault after fresh data returns")

                driver.enable(True)
                driver.advance(0.3)
                driver.assert_moving()
                driver.manual_pub.publish(Bool(data=True))
                driver.advance(0.2)
                driver.assert_stopped("MANUAL")
                driver.manual_pub.publish(Bool(data=False))
                driver.advance(0.3)
                driver.assert_stopped("DISABLED")
                checks.append("manual takeover and release cannot automatically resume")

                driver.enable(True)
                driver.advance(0.3)
                driver.assert_moving()
                driver.enable(False)
                driver.advance(0.4)
                driver.assert_stopped("DISABLED")
                checks.append("explicit disable remains disabled with fresh inputs")

                driver.enable(True)
                driver.advance(0.3, frame="map")
                driver.assert_stopped("FAULT_STOP")
                checks.append("wrong-frame path stops")

                driver.advance(0.2)
                driver.enable(True)
                driver.advance(0.3, sign=True, measured_speed=0.5)
                driver.assert_stopped("SIGN_STOP")
                driver.advance(0.4, sign=True)
                driver.assert_stopped("SIGN_DWELL")
                driver.advance(3.0, sign=True, obstacle=True)
                driver.assert_stopped("OBSTACLE_STOP")
                driver.advance(0.3, sign=True)
                driver.assert_moving()
                checks.append("measured-stop dwell, obstacle priority and same-sign suppression")
                print(json.dumps({"result": "PASS", "checks": checks,
                                  "command_messages_observed": driver.command_count,
                                  "synthetic_simulation_seconds": driver.sim_ns / 1e9}, indent=2))
            except Exception:
                output.seek(0)
                print(output.read(), file=sys.stderr)
                raise
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
