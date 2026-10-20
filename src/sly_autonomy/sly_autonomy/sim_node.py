"""ROS 2 adapter for isolated simulation. No PACMod support is present."""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .control import ControlConfig, pure_pursuit
from .safety import Inputs, SafetyConfig, SafetySupervisor


class SimController(Node):
    def __init__(self):
        super().__init__("sly_sim_controller")
        readonly = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, readonly).value

        if not parameter("simulation_only", True):
            raise ValueError("This node only supports simulation_only=true")
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("use_sim_time must be true; hardware execution is not supported")
        self.add_on_set_parameters_callback(self._parameters_changed)
        self.local_frame = parameter("local_frame", "base_link")
        self.control_config = ControlConfig(**{
            name: parameter(name, value)
            for name, value in vars(ControlConfig()).items()
        })
        safety_config = SafetyConfig(**{
            name: parameter(name, value)
            for name, value in vars(SafetyConfig()).items()
        })
        self.max_accel = parameter("max_accel_mps2", 0.3)
        if not math.isfinite(self.max_accel) or self.max_accel <= 0:
            raise ValueError("max_accel_mps2 must be finite and positive")
        self.supervisor = SafetySupervisor(safety_config)
        self.points = []
        self.path_valid = False
        self.path_stamp = None
        self.odom_stamp = None
        self.perception_stamp = None
        self.speed = float("nan")
        self.perception_valid = False
        self.obstacle = True
        self.sign = False
        self.last_command_speed = 0.0
        self.last_tick = None
        self.last_state = None
        self.create_subscription(Path, "/sly/local_path", self._path, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/sly/odom", self._odom, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/sly/perception", self._perception,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, "/sly/manual_takeover", self._manual, 10)
        self.command_pub = self.create_publisher(AckermannDrive, "/sly/sim/ackermann_cmd", 1)
        self.state_pub = self.create_publisher(DiagnosticArray, "/sly/state", 10)
        self.create_service(SetBool, "/sly/enable", self._enable)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "SIMULATION ONLY. Disabled until /sly/enable succeeds. "
            "Fresh path, odometry, and explicit perception validity are required. "
            "Output defaults to /sly/sim/ackermann_cmd, disconnected from GEM."
        )

    @staticmethod
    def _parameters_changed(parameters):
        if any(p.name == "use_sim_time" and p.value is not True for p in parameters):
            return SetParametersResult(successful=False, reason="simulation clock is mandatory")
        return SetParametersResult(successful=True)

    @staticmethod
    def _stamp(header):
        value = header.stamp.sec + header.stamp.nanosec * 1e-9
        return value if value > 0.0 else None

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _path(self, message):
        self.path_stamp = self._stamp(message.header)
        self.path_valid = False
        if message.header.frame_id != self.local_frame:
            return
        # A local path has one coordinate frame and acquisition time, carried
        # by the enclosing header. Conflicting pose frames are rejected.
        if any(p.header.frame_id not in ("", self.local_frame) for p in message.poses):
            return
        self.points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
        try:
            pure_pursuit(self.points, self.control_config)
            self.path_valid = True
        except (TypeError, ValueError, OverflowError):
            pass

    def _odom(self, message):
        self.odom_stamp = self._stamp(message.header)
        velocity = message.twist.twist.linear
        self.speed = math.hypot(velocity.x, velocity.y)

    def _perception(self, message):
        self.perception_stamp = self._stamp(message.header)
        self.perception_valid = False
        self.obstacle = True
        self.sign = False
        matches = [s for s in message.status if s.name == "sly/perception"]
        if len(matches) != 1 or message.header.frame_id != self.local_frame:
            return
        status = matches[0]
        values = {item.key: item.value for item in status.values}
        if len(values) != len(status.values):
            return
        keys = ("valid", "obstacle_present", "stop_sign_stop_required")
        if any(values.get(key) not in ("true", "false") for key in keys):
            return
        self.perception_valid = status.level == DiagnosticStatus.OK and values["valid"] == "true"
        self.obstacle = values["obstacle_present"] == "true"
        self.sign = values["stop_sign_stop_required"] == "true"

    def _manual(self, message):
        self.supervisor.set_manual_takeover(message.data)

    def _inputs(self):
        return Inputs(self.path_stamp, self.odom_stamp, self.perception_stamp,
                      self.path_valid, self.perception_valid, self.speed,
                      self.obstacle, self.sign)

    def _enable(self, request, response):
        if request.data:
            response.success, response.message = self.supervisor.request_enable(
                self._now(), self._inputs())
        else:
            self.supervisor.disable()
            response.success = True
            response.message = "disabled; zero simulator speed requested on next control tick"
        return response

    def _tick(self):
        now = self._now()
        decision = self.supervisor.tick(now, self._inputs())
        command = AckermannDrive()
        if decision.allow_motion:
            # Path was validated on callback; use a conservative acceleration
            # ramp only when moving. Stop decisions bypass this ramp.
            result = pure_pursuit(self.points, self.control_config)
            dt = 0.0 if self.last_tick is None else max(0.0, min(0.1, now - self.last_tick))
            command.speed = min(result.speed_mps, self.last_command_speed + self.max_accel * dt)
            command.steering_angle = result.steering_rad
        self.last_command_speed = command.speed
        self.last_tick = now
        self.command_pub.publish(command)
        status = DiagnosticStatus()
        status.name = "sly/controller"
        status.level = (DiagnosticStatus.ERROR if decision.state == "FAULT_STOP"
                        else DiagnosticStatus.OK if decision.allow_motion
                        else DiagnosticStatus.WARN)
        status.message = decision.reason
        status.values = [KeyValue(key="state", value=decision.state),
                         KeyValue(key="enabled", value=str(self.supervisor.enabled).lower()),
                         KeyValue(key="target_speed_mps", value=f"{command.speed:.3f}")]
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        report.status = [status]
        self.state_pub.publish(report)
        if self.last_state != decision.state:
            self.get_logger().info(f"{decision.state}: {decision.reason}")
            self.last_state = decision.state


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            # SIGINT may already have invalidated the ROS context. Shutdown
            # publishing is best effort, never an actuator watchdog guarantee.
            if rclpy.ok():
                node.command_pub.publish(AckermannDrive())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
