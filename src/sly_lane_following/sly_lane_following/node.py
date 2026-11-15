"""ROS adapter: stamped visual evidence only; metric paths remain empty."""

from collections import OrderedDict
import json
from pathlib import Path as FilePath

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from .core import LaneDetector
from .control import ImageServo
from ackermann_msgs.msg import AckermannDriveStamped
from std_srvs.srv import Trigger


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class LaneNode(Node):
    def __init__(self):
        super().__init__("sly_lane_perception")
        default = FilePath(get_package_share_directory("sly_lane_following")) / "config/real_bag.yaml"
        self.declare_parameter("config", str(default))
        self.declare_parameter("image_topic", "/oak/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info")
        self.declare_parameter("reliable_input", True)
        self.declare_parameter("input_timeout_s", 0.5)
        self.timeout = float(self.get_parameter("input_timeout_s").value)
        if self.timeout <= 0:
            raise ValueError("input_timeout_s must be positive")
        with open(self.get_parameter("config").value, encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        if not isinstance(self.config, dict):
            raise ValueError("Lane config must be a YAML mapping")
        self.detector = LaneDetector(self.config)
        self.declare_parameter("control_config", "")
        control_file = self.get_parameter("control_config").value
        self.servo = ImageServo(yaml.safe_load(FilePath(control_file).read_text())) if control_file else None
        self.request_pub = self.create_publisher(AckermannDriveStamped, "/sly/control/request", 5) if self.servo else None
        self.create_service(Trigger, "/sly/lane/reset", self.reset_service)
        self.declare_parameter("trace_path", "")
        trace_path=self.get_parameter("trace_path").value
        self.trace=open(trace_path,"x",buffering=1) if trace_path else None
        self.bridge = CvBridge()
        self.images, self.infos = OrderedDict(), OrderedDict()
        self.last_clock = self.get_clock().now().nanoseconds
        self.last_received = self.last_clock
        self.last_paired_clock = self.last_clock
        self.last_processed = None
        self.last_source = None
        self.last_decoded = None
        self.last_header = None
        self.last_reason = None
        self.timeout_sent = False
        reliability = ReliabilityPolicy.RELIABLE if self.get_parameter("reliable_input").value else ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(depth=5, reliability=reliability, durability=DurabilityPolicy.VOLATILE)
        self.debug_pub = self.create_publisher(Image, "/sly/lane/debug_image", 2)
        self.status_pub = self.create_publisher(DiagnosticArray, "/sly/lane/status", 5)
        self.path_pub = self.create_publisher(Path, "/sly/lane/local_path", 5)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self.on_info, qos)
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("Image-space mode; metric local_path is unused. Actuation requires a separately armed adapter.")

    def reset_service(self, request, response):
        self.reset()
        response.success=True
        response.message="Perception reset only; actuator enable remains separate."
        return response

    def reset(self):
        self.detector = LaneDetector(self.config)
        if self.servo: self.servo.reset()
        self.images.clear()
        self.infos.clear()
        self.last_processed = self.last_source = None
        self.last_decoded = self.last_header = None
        self.timeout_sent = False

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.reset()
            self.last_received = now
            self.last_paired_clock = now
        self.last_clock = now
        return now

    @staticmethod
    def cache(target, message):
        target[stamp_ns(message.header)] = message
        while len(target) > 20:
            target.popitem(last=False)

    def on_image(self, msg):
        now = self.check_clock()
        stamp = stamp_ns(msg.header)
        if self.last_source is not None and stamp < self.last_source - 1_000_000_000:
            self.reset()
            self.last_paired_clock = now
        self.last_source = stamp
        self.last_received = now
        self.timeout_sent = False
        self.cache(self.images, msg)

    def on_info(self, msg):
        self.check_clock()
        self.cache(self.infos, msg)

    def tick(self):
        now = self.check_clock()
        # ROS time freezes while bag playback is paused, so pausing is not a timeout.
        if (now - self.last_received) / 1e9 > self.timeout:
            if not self.timeout_sent:
                self.failure("STALE_IMAGE", self.last_header, self.last_decoded)
                self.timeout_sent = True
            return
        newer = [t for t in self.images if self.last_processed is None or t > self.last_processed]
        if not newer:
            return
        paired = [t for t in newer if t in self.infos]
        if not paired:
            # Wait briefly for the same stamped CameraInfo, never borrow old intrinsics.
            if (now - self.last_paired_clock) / 1e9 > 0.2:
                self.failure("CAMERA_INFO_MISSING", self.images[max(newer)].header)
            return
        stamp = max(paired)
        image = self.images[stamp]
        info = self.infos[stamp]
        self.last_paired_clock = now
        self.last_processed = stamp
        if (now - stamp) / 1e9 > self.timeout:
            self.failure("STALE_IMAGE_HEADER", image.header)
            return
        if image.header.frame_id != info.header.frame_id:
            self.failure("CAMERA_INFO_FRAME_MISMATCH", image.header)
            return
        camera = {
            "width": info.width, "height": info.height,
            "K": list(info.k), "D": list(info.d),
            "distortion_model": info.distortion_model,
        }
        try:
            raw = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            self.last_decoded, self.last_header = raw, image.header
            result = self.detector.process(raw, camera, stamp)
            debug = result.get("debug_image")
            if debug is None:
                raise ValueError("LaneDetector.process must return debug_image")
            if self.servo:
                control = self.servo.compute(result,raw.shape,stamp)
                result['control']=control
                for key,color in [('near',(255,0,255)),('far',(255,0,255)),('reference_near',(255,255,255)),('reference_far',(255,255,255))]:
                    if key in control: cv2.circle(debug,tuple(int(v) for v in control[key]),5,color,2)
                cv2.putText(debug,f"{control['reason']} v={control['speed']:.2f} steer={control['steering_angle']:.3f}",
                            (10,debug.shape[0]-15),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            if self.trace:
                self.trace.write(json.dumps({k:v for k,v in result.items() if k != "debug_image"})+"\n")
            self.publish(result, image.header, debug)
        except (ValueError, TypeError, CvBridgeError, cv2.error) as exc:
            self.failure("PROCESSING_ERROR", image.header, detail=str(exc))

    def failure(self, reason, header=None, image=None, detail=""):
        # Input/calibration faults break temporal evidence; a future valid
        # observation must initialize its corridor from fresh measurements.
        self.detector = LaneDetector(self.config)
        header = header if header is not None else Header(stamp=self.get_clock().now().to_msg())
        result = {"vision_valid": False, "reason": reason, "confidence": 0.0, "selected": [], "detail": detail}
        debug = None
        if image is not None:
            debug = image.copy()
            cv2.putText(debug, reason, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        self.publish(result, header, debug)

    def publish(self, result, header, debug=None):
        if debug is not None:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            msg.header = header
            self.debug_pub.publish(msg)
        control = result.get('control',dict(speed=0.,steering_angle=0.,reason=result.get('reason','INVALID')))
        if self.request_pub:
            request=AckermannDriveStamped()
            request.header=header
            request.drive.speed=float(control['speed'])
            request.drive.steering_angle=float(control['steering_angle'])
            self.request_pub.publish(request)
        reason = str(result.get("reason", "UNKNOWN"))
        vision = DiagnosticStatus(
            name="sly/lane/vision", hardware_id=header.frame_id,
            level=DiagnosticStatus.OK if result.get("vision_valid", False) else DiagnosticStatus.WARN,
            message=reason,
            values=[KeyValue(key="vision_valid", value=str(bool(result.get("vision_valid", False))).lower()),
                    KeyValue(key="confidence", value=str(result.get("confidence", 0.0))),
                    KeyValue(key="selected", value=str(result.get("selected", []))),
                    KeyValue(key="coordinates", value="undistorted image pixels"),
                    KeyValue(key="detail", value=str(result.get("detail", ""))),
                    KeyValue(key="control", value=str(control))],
        )
        metric = DiagnosticStatus(
            name="sly/lane/metric_path", hardware_id="sly_rear_axle_ground",
            level=DiagnosticStatus.OK, message="NOT_USED_IMAGE_SERVO",
            values=[KeyValue(key="metric_valid", value="false")],
        )
        self.status_pub.publish(DiagnosticArray(header=header, status=[vision, metric]))
        empty = Path()
        empty.header.stamp = header.stamp
        empty.header.frame_id = "sly_rear_axle_ground"
        self.path_pub.publish(empty)
        if reason != self.last_reason:
            self.get_logger().info("Vision status: " + reason)
            self.last_reason = reason


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
