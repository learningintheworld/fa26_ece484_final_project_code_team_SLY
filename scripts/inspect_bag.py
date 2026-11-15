#!/usr/bin/env python3
"""Read a ROS 2 SQLite bag and write a sensor-data inspection report.

Run inside the existing ROS 2 Jazzy container. The input bag is never modified.
No sensor driver, controller, ROS publisher or executor is started.
"""

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sqlite3
import sys

sys.dont_write_bytecode = True

RGB = "/oak/rgb/image_raw"
INFO = "/oak/rgb/camera_info"
POINTS = "/ouster/points"
IMU = "/ouster/imu"
EXPECTED_TYPES = {
    RGB: "sensor_msgs/msg/Image",
    INFO: "sensor_msgs/msg/CameraInfo",
    POINTS: "sensor_msgs/msg/PointCloud2",
    IMU: "sensor_msgs/msg/Imu",
}
POINT_TYPES = {
    1: ("i1", 1), 2: ("u1", 1), 3: ("i2", 2), 4: ("u2", 2),
    5: ("i4", 4), 6: ("u4", 4), 7: ("f4", 4), 8: ("f8", 8),
}


class Issues:
    """Keep exact issue counts and bounded examples, not thousands of duplicates."""

    def __init__(self):
        self.items = {}

    def add(self, severity, code, topic, detail):
        key = (severity, code, topic)
        entry = self.items.setdefault(key, {
            "severity": severity, "code": code, "topic": topic,
            "count": 0, "examples": [],
        })
        entry["count"] += 1
        if len(entry["examples"]) < 5:
            entry["examples"].append(str(detail))

    def list(self):
        return list(self.items.values())

    def has_errors(self):
        return any(item["severity"] == "error" for item in self.items.values())


def stamp_ns(msg):
    stamp = msg.header.stamp
    if not 0 <= int(stamp.nanosec) < 1_000_000_000:
        raise ValueError(f"非法 nanosec: {stamp.nanosec}")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def interval_stats(times):
    if len(times) < 2:
        return {"interval_count": 0}
    diffs = np.diff(np.asarray(times, dtype=np.int64))
    positive = diffs[diffs > 0]
    baseline = float(np.median(positive)) if positive.size else None
    gap_indices = np.flatnonzero(diffs > 3.0 * baseline) if baseline else []
    largest = sorted(gap_indices, key=lambda i: int(diffs[i]), reverse=True)[:10]
    return {
        "interval_count": int(diffs.size),
        "median_ms": float(np.median(diffs)) / 1e6,
        "p95_ms": float(np.percentile(diffs, 95)) / 1e6,
        "min_ms": float(diffs.min()) / 1e6,
        "max_ms": float(diffs.max()) / 1e6,
        "reversed_count": int(np.count_nonzero(diffs < 0)),
        "duplicate_count": int(np.count_nonzero(diffs == 0)),
        "positive_median_ms": baseline / 1e6 if baseline else None,
        "gap_threshold_ms": 3.0 * baseline / 1e6 if baseline else None,
        "gap_count": len(gap_indices),
        "largest_gaps": [{
            "after_message_index_zero_based": int(i),
            "after_seconds_from_first": (int(times[i]) - int(times[0])) / 1e9,
            "gap_ms": int(diffs[i]) / 1e6,
        } for i in largest],
    }


def finite_array(values, expected, label):
    a = np.asarray(values, dtype=np.float64)
    if a.shape != (expected,) or not np.isfinite(a).all():
        raise ValueError(f"{label} 应为 {expected} 个有限数，实际 shape={a.shape}")
    return a


def check_image(msg, bridge):
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h <= 0 or w <= 0:
        raise ValueError(f"图像尺寸非法: {w}x{h}")
    if msg.encoding != "bgr8":
        raise ValueError(f"本数据预期 bgr8，实际 {msg.encoding!r}")
    if step < w * 3 or len(msg.data) != h * step:
        raise ValueError(f"图像布局不一致: step={step}, w={w}, h={h}, bytes={len(msg.data)}")
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    if image.shape != (h, w, 3) or image.dtype != np.uint8:
        raise ValueError(f"cv_bridge 输出异常: {image.shape}, {image.dtype}")
    return image


def check_camera_info(msg, issues):
    if int(msg.height) <= 0 or int(msg.width) <= 0:
        raise ValueError("CameraInfo 的图像尺寸必须大于零")
    k = finite_array(msg.k, 9, "K")
    finite_array(msg.r, 9, "R")
    finite_array(msg.p, 12, "P")
    d = np.asarray(msg.d, dtype=np.float64)
    if d.ndim != 1 or not np.isfinite(d).all():
        raise ValueError("D 必须是一维有限数数组")
    expected = {"plumb_bob": 5, "rational_polynomial": 8, "equidistant": 4}.get(msg.distortion_model)
    if expected is not None and d.size != expected:
        raise ValueError(f"{msg.distortion_model} 的 D 应为 {expected} 维，实际 {d.size}")
    if expected is None:
        issues.add("warning", "unknown_distortion_model", INFO, msg.distortion_model)
    if k[0] <= 0 or k[4] <= 0 or not np.allclose(k[6:9], [0, 0, 1]):
        issues.add("warning", "invalid_camera_intrinsics", INFO, k.tolist())
    return {
        "width": int(msg.width), "height": int(msg.height),
        "frame_id": msg.header.frame_id, "distortion_model": msg.distortion_model,
        "K": k.tolist(), "D": d.tolist(),
    }


def check_points(msg):
    h, w = int(msg.height), int(msg.width)
    step, row = int(msg.point_step), int(msg.row_step)
    if h < 0 or w < 0 or step <= 0 or row < w * step:
        raise ValueError(f"点云布局非法: height={h}, width={w}, point_step={step}, row_step={row}")
    if len(msg.data) != h * row:
        raise ValueError(f"点云字节数 {len(msg.data)} != height*row_step {h * row}")
    fields = []
    names = set()
    for f in msg.fields:
        kind = POINT_TYPES.get(int(f.datatype))
        if kind is None or int(f.count) <= 0 or int(f.offset) < 0:
            raise ValueError(f"字段定义非法: {f.name}, type={f.datatype}, count={f.count}")
        if int(f.offset) + int(f.count) * kind[1] > step:
            raise ValueError(f"字段 {f.name} 超出 point_step")
        if f.name in names:
            raise ValueError(f"重复点云字段: {f.name}")
        names.add(f.name)
        fields.append({"name": f.name, "offset": int(f.offset),
                       "datatype": int(f.datatype), "count": int(f.count)})
    if not {"x", "y", "z"} <= names:
        raise ValueError("点云缺少 x/y/z 字段")
    for f in msg.fields:
        if f.name in {"x", "y", "z"} and int(f.count) != 1:
            raise ValueError(f"点云坐标字段 {f.name} 不是标量")
    return {"width": w, "height": h, "point_step": step, "row_step": row,
            "frame_id": msg.header.frame_id, "fields": fields}


def sampled_xyz(msg):
    """Create a strided view; only materialize <= roughly 6,000 sampled points."""
    fields = {f.name: f for f in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + POINT_TYPES[int(fields[n].datatype)][0] for n in ["x", "y", "z"]],
        "offsets": [int(fields[n].offset) for n in ["x", "y", "z"]],
        "itemsize": int(msg.point_step),
    })
    data = np.ndarray((int(msg.height), int(msg.width)), dtype=dtype,
                      buffer=msg.data, strides=(int(msg.row_step), int(msg.point_step)))
    stride = max(1, math.ceil(math.sqrt(data.size / 6000)))
    subset = data[::stride, ::stride]
    return np.column_stack([subset[n].ravel() for n in ["x", "y", "z"]]).astype(np.float64)


def check_imu(msg):
    orientation = finite_array([msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                msg.orientation.w], 4, "IMU orientation")
    oc = finite_array(msg.orientation_covariance, 9, "orientation_covariance")
    av = finite_array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], 3, "angular_velocity")
    ac = finite_array(msg.angular_velocity_covariance, 9, "angular_velocity_covariance")
    la = finite_array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], 3, "linear_acceleration")
    lc = finite_array(msg.linear_acceleration_covariance, 9, "linear_acceleration_covariance")
    return orientation, av, la, [bool(c[0] == -1) for c in [oc, ac, lc]]


def write_rgb_contact(samples, output):
    width, height, label = 384, 240, 34
    canvas = np.full((2 * (height + label), 3 * width, 3), 255, np.uint8)
    for index, (seconds, frame) in enumerate(samples[:6]):
        x, y = (index % 3) * width, (index // 3) * (height + label)
        text = f"RGB {seconds:.2f}s  {frame.shape[1]}x{frame.shape[0]}"
        cv2.putText(canvas, text, (x + 8, y + 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (20, 20, 20), 1, cv2.LINE_AA)
        canvas[y + label:y + label + height, x:x + width] = cv2.resize(frame, (width, height))
    path = output / "rgb_contact.jpg"
    if not cv2.imwrite(str(path), canvas):
        raise IOError(f"无法写入 {path}")
    return path.name


def write_point_preview(samples, output):
    cell, bound = 500, 40.0
    canvas = np.full((cell, cell * len(samples), 3), 255, np.uint8)
    summaries = []
    for i, (seconds, xyz, frame) in enumerate(samples):
        finite = np.isfinite(xyz).all(axis=1)
        points = xyz[finite]
        visible = points[(np.abs(points[:, 0]) <= bound) & (np.abs(points[:, 1]) <= bound)]
        cx, cy, scale = i * cell + cell // 2, cell // 2, 5.0
        cv2.line(canvas, (i * cell + 30, cy), ((i + 1) * cell - 30, cy), (190, 190, 190), 1)
        cv2.line(canvas, (cx, 40), (cx, cell - 35), (190, 190, 190), 1)
        for point in visible:
            x, y = int(cx + point[0] * scale), int(cy - point[1] * scale)
            cv2.circle(canvas, (x, y), 1, (150, 75, 30), -1)
        cv2.circle(canvas, (cx, cy), 4, (30, 30, 220), -1)
        cv2.putText(canvas, f"{seconds:.2f}s | {frame}", (i * cell + 12, 24), cv2.FONT_HERSHEY_SIMPLEX, .55, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.putText(canvas, "XY +/-40m; x right, y up; strided sample", (i * cell + 10, cell - 10), cv2.FONT_HERSHEY_SIMPLEX, .43, (20, 20, 20), 1, cv2.LINE_AA)
        summaries.append({"relative_record_time_s": seconds, "frame": frame,
                          "sampled_points": len(xyz), "finite_sampled_points": int(finite.sum()),
                          "displayed_points": len(visible)})
    path = output / "pointcloud_xy_preview.png"
    if not cv2.imwrite(str(path), canvas):
        raise IOError(f"无法写入 {path}")
    return path.name, summaries


def number(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def write_markdown(report, output):
    lines = ["# GEM e4 数据检查", "", f"- 数据：`{report['bag_path']}`",
             f"- 结果：**{report['status']}**；逐条读取 {report['messages_read']} 条，反序列化成功 {report['messages_deserialized']} 条。",
             "- 原始 bag 只读；没有启动传感器、车辆控制或发布 ROS 消息。", "",
             "| Topic | 记录数 / 元数据数 | 首条相对时间(s) | header 间隔 median / p95 / max(ms) | 逆序 / 重复 / 明显间隙 |",
             "|---|---:|---:|---|---|"]
    for topic, info in report["topics"].items():
        stats = info["header_intervals"]
        text = " / ".join(number(stats.get(k)) for k in ["median_ms", "p95_ms", "max_ms"])
        counts = " / ".join(str(stats.get(k, 0)) for k in ["reversed_count", "duplicate_count", "gap_count"])
        lines.append(f"| `{topic}` | {info['record_count']} / {info.get('metadata_count', '—')} | {number(info.get('first_record_s'))} | {text} | {counts} |")
    lines += ["", "明显间隙定义为大于该 topic 正间隔中位数的 3 倍；这是一项筛查指标，不等同于已证明丢包。记录时间间隔、header 时间间隔、时间差与最大间隙明细见 `report.json`。", "",
              "## 数据边界", ""]
    lines += [f"- {note}" for note in report["notes"]]
    match = report.get("rgb_camera_info_matching", {})
    if match:
        lines += [f"- RGB / CameraInfo 按完整 header 时间戳匹配：{match['matched_messages']} 对；未匹配图像 {match['unmatched_images']} 条，未匹配 CameraInfo {match['unmatched_camera_info']} 条。"]
    lines += ["", "## 异常", ""]
    if not report["issues"]:
        lines.append("未发现上述检查范围内的异常；这不代表已经验证标定精度或驾驶闭环。")
    for item in report["issues"]:
        lines.append(f"- **{item['severity']}** `{item['topic']}` / `{item['code']}`：{item['count']} 次。示例：{item['examples'][0]}")
    lines += ["", "## 预览", ""]
    for name in report.get("artifacts", []):
        lines.append(f"- [{name}]({name})")
    lines += ["", "RGB 联系图是时间抽样，点云图是局部坐标系内的稀疏 XY 投影；均不能替代完整交互回放。"]
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def analyze(bag, output):
    import rosbag2_py
    from cv_bridge import CvBridge
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml

    issues = Issues()
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))["rosbag2_bagfile_information"]
    if metadata.get("storage_identifier") != "sqlite3":
        raise ValueError("本检查器要求 sqlite3 格式 bag")
    meta_topics = {t["topic_metadata"]["name"]: t for t in metadata["topics_with_message_count"]}
    duration = int(metadata["duration"]["nanoseconds"]) / 1e9
    start = int(metadata["starting_time"]["nanoseconds_since_epoch"])
    storage_schemas = []
    for relative in metadata.get("relative_file_paths", []):
        storage_path = (bag / relative).resolve()
        if bag not in storage_path.parents:
            raise ValueError("元数据中的存储文件路径超出 bag 目录")
        try:
            with sqlite3.connect(storage_path.as_uri() + "?mode=ro&immutable=1", uri=True) as connection:
                rows = connection.execute("SELECT schema_version, ros_distro FROM schema").fetchall()
            storage_schemas.append({"file": relative, "schemas": [
                {"schema_version": int(version), "ros_distro": distro} for version, distro in rows]})
        except sqlite3.Error as exc:
            issues.add("warning", "storage_schema_unavailable", "bag", f"{relative}: {exc}")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    for topic, kind in types.items():
        try:
            classes[topic] = get_message(kind)
        except Exception as exc:
            issues.add("error", "message_type_unavailable", topic, exc)
    for topic, kind in EXPECTED_TYPES.items():
        if topic not in types:
            issues.add("error", "missing_required_topic", topic, "bag 中不存在该 topic")
        elif types[topic] != kind:
            issues.add("error", "unexpected_message_type", topic, types[topic])

    entries = {topic: {"type": kind, "record_times": [], "header_times": [],
                       "header_record_times": [], "frames": Counter(),
                       "decoded": 0, "payload_checks_passed": 0} for topic, kind in types.items()}
    bridge = CvBridge()
    image_stamps, info_stamps = Counter(), Counter()
    image_signatures, info_signatures = defaultdict(set), defaultdict(set)
    rgb_samples, cloud_samples = [], []
    rgb_targets = [duration * f for f in [0, .2, .4, .6, .8, .97]]
    cloud_targets = [duration * f for f in [0, .5, .97]]
    rgb_index = cloud_index = 0
    camera_first = point_first = None
    unavailable = Counter()
    imu_min = np.full(6, np.inf)
    imu_max = np.full(6, -np.inf)
    total = decoded = 0
    try:
        while reader.has_next():
            topic, blob, recorded = reader.read_next()
            total += 1
            entry = entries[topic]
            entry["record_times"].append(int(recorded))
            if total % 2500 == 0:
                print(f"已读取 {total}/{metadata['message_count']} 条消息", flush=True)
            if topic not in classes:
                continue
            try:
                msg = deserialize_message(blob, classes[topic])
                decoded += 1
                entry["decoded"] += 1
            except Exception as exc:
                issues.add("error", "deserialize_failed", topic, f"记录 {total}: {exc}")
                continue
            try:
                header = stamp_ns(msg)
                entry["header_times"].append(header)
                entry["header_record_times"].append(int(recorded))
                entry["frames"][msg.header.frame_id] += 1
                if header <= 0:
                    issues.add("warning", "nonpositive_header_stamp", topic, header)
                relative = (int(recorded) - start) / 1e9
                if topic == RGB:
                    image = check_image(msg, bridge)
                    image_stamps[header] += 1
                    image_signatures[header].add((int(msg.width), int(msg.height), msg.header.frame_id))
                    if rgb_index < len(rgb_targets) and relative >= rgb_targets[rgb_index]:
                        rgb_samples.append((relative, image.copy()))
                        rgb_index += 1
                elif topic == INFO:
                    details = check_camera_info(msg, issues)
                    if camera_first is None:
                        camera_first = details
                    elif details != camera_first:
                        issues.add("warning", "camera_info_changed", topic, f"相对时间 {relative:.3f}s 标定/尺寸/frame 与首条不同")
                    info_stamps[header] += 1
                    info_signatures[header].add((int(msg.width), int(msg.height), msg.header.frame_id))
                elif topic == POINTS:
                    details = check_points(msg)
                    if point_first is None:
                        point_first = details
                    elif details != point_first:
                        issues.add("warning", "pointcloud_layout_changed", topic, f"相对时间 {relative:.3f}s 字段/尺寸/frame 与首条不同")
                    if cloud_index < len(cloud_targets) and relative >= cloud_targets[cloud_index]:
                        cloud_samples.append((relative, sampled_xyz(msg), msg.header.frame_id))
                        cloud_index += 1
                elif topic == IMU:
                    orientation, angular, accel, missing = check_imu(msg)
                    for key, value in zip(["orientation", "angular_velocity", "linear_acceleration"], missing):
                        unavailable[key] += int(value)
                    if not missing[0] and abs(float(np.linalg.norm(orientation)) - 1.0) > .05:
                        issues.add("warning", "imu_quaternion_not_unit", topic, orientation.tolist())
                    values = np.concatenate([angular, accel])
                    imu_min = np.minimum(imu_min, values)
                    imu_max = np.maximum(imu_max, values)
                else:
                    issues.add("warning", "no_payload_validator", topic, "已反序列化，但未做此类型专属字段检查")
                entry["payload_checks_passed"] += 1
            except Exception as exc:
                issues.add("error", "payload_validation_failed", topic, f"记录 {total}: {exc}")
    except Exception as exc:
        issues.add("error", "reader_failed", "bag", f"读取至 {total} 条后失败: {exc}")

    topic_report = {}
    for topic, entry in entries.items():
        records, headers = entry["record_times"], entry["header_times"]
        rec_stats, head_stats = interval_stats(records), interval_stats(headers)
        meta_count = meta_topics.get(topic, {}).get("message_count")
        if meta_count is not None and len(records) != meta_count:
            issues.add("error", "message_count_mismatch", topic, f"元数据 {meta_count}，实际读取 {len(records)}")
        for basis, stats in [("record", rec_stats), ("header", head_stats)]:
            for key in ["reversed_count", "duplicate_count", "gap_count"]:
                if stats.get(key, 0):
                    issues.add("warning", f"{basis}_{key}", topic, f"{stats[key]} 个；详见时间统计")
        offsets = np.asarray(entry["header_record_times"], np.int64) - np.asarray(headers, np.int64)
        timing_offset = {k: float(fn(offsets)) / 1e6 for k, fn in [
            ("median_ms", np.median), ("p95_ms", lambda a: np.percentile(a, 95)),
            ("min_ms", np.min), ("max_ms", np.max)]} if offsets.size else {}
        topic_report[topic] = {
            "type": entry["type"], "metadata_count": meta_count, "record_count": len(records),
            "deserialized_count": entry["decoded"], "payload_checks_passed": entry["payload_checks_passed"],
            "frames": dict(entry["frames"]),
            "first_record_s": (records[0] - start) / 1e9 if records else None,
            "last_record_s": (records[-1] - start) / 1e9 if records else None,
            "record_intervals": rec_stats, "header_intervals": head_stats,
            "record_minus_header": timing_offset,
        }
    for topic in meta_topics.keys() - entries.keys():
        issues.add("error", "metadata_topic_missing_from_storage", topic, "元数据包含此 topic，但存储中不存在")
    if total != metadata["message_count"]:
        issues.add("error", "total_count_mismatch", "bag", f"元数据 {metadata['message_count']}，读取 {total}")
    matched = sum((image_stamps & info_stamps).values())
    matching = {"matched_messages": matched,
                "unmatched_images": sum(image_stamps.values()) - matched,
                "unmatched_camera_info": sum(info_stamps.values()) - matched}
    if matching["unmatched_images"] or matching["unmatched_camera_info"]:
        issues.add("warning", "image_camera_info_unmatched", RGB, matching)
    mismatches = [stamp for stamp in image_signatures.keys() & info_signatures.keys()
                  if image_signatures[stamp] != info_signatures[stamp]]
    matching["dimension_or_frame_mismatches"] = len(mismatches)
    if mismatches:
        issues.add("error", "image_camera_info_mismatch", RGB, f"{len(mismatches)} 个时间戳的尺寸或 frame 不匹配")

    notes = [
        "这是离线传感器数据检查，不是相机外参精度、传感器硬件同步或车辆闭环性能验证。",
        "时间差 record−header 包含记录/传输延迟和可能的时钟偏差，不能仅据此认定硬件同步准确。",
        "当前检查逐条反序列化所有消息，并解码全部 RGB；点云逐条检查布局与字段，不逐点验证全部距离值。",
    ]
    source_distros = sorted({s["ros_distro"] for f in storage_schemas for s in f["schemas"] if s["ros_distro"]})
    if source_distros:
        notes.append(f"SQLite schema 记录的来源 ROS 版本为 {', '.join(source_distros)}；当前检查环境为 {os.environ.get('ROS_DISTRO', '未设置')}。旧 metadata 缺少 ros_distro 时，ros2 bag info 显示 unknown 不代表解码失败。")
    for name, present in [
        ("TF", any(t in types for t in ["/tf", "/tf_static"])),
        ("GNSS", any("NavSatFix" in k or "INSNavGeod" in k for k in types.values())),
        ("车速/里程计", any("VehicleSpeedRpt" in k or "Odometry" in k for k in types.values())),
        ("深度图", any(("depth" in t or "stereo" in t) and k == "sensor_msgs/msg/Image" for t, k in types.items())),
    ]:
        if not present:
            notes.append(f"未录到{name}；不能据此还原未保存的测量或已标定的传感器融合关系。")
    for topic in [POINTS, IMU]:
        delay = topic_report.get(topic, {}).get("first_record_s")
        if delay is not None:
            notes.append(f"{topic} 首条记录在 bag 起点后 {delay:.3f} 秒；开头这段无该 topic，不应自动解释为传感器丢包，需结合采集启动顺序。")
    if unavailable["orientation"]:
        notes.append(f"{unavailable['orientation']} 条 Ouster IMU 的 orientation_covariance[0]=-1，表示无有效姿态估计；单位四元数不能作为真实航向。")

    artifacts, point_samples_report = [], []
    if rgb_samples:
        artifacts.append(write_rgb_contact(rgb_samples, output))
    if cloud_samples:
        artifact, point_samples_report = write_point_preview(cloud_samples, output)
        artifacts.append(artifact)
    report = {
        "schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
        "bag_path": str(bag), "storage_id": "sqlite3", "read_only_input": True,
        "inspection_ros_distro": os.environ.get("ROS_DISTRO"), "storage_schemas": storage_schemas,
        "metadata_duration_s": duration, "metadata_message_count": metadata["message_count"],
        "messages_read": total, "messages_deserialized": decoded,
        "topics": topic_report, "first_camera_info": camera_first,
        "first_pointcloud_layout": point_first, "rgb_camera_info_matching": matching,
        "imu": {"unavailable_measurement_counts": dict(unavailable),
                "angular_velocity_and_acceleration_min": imu_min.tolist() if np.isfinite(imu_min).all() else None,
                "angular_velocity_and_acceleration_max": imu_max.tolist() if np.isfinite(imu_max).all() else None,
                "units": ["rad/s", "rad/s", "rad/s", "m/s^2", "m/s^2", "m/s^2"]},
        "rgb_sample_relative_record_times_s": [s[0] for s in rgb_samples],
        "pointcloud_preview_samples": point_samples_report,
        "notes": notes, "issues": issues.list(), "artifacts": artifacts,
        "status": "FAIL" if issues.has_errors() else ("PASS_WITH_WARNINGS" if issues.list() else "PASS"),
    }
    (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_markdown(report, output)
    print(f"完成：{report['status']}；读取 {total} 条，反序列化 {decoded} 条；报告：{output / 'report.md'}", flush=True)
    return 1 if issues.has_errors() else 0


def main():
    parser = argparse.ArgumentParser(description="只读检查 GEM e4 rosbag，写入报告与抽样预览")
    parser.add_argument("bag", type=Path, help="包含 metadata.yaml 和 .db3 的 bag 目录")
    parser.add_argument("--output", type=Path, required=True, help="报告输出目录；必须在原始 bag 目录之外")
    args = parser.parse_args()
    bag, output = args.bag.expanduser().resolve(), args.output.expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        parser.error("bag 必须是包含 metadata.yaml 的现有目录")
    if output == bag or bag in output.parents:
        parser.error("output 不得位于原始 bag 目录内")
    output.mkdir(parents=True, exist_ok=True)
    try:
        global np, cv2
        import numpy as np
        import cv2
        return analyze(bag, output)
    except Exception as exc:
        failure = {"status": "FAIL", "bag_path": str(bag), "read_only_input": True,
                   "fatal_error": {"type": type(exc).__name__, "detail": str(exc)}}
        (output / "report.json").write_text(json.dumps(failure, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        (output / "report.md").write_text(f"# GEM e4 数据检查\n\n**FAIL**：`{type(exc).__name__}`：{exc}\n\n输入 bag 未修改。检查没有完整完成，不能将本报告视为数据通过。\n", encoding="utf-8")
        print(f"检查失败：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
