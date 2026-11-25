#!/usr/bin/env python3
"""ROS 2 web gateway for the robot dashboard.

The browser never talks to Zenoh or DDS directly. This process joins local ROS
domain 3 (ROS_DOMAIN_ID=03), caches a product-facing state model, and serves it over HTTP.
The only control surface is a fixed-place NavigateToPose action plus cancel;
raw velocity commands are deliberately not exposed.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from pathlib import Path
from typing import Any

from PIL import Image as PilImage
import yaml
from simulation import SimulationState, gazebo_stats_loop


ROOT = Path(__file__).resolve().parent
STATIC_DIR = ROOT / "static"
MAP_CONFIG = ROOT / "config" / "map.yaml"
PRESETS_CONFIG = ROOT / "config" / "presets.json"
BATTERY_CONFIG = ROOT / "config" / "battery.json"
ROBOT_ADDRESS = os.environ.get("ROBOT_VIRTUAL_ADDRESS", "")
ACTIVE_NAVIGATION = {"sending", "executing", "canceling", "stop_failed", "unconfirmed"}
MODE = "simulation"


def load_battery_profile() -> dict[str, Any]:
    try:
        return json.loads(BATTERY_CONFIG.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


BATTERY_PROFILE = {}


def estimate_battery_percent(voltage: float | None, profile: dict[str, Any]) -> int | None:
    cells = profile.get("series_cells")
    if (voltage is None or not profile.get("confirmed")
            or type(cells) is not int or not 1 <= cells <= 12
            or profile.get("chemistry") != "li-ion"):
        return None
    # Approximate resting-voltage curve, not coulomb counting or a BMS SOC.
    # Load and temperature affect the reading; the UI explicitly says estimated.
    curve = [(3.3, 0), (3.5, 10), (3.6, 20), (3.7, 35), (3.8, 50),
             (3.9, 65), (4.0, 80), (4.1, 90), (4.2, 100)]
    cell_voltage = voltage / cells
    if cell_voltage <= curve[0][0]:
        return 0
    for (low_v, low_p), (high_v, high_p) in zip(curve, curve[1:]):
        if cell_voltage <= high_v:
            return round(low_p + (cell_voltage - low_v) / (high_v - low_v) * (high_p - low_p))
    return 100


def now_ms() -> int:
    return int(time.time() * 1000)


def yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass
class DashboardState:
    lock: threading.RLock = field(default_factory=threading.RLock)
    started_at: float = field(default_factory=time.monotonic)
    freshness: dict[str, float] = field(default_factory=dict)
    scan_times: list[float] = field(default_factory=list)
    pose: dict[str, Any] | None = None
    velocity: dict[str, float] = field(default_factory=lambda: {"linear": 0.0, "angular": 0.0})
    path: list[dict[str, float]] = field(default_factory=list)
    battery: dict[str, Any] | None = None
    diagnostics: list[dict[str, Any]] = field(default_factory=list)
    network: dict[str, Any] = field(default_factory=lambda: {
        "reachable": False,
        "latency_ms": None,
        "address": ROBOT_ADDRESS,
        "checked_at": None,
    })
    map_meta: dict[str, Any] = field(default_factory=dict)
    map_png: bytes | None = None
    map_revision: int = 0
    camera_jpeg: bytes | None = None
    last_camera_encode: float = 0.0
    presets: dict[str, Any] = field(default_factory=dict)
    preset_goals: dict[str, dict[str, Any]] = field(default_factory=dict)
    navigation: dict[str, Any] = field(default_factory=lambda: {
        "available": False,
        "status": "idle",
        "status_label": "Nav2 未就绪",
        "active_place": None,
        "message": "等待导航服务",
        "distance_remaining": None,
        "eta_seconds": None,
        "recoveries": 0,
        "updated_at": None,
        "goal_id": None,
        "feedback_at": None,
    })

    def __post_init__(self) -> None:
        self._load_fallback_map()
        self._load_presets()

    def _load_fallback_map(self) -> None:
        try:
            raw = yaml.safe_load(MAP_CONFIG.read_text(encoding="utf-8"))
            with PilImage.open(STATIC_DIR / "assets" / "map.png") as image:
                width, height = image.size
            self.map_meta = {
                "width": width,
                "height": height,
                "resolution": float(raw["resolution"]),
                "origin": [float(v) for v in raw["origin"]],
                "source": "cached",
            }
        except Exception:
            self.map_meta = {
                "width": 335,
                "height": 546,
                "resolution": 0.05,
                "origin": [-4.07, -17.3, 0.0],
                "source": "cached",
            }

    def _load_presets(self) -> None:
        try:
            loaded = json.loads(PRESETS_CONFIG.read_text(encoding="utf-8"))
            places = loaded.get("places", [])
            self.preset_goals = {
                str(place["id"]): {
                    "id": str(place["id"]),
                    "name": str(place["name"]),
                    "x": float(place["x"]),
                    "y": float(place["y"]),
                    "yaw": float(place["yaw"]),
                    "verified": bool(place.get("verified", False)),
                    "enabled": bool(place.get("enabled", False)),
                }
                for place in places
            }
            self.presets = {
                "navigation_enabled": bool(loaded.get("navigation_enabled", False)),
                "notice": str(loaded.get("notice", "")),
                "places": [
                    {
                        "id": str(place["id"]),
                        "name": str(place["name"]),
                        "description": str(place.get("description", "")),
                        "icon": str(place.get("icon", "location")),
                        "verified": bool(place.get("verified", False)),
                        "enabled": bool(place.get("enabled", False)),
                    }
                    for place in places
                ],
            }
        except (OSError, ValueError, KeyError, TypeError):
            self.presets = {
                "navigation_enabled": False,
                "notice": "预设地点配置不可用。",
                "places": [],
            }
            self.preset_goals = {}

    def touch(self, source: str) -> None:
        with self.lock:
            self.freshness[source] = time.monotonic()

    def age(self, source: str) -> float | None:
        stamp = self.freshness.get(source)
        return None if stamp is None else time.monotonic() - stamp

    def is_fresh(self, source: str, seconds: float) -> bool:
        age = self.age(source)
        return age is not None and age < seconds

    def sensor(self, key: str, label: str, timeout: float) -> dict[str, Any]:
        age = self.age(key)
        if age is None:
            return {"id": key, "label": label, "status": "unknown", "detail": "等待数据"}
        if age < timeout:
            return {"id": key, "label": label, "status": "ok", "detail": "正常"}
        return {"id": key, "label": label, "status": "error", "detail": f"{int(age)} 秒未更新"}

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            online = any(self.is_fresh(key, 4.0) for key in ("odom", "scan", "camera", "tf"))
            odom_fresh = self.is_fresh("odom", 4.0)
            moving = odom_fresh and (abs(self.velocity["linear"]) > 0.015 or abs(self.velocity["angular"]) > 0.04)
            localized = self.is_fresh("pose", 5.0)
            # A static map is normally published once with transient durability.
            live_map = self.map_meta.get("source") == "live"
            sensors = [
                self.sensor("scan", "激光雷达", 4.0),
                self.sensor("camera", "Astra RGB", 4.0),
                self.sensor("odom", "里程计（估算）", 4.0),
                self.sensor("pose", "定位", 5.0),
            ]
            # A sensor which never published should not remain "waiting" forever
            # after the rest of the robot has demonstrably come online.
            if online and time.monotonic() - self.started_at > 10.0:
                for sensor in sensors:
                    if sensor["status"] == "unknown" and sensor["id"] in {"scan", "camera", "odom"}:
                        sensor.update(status="error", detail="未收到数据")

            alerts: list[dict[str, str]] = []
            if not online:
                alerts.append({"level": "critical", "title": "小车未连接", "message": "未收到实时数据，请检查小车电源和虚拟网络连接。"})
            elif not live_map:
                alerts.append({"level": "warning", "title": "正在使用缓存地图", "message": "尚未获取本次运行的地图，恢复连接后自动更新。"})
            if online and not localized:
                alerts.append({"level": "warning", "title": "定位待恢复", "message": "当前位置尚未确认，地图上的历史位置不能作为实时位置使用。"})
            if online and not self.is_fresh("camera", 4.0):
                alerts.append({"level": "warning", "title": "相机画面中断", "message": "Astra RGB 超过 4 秒未更新。"})
            if online and sensors[0]["status"] == "error":
                alerts.append({"level": "critical", "title": "激光雷达异常", "message": "扫描数据已中断，导航目标发送已阻止。请检查雷达供电与设备状态。"})
            if online and not odom_fresh:
                alerts.append({"level": "critical", "title": "里程计数据中断", "message": "未收到运动估计数据，导航目标发送已阻止。"})
            if self.navigation["status"] in {"unconfirmed", "stop_failed"}:
                alerts.append({"level": "critical" if self.navigation["status"] == "stop_failed" else "warning",
                               "title": self.navigation["status_label"], "message": self.navigation["message"]})
            for item in (self.diagnostics[:4] if self.is_fresh("diagnostics", 15.0) else []):
                # robot_localization reports the original publisher name even
                # though this deployment remaps odometry/filtered to /odom.
                # Suppress only this exact empty-statistics report while actual
                # odometry AND live transforms are independently arriving.
                if (item["name"] == "ekf_filter_node: odometry/filtered topic status"
                        and item["message"] == "No events recorded."
                        and odom_fresh and self.is_fresh("tf", 4.0)):
                    continue
                alerts.append({
                    "level": "critical" if item["level"] >= 2 else "warning",
                    "title": item["name"],
                    "message": item["message"],
                })

            status = "offline"
            status_label = "离线"
            if online and localized and sensors[0]["status"] == "ok" and odom_fresh:
                status = "moving" if moving else "ready"
                status_label = "行驶中" if moving else "已就绪"
            elif online:
                status = "limited"
                status_label = "在线 · 状态待检查" if localized else "在线 · 待定位"

            navigation = json.loads(json.dumps(self.navigation, ensure_ascii=False))
            navigation["feedback_age_seconds"] = round((now_ms() - navigation["feedback_at"]) / 1000.0, 1) if navigation["feedback_at"] else None
            navigation["safe_to_navigate"] = bool(
                navigation["available"] and online and localized and odom_fresh and self.is_fresh("scan", 4.0)
            )
            if not navigation["available"]:
                navigation["blocked_reason"] = "等待 Nav2 导航服务"
            elif not online:
                navigation["blocked_reason"] = "实车离线，已阻止发送目标"
            elif not self.is_fresh("scan", 4.0):
                navigation["blocked_reason"] = "激光雷达无数据，已阻止发送目标"
            elif not localized:
                navigation["blocked_reason"] = "尚未获得 AMCL 定位，已阻止发送目标"
            elif not odom_fresh:
                navigation["blocked_reason"] = "里程计数据已中断，已阻止发送目标"
            else:
                navigation["blocked_reason"] = None

            return {
                "server_time": now_ms(),
                "read_only": False,
                "robot": {
                    "name": "ROSLander 01",
                    "online": online,
                    "status": status,
                    "status_label": status_label,
                    "moving": moving,
                    "pose": self.pose,
                    "localized": localized,
                    "velocity_fresh": odom_fresh,
                    "velocity_source": "command_integration_and_imu",
                    "velocity": dict(self.velocity),
                    "battery": {**self.battery, "fresh": self.is_fresh("battery", 15.0)} if self.battery else None,
                },
                "network": dict(self.network),
                "map": {
                    **self.map_meta,
                    "live": live_map,
                    "revision": self.map_revision,
                    "image_url": "/api/map.png",
                },
                "camera": {
                    "online": self.is_fresh("camera", 4.0),
                    "image_url": "/api/camera.jpg" if self.is_fresh("camera_frame", 4.0) else "/api/camera-stream",
                    "mode": "snapshot" if self.is_fresh("camera_frame", 4.0) else "stream",
                    "transport": "ROS Image" if self.is_fresh("camera_frame", 4.0) else "External MJPEG",
                },
                "sensors": sensors,
                "alerts": alerts,
                "path": list(self.path),
                "presets": json.loads(json.dumps(self.presets, ensure_ascii=False)),
                "navigation": navigation,
                "telemetry": {
                    "scan_hz": round((len(self.scan_times) - 1) / (self.scan_times[-1] - self.scan_times[0]), 1)
                    if len(self.scan_times) > 1 and self.is_fresh("scan", 4.0) and self.scan_times[-1] > self.scan_times[0] else None,
                    "ages": {key: round(age, 1) if (age := self.age(key)) is not None else None
                             for key in ("scan", "camera", "odom", "pose", "battery")},
                },
            }


STATE = SimulationState(ROOT.parent / 'maps/map_01.yaml')


def network_probe_loop(stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        reachable = False
        latency: float | None = None
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "2", ROBOT_ADDRESS],
                capture_output=True,
                text=True,
                timeout=4,
                check=False,
            )
            reachable = result.returncode == 0
            if reachable and "time=" in result.stdout:
                fragment = result.stdout.split("time=", 1)[1].split()[0]
                latency = round(float(fragment), 1)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            pass
        with STATE.lock:
            STATE.network = {
                "reachable": reachable,
                "latency_ms": latency,
                "address": ROBOT_ADDRESS,
                "checked_at": now_ms(),
            }
        stop_event.wait(5.0)


class RosGateway:
    """Optional ROS layer. Importing the HTTP app does not require ROS."""

    def __init__(self, simulation: bool = False) -> None:
        import cv2
        import numpy as np
        import rclpy
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from cv_bridge import CvBridge
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.clock import Clock, ClockType
        from rosgraph_msgs.msg import Clock as ClockMessage
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import UInt16
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener, TransformException
        from unique_identifier_msgs.msg import UUID

        self.simulation = simulation
        self.last_pose_stamp = None
        self.rclpy = rclpy
        self.cv2 = cv2
        self.np = np
        self.bridge = CvBridge()
        self.GoalStatus = GoalStatus
        self.NavigateToPose = NavigateToPose
        self.UUID = UUID
        rclpy.init(args=None)
        self.node = Node("robot_web_dashboard_sim" if simulation else "robot_web_dashboard",
                         parameter_overrides=[Parameter('use_sim_time', value=True)] if simulation else [])
        self.tf_buffer = Buffer(node=self.node) if simulation else Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.TransformException = TransformException
        self.nav_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose") if not simulation or STATE.navigation_enabled else None
        self.nav_lock = threading.RLock()
        self.goal_handle = None
        self.cancel_when_accepted = False
        self.active_request_id = None
        self.active_goal_uuid = None
        self.goal_started_at = 0.0
        self.last_goal_activity = 0.0
        self.cancel_sent_at = None
        self.cancel_sequence = 0
        self.diagnostic_entries: dict[str, tuple[float, dict[str, Any]]] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(depth=10)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.node.create_subscription(Odometry, "/odom", self._odom, sensor_qos)
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._pose, map_qos if simulation else reliable_qos)
        self.node.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        self.node.create_subscription(NavPath, "/plan", self._path, reliable_qos)
        self.node.create_subscription(LaserScan, "/scan", self._scan, sensor_qos)
        self.node.create_subscription(TFMessage, "/tf", lambda _: STATE.touch("tf"), sensor_qos)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics, reliable_qos)
        self.node.create_subscription(Image, "/astra_camera/rgb/image_raw", self._camera, sensor_qos)
        if simulation:
            self.node.create_subscription(ClockMessage, '/clock', self._clock, sensor_qos)
            self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._observe_nav_status, map_qos)
            self.node.create_subscription(NavigateToPose.Impl.FeedbackMessage, '/navigate_to_pose/_action/feedback', self._observe_nav_feedback, reliable_qos)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.node.create_timer(1.0, self._update_nav_availability, clock=self.wall_clock)
            self.node.create_timer(.2, self._update_pose_from_tf, clock=self.wall_clock)
        else:
            self.node.create_subscription(UInt16, "/ros_robot_controller/battery", self._battery, sensor_qos)
            self.node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._navigation_status, map_qos)
            self.node.create_timer(1.0, self._update_nav_availability)
            self.node.create_timer(0.2, self._update_pose_from_tf)

    def _clock(self, msg: Any) -> None:
        if STATE.update_clock(msg.clock.sec + msg.clock.nanosec / 1e9):
            self.tf_buffer.clear()
            self.last_pose_stamp = None

    def _current_stamp(self, stamp: Any) -> bool:
        if not getattr(self, 'simulation', False):
            return True
        if STATE.clock_seconds is None:
            return False
        seconds = stamp.sec + stamp.nanosec / 1e9
        return abs(STATE.clock_seconds - seconds) <= 4.

    def _observe_nav_status(self, msg: Any) -> None:
        STATE.touch('nav_status')
        # The owned Action client handles its result/feedback and matches UUIDs.
        # The observer must not overwrite it with an older or unrelated goal.
        with self.nav_lock:
            if self.active_request_id is not None:
                self._navigation_status(msg)
                if self.active_request_id is None:
                    self._observe_external_nav_status(msg)
                return
            self._observe_external_nav_status(msg)

    def _observe_external_nav_status(self, msg: Any) -> None:
        if not msg.status_list:
            return
        pending = [s for s in msg.status_list if s.status in (1, 2, 3)]
        latest = max(pending or msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        if STATE.clock_seconds is not None and latest.goal_info.stamp.sec + latest.goal_info.stamp.nanosec / 1e9 > STATE.clock_seconds + 4.:
            return
        goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
        labels = {1: ('sending','目标已接收'), 2: ('executing','正在导航'), 3: ('canceling','正在取消'),
                  4: ('succeeded','已经到达'), 5: ('canceled','导航已取消'), 6: ('failed','导航失败')}
        status, label = labels.get(latest.status, ('idle','等待任务'))
        with STATE.lock:
            if STATE.navigation.get('source') == 'web' and STATE.navigation.get('goal_id') == goal_id:
                return  # Do not resurrect a completed owned goal with late status.
            if STATE.observed_goal_uuid != goal_id:
                STATE.navigation.update(distance_remaining=None, eta_seconds=None, recoveries=0, feedback_at=None)
            STATE.observed_goal_uuid = goal_id
            STATE.navigation.update(status=status, status_label=label, goal_id=goal_id,
                                    message='观察外部 Nav2 任务 · 等待结束后可选择预设点',
                                    source='external', can_cancel=False, active_place=None, updated_at=now_ms())
            if latest.status in (4,5,6):
                STATE.navigation.update(distance_remaining=None, eta_seconds=None)
            STATE.touch('nav_status')

    def _observe_nav_feedback(self, wrapped: Any) -> None:
        with STATE.lock:
            if STATE.navigation.get('source') == 'web':
                return
            if bytes(wrapped.goal_id.uuid).hex() != STATE.observed_goal_uuid or STATE.navigation['status'] not in ACTIVE_NAVIGATION:
                return
            feedback = wrapped.feedback
            STATE.navigation.update(distance_remaining=float(feedback.distance_remaining),
                                    eta_seconds=self._duration_seconds(feedback.estimated_time_remaining),
                                    recoveries=int(feedback.number_of_recoveries), feedback_at=now_ms())
            STATE.touch('nav_feedback')

    def _scan(self, msg: Any) -> None:
        if not self._current_stamp(msg.header.stamp):
            return
        with STATE.lock:
            stamp = time.monotonic()
            STATE.scan_times = [t for t in STATE.scan_times[-119:] if stamp - t < 12.0]
            STATE.scan_times.append(stamp)
            STATE.touch("scan")

    def _update_pose_from_tf(self) -> None:
        if getattr(self, 'simulation', False) and STATE.simulation_status()['state'] != 'running':
            return
        # AMCL may only publish a pose after motion; live TF remains available
        # while stationary and gives map-frame position between AMCL updates.
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_footprint", self.rclpy.time.Time())
        except self.TransformException:
            return
        age = (self.node.get_clock().now() - self.rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        if abs(age) > 4.0:
            return
        if getattr(self, 'simulation', False):
            stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
            if stamp == self.last_pose_stamp:
                return
            self.last_pose_stamp = stamp
        with STATE.lock:
            confidence = STATE.pose.get("confidence") if STATE.pose else None
            STATE.pose = {
                "x": round(transform.transform.translation.x, 3),
                "y": round(transform.transform.translation.y, 3),
                "yaw": round(yaw_from_quaternion(transform.transform.rotation), 4),
                "confidence": confidence,
            }
            STATE.touch("pose")

    def _set_navigation(self, **changes: Any) -> None:
        if getattr(self, 'simulation', False) and 'message' in changes:
            changes['message'] = changes['message'].replace('实车', '本机仿真')
        with STATE.lock:
            STATE.navigation.update(changes)
            STATE.navigation["updated_at"] = now_ms()

    def _update_nav_availability(self) -> None:
        if getattr(self, 'simulation', False):
            services = dict(self.node.get_service_names_and_types())
            with STATE.lock:
                STATE.navigation['available'] = (self.nav_client.server_is_ready() if self.nav_client is not None
                    else '/navigate_to_pose/_action/get_result' in services)
                STATE.camera_expected = bool(self.node.get_publishers_info_by_topic('/astra_camera/rgb/image_raw'))
                if STATE.navigation['status'] == 'idle':
                    STATE.navigation['status_label'] = 'Nav2 已就绪' if STATE.navigation['available'] else '等待 Nav2'
            self._check_navigation_watchdog()
            return
        ready = self.nav_client.server_is_ready()
        with STATE.lock:
            STATE.navigation["available"] = ready
            if STATE.navigation["status"] == "idle":
                STATE.navigation["status_label"] = "Nav2 已就绪" if ready else "Nav2 未就绪"
                STATE.navigation["message"] = "可以发送预设目标" if ready else "等待导航服务"
        self._check_navigation_watchdog()

    def _finish_navigation(self, request_id: str, status: int) -> bool:
        labels = {
            4: ("succeeded", "已经到达", "实车已确认任务完成，可以再次选择地点"),
            5: ("canceled", "已停止导航", "实车已确认取消，可以再次选择地点"),
            6: ("failed", "导航失败", "实车已中止任务，可以检查后重新选择地点"),
            0: ("rejected", "任务被拒绝", "导航服务未接受目标，可以重新选择地点"),
        }
        if status not in labels:
            return False
        with self.nav_lock:
            if request_id != self.active_request_id:
                return False
            self.active_request_id = None
            self.active_goal_uuid = None
            self.goal_handle = None
            self.cancel_when_accepted = False
            self.cancel_sent_at = None
            state, label, message = labels[status]
            if getattr(self, 'simulation', False):
                message = message.replace('实车', '本机仿真')
            self._set_navigation(status=state, status_label=label, message=message,
                                 active_place=STATE.navigation['active_place'] if getattr(self, 'simulation', False) else None,
                                 can_cancel=False, distance_remaining=None, eta_seconds=None)
        return True

    def _navigation_status(self, msg: Any) -> None:
        with self.nav_lock:
            request_id = self.active_request_id
            goal_uuid = self.active_goal_uuid
            if request_id is None:
                return
            for item in msg.status_list:
                if bytes(item.goal_info.goal_id.uuid) == goal_uuid and item.status in {4, 5, 6}:
                    self._finish_navigation(request_id, int(item.status))
                    return

    def _check_navigation_watchdog(self) -> None:
        with self.nav_lock:
            request_id = self.active_request_id
            if request_id is None:
                return
            if self.goal_handle is not None and int(self.goal_handle.status) in {4, 5, 6} and self._finish_navigation(request_id, int(self.goal_handle.status)):
                return
            now = time.monotonic()
            with STATE.lock:
                status = STATE.navigation["status"]
            if status == "canceling" and self.cancel_sent_at is not None and now - self.cancel_sent_at > 8.0:
                self._set_navigation(status="stop_failed", status_label="停止尚未确认",
                                     message="停止应答已超时，可重试停止；确认任务结束前暂不接收新目标", eta_seconds=None)
            elif status == "sending" and now - self.goal_started_at > 8.0:
                self._set_navigation(status="unconfirmed", status_label="任务状态待确认",
                                     message="目标应答超时，正在核对实车状态；请勿重复发送", eta_seconds=None)
            elif status == "executing" and now - self.last_goal_activity > 10.0 and not (
                    getattr(self, 'simulation', False) and STATE.simulation_status()['paused']):
                self._set_navigation(status="unconfirmed", status_label="导航反馈已中断",
                                     message="仍在核对实车任务，可尝试停止；确认结束后自动恢复地点按钮",
                                     distance_remaining=None, eta_seconds=None)

    def start_navigation(self, place_id: str, acknowledge_unverified: bool) -> tuple[int, dict[str, Any]]:
        simulation = getattr(self, 'simulation', False)
        if simulation and not STATE.navigation_enabled:
            return HTTPStatus.FORBIDDEN, {'error':'read_only','message':'仿真监控模式未开放网页导航控制'}
        with STATE.lock:
            navigation_enabled = bool(STATE.presets.get("navigation_enabled", False))
            place = dict(STATE.preset_goals.get(place_id, {}))
            active = STATE.navigation["status"] in ACTIVE_NAVIGATION
            online = any(STATE.is_fresh(key, 4.0) for key in ("odom", "scan", "camera", "tf"))
            scan_ready = STATE.is_fresh("scan", 4.0)
            localized = STATE.is_fresh("pose", 5.0)
            odom_ready = STATE.is_fresh("odom", 4.0)
        if not navigation_enabled:
            return HTTPStatus.CONFLICT, {"error": "navigation_disabled", "message": "导航功能未启用"}
        if not place or not place.get("enabled"):
            return HTTPStatus.NOT_FOUND, {"error": "unknown_place", "message": "地点不存在或未启用"}
        if not place.get("verified") and not acknowledge_unverified:
            return HTTPStatus.CONFLICT, {"error": "confirmation_required", "message": "测试坐标尚未标定，必须明确确认"}
        if active:
            return HTTPStatus.CONFLICT, {"error": "navigation_active", "message": "已有导航任务正在执行"}
        if simulation:
            with STATE.lock:
                reason = STATE.navigation_blocked_reason()
            if reason:
                return HTTPStatus.CONFLICT, {'error': 'simulation_not_ready', 'message': reason}
        if not self.nav_client.server_is_ready():
            return HTTPStatus.SERVICE_UNAVAILABLE, {"error": "nav2_unavailable", "message": "Nav2 尚未就绪"}
        if not online:
            return HTTPStatus.CONFLICT, {"error": "robot_offline", "message": "实车离线，不能发送导航目标"}
        if not scan_ready:
            return HTTPStatus.CONFLICT, {"error": "scan_unavailable", "message": "激光雷达无数据，已阻止发送导航目标"}
        if not localized:
            return HTTPStatus.CONFLICT, {"error": "localization_unavailable", "message": "尚未获得 AMCL 定位，已阻止发送导航目标"}
        if not odom_ready:
            return HTTPStatus.CONFLICT, {"error": "odometry_unavailable", "message": "里程计数据已中断，已阻止发送导航目标"}

        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = place["x"]
        goal.pose.pose.position.y = place["y"]
        goal.pose.pose.orientation.z = math.sin(place["yaw"] / 2.0)
        goal.pose.pose.orientation.w = math.cos(place["yaw"] / 2.0)

        with self.nav_lock:
            with STATE.lock:
                if self.active_request_id is not None or STATE.navigation["status"] in ACTIVE_NAVIGATION:
                    return HTTPStatus.CONFLICT, {"error": "navigation_active", "message": "当前任务尚未结束，请先停止或等待状态确认"}
            goal_uuid = uuid.uuid4()
            request_id = goal_uuid.hex
            self.active_request_id = request_id
            self.active_goal_uuid = goal_uuid.bytes
            self.goal_started_at = time.monotonic()
            self.last_goal_activity = self.goal_started_at
            self.cancel_sent_at = None
            self.goal_handle = None
            self.cancel_when_accepted = False
            self._set_navigation(
                status="sending", status_label="正在发送目标",
                active_place={"id": place["id"], "name": place["name"]},
                message="等待 Nav2 接收任务", distance_remaining=None,
                eta_seconds=None, recoveries=0, goal_id=request_id, feedback_at=None,
                source='web', can_cancel=True)
        try:
            future = self.nav_client.send_goal_async(
                goal, goal_uuid=self.UUID(uuid=list(goal_uuid.bytes)),
                feedback_callback=lambda wrapped: self._navigation_feedback(wrapped, request_id))
            future.add_done_callback(lambda completed: self._goal_response(completed, request_id))
        except Exception as exc:
            with self.nav_lock:
                if request_id == self.active_request_id:
                    self._set_navigation(status="unconfirmed", status_label="目标发送待确认", message=str(exc))
            return HTTPStatus.SERVICE_UNAVAILABLE, {"error": "send_unconfirmed", "message": "目标发送结果待确认，请勿重复发送"}
        return HTTPStatus.ACCEPTED, {"ok": True, "message": "目标已发送，等待 Nav2 接收"}

    def _goal_response(self, future: Any, request_id: str) -> None:
        with self.nav_lock:
            if request_id != self.active_request_id:
                return
            try:
                goal_handle = future.result()
            except Exception as exc:
                self._set_navigation(status="unconfirmed", status_label="目标接收待确认", message=str(exc))
                return
            if goal_handle is None or not goal_handle.accepted:
                self._finish_navigation(request_id, 0)
                return
            self.goal_handle = goal_handle
            cancel_now = self.cancel_when_accepted
            self.last_goal_activity = time.monotonic()
            self._set_navigation(status="executing", status_label="正在导航", message="目标已接收，正在规划路线")
            goal_handle.get_result_async().add_done_callback(lambda completed: self._navigation_result(completed, request_id))
            if cancel_now:
                self.cancel_navigation()

    @staticmethod
    def _duration_seconds(duration: Any) -> float | None:
        if duration is None:
            return None
        return round(float(duration.sec) + float(duration.nanosec) / 1_000_000_000.0, 1)

    def _navigation_feedback(self, wrapped: Any, request_id: str) -> None:
        with self.nav_lock:
            if request_id != self.active_request_id or self.goal_handle is None:
                return
            if hasattr(wrapped, "goal_id") and bytes(wrapped.goal_id.uuid) != self.active_goal_uuid:
                return
            feedback = wrapped.feedback
            if getattr(self, 'simulation', False):
                STATE.touch('nav_feedback')
            self.last_goal_activity = time.monotonic()
            with STATE.lock:
                current_status = STATE.navigation["status"]
                if current_status not in ACTIVE_NAVIGATION:
                    return
            stopping = current_status in {"canceling", "stop_failed"}
            self._set_navigation(
                status=current_status if stopping else "executing",
                status_label="停止尚未确认" if current_status == "stop_failed" else "正在停止" if stopping else "正在导航",
                message="请重试停止并检查现场状态" if current_status == "stop_failed" else "正在等待导航任务停止结果" if stopping else "小车正在前往目标位置",
                distance_remaining=round(float(feedback.distance_remaining), 2),
                eta_seconds=self._duration_seconds(feedback.estimated_time_remaining),
                recoveries=int(feedback.number_of_recoveries), feedback_at=now_ms())

    def _navigation_result(self, future: Any, request_id: str) -> None:
        with self.nav_lock:
            if request_id != self.active_request_id:
                return
            try:
                status = int(future.result().status)
            except Exception as exc:
                self._set_navigation(status="unconfirmed", status_label="任务结果待确认", message=f"正在通过实车状态核对结果：{exc}")
                return
            if status not in {4, 5, 6} or not self._finish_navigation(request_id, status):
                self._set_navigation(status="unconfirmed", status_label="任务结果待确认", message="未收到确定的结束状态，正在核对实车")

    def cancel_navigation(self) -> tuple[int, dict[str, Any]]:
        if getattr(self, 'simulation', False) and not STATE.navigation_enabled:
            return HTTPStatus.FORBIDDEN, {'error':'read_only','message':'仿真监控模式未开放网页导航控制'}
        with self.nav_lock:
            with STATE.lock:
                status = STATE.navigation["status"]
            request_id = self.active_request_id
            goal_handle = self.goal_handle
            if request_id is None or status not in ACTIVE_NAVIGATION:
                return HTTPStatus.CONFLICT, {"error": "no_active_goal", "message": "当前没有网页发起的导航任务"}
            self.cancel_sent_at = time.monotonic()
            if goal_handle is None:
                self.cancel_when_accepted = True
                self._set_navigation(status="canceling", status_label="正在停止", message="目标接收后将立即取消")
                return HTTPStatus.ACCEPTED, {"ok": True, "message": "停止请求已登记"}
            self.cancel_sequence += 1
            sequence = self.cancel_sequence
            self._set_navigation(status="canceling", status_label="正在停止", message="正在请求 Nav2 取消任务")
            try:
                goal_handle.cancel_goal_async().add_done_callback(lambda completed: self._cancel_response(completed, request_id, sequence))
            except Exception as exc:
                self._set_navigation(status="stop_failed", status_label="停止尚未确认", message=str(exc))
        return HTTPStatus.ACCEPTED, {"ok": True, "message": "停止请求已发送"}

    def _cancel_response(self, future: Any, request_id: str, sequence: int) -> None:
        with self.nav_lock:
            if request_id != self.active_request_id or sequence != self.cancel_sequence:
                return
            try:
                response = future.result()
                accepted = any(bytes(item.goal_id.uuid) == self.active_goal_uuid for item in response.goals_canceling)
            except Exception as exc:
                self._set_navigation(status="stop_failed", status_label="停止尚未确认", message=f"请重试停止：{exc}")
                return
            if accepted:
                self._set_navigation(status="canceling", status_label="正在停止", message="Nav2 已接受取消请求，等待结束确认")
            else:
                if self.goal_handle is not None and int(self.goal_handle.status) in {4, 5, 6} and self._finish_navigation(request_id, int(self.goal_handle.status)):
                    return
                self._set_navigation(status="stop_failed", status_label="停止尚未确认", message="导航服务未接受取消请求，正在核对任务是否已结束；可重试停止")

    def _odom(self, msg: Any) -> None:
        if not self._current_stamp(msg.header.stamp):
            return
        with STATE.lock:
            STATE.velocity = {
                "linear": round(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y), 3),
                "angular": round(msg.twist.twist.angular.z, 3),
            }
        STATE.touch("odom")

    def _pose(self, msg: Any) -> None:
        if not self._current_stamp(msg.header.stamp):
            return
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        with STATE.lock:
            STATE.pose = {
                "x": round(pose.position.x, 3),
                "y": round(pose.position.y, 3),
                "yaw": round(yaw_from_quaternion(pose.orientation), 4),
                "confidence": round(clamp(100.0 - math.sqrt(max(0.0, covariance[0] + covariance[7])) * 100.0, 0.0, 100.0)),
            }
        STATE.touch("pose")

    def _map(self, msg: Any) -> None:
        values = self.np.asarray(msg.data, dtype=self.np.int16).reshape((msg.info.height, msg.info.width))
        pixels = self.np.full(values.shape, 207, dtype=self.np.uint8)
        pixels[values == 0] = 250
        pixels[values >= 65] = 24
        pixels[(values > 0) & (values < 65)] = 132
        pixels = self.np.flipud(pixels)
        output = BytesIO()
        PilImage.fromarray(pixels, mode="L").save(output, format="PNG", optimize=True)
        origin = msg.info.origin
        with STATE.lock:
            STATE.map_png = output.getvalue()
            STATE.map_revision += 1
            STATE.map_meta = {
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin": [float(origin.position.x), float(origin.position.y), yaw_from_quaternion(origin.orientation)],
                "source": "live",
            }
            if getattr(self, 'simulation', False):
                STATE.map_matches_navigation = STATE.matches_navigation_map(STATE.map_meta, msg.data, msg.header.frame_id)
        STATE.touch("map")

    def _path(self, msg: Any) -> None:
        if not self._current_stamp(msg.header.stamp):
            return
        poses = msg.poses
        step = max(1, len(poses) // 180)
        with STATE.lock:
            STATE.path = [
                {"x": round(p.pose.position.x, 3), "y": round(p.pose.position.y, 3)}
                for p in poses[::step]
            ]
        STATE.touch("path")

    def _diagnostics(self, msg: Any) -> None:
        now = time.monotonic()
        for item in msg.status:
            name = item.name or "设备异常"
            level = int.from_bytes(item.level, "little") if isinstance(item.level, bytes) else int(item.level)
            if level <= 0:
                self.diagnostic_entries.pop(name, None)
                continue
            # This camera driver reports unsupported optional temperature telemetry
            # as an error although RGB streaming remains healthy.
            if "Property is not supported" in item.message:
                self.diagnostic_entries.pop(name, None)
                continue
            self.diagnostic_entries[name] = (now, {"level": level, "name": name, "message": item.message or "未提供详细原因"})
        self.diagnostic_entries = {key: value for key, value in self.diagnostic_entries.items() if now - value[0] < 15.0}
        with STATE.lock:
            STATE.diagnostics = sorted((value[1] for value in self.diagnostic_entries.values()), key=lambda item: -item["level"])
        STATE.touch("diagnostics")

    def _camera(self, msg: Any) -> None:
        if not self._current_stamp(msg.header.stamp):
            return
        monotonic = time.monotonic()
        if monotonic - STATE.last_camera_encode < 0.22:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width = image.shape[:2]
            if width > 960:
                image = self.cv2.resize(image, (960, round(height * 960 / width)))
            ok, encoded = self.cv2.imencode(".jpg", image, [self.cv2.IMWRITE_JPEG_QUALITY, 78])
            if ok:
                with STATE.lock:
                    STATE.camera_jpeg = encoded.tobytes()
                    STATE.last_camera_encode = monotonic
                STATE.touch("camera")
                STATE.touch("camera_frame")
        except Exception as exc:
            self.node.get_logger().warning(f"camera frame conversion failed: {exc}")

    def _battery(self, msg: Any) -> None:
        raw = int(msg.data)
        voltage = raw / 1000.0 if raw > 1000 else None
        percent = estimate_battery_percent(voltage, BATTERY_PROFILE)
        with STATE.lock:
            STATE.battery = {"raw": raw, "voltage": voltage, "percent": percent,
                             "estimated": percent is not None, "source": "robot_controller",
                             "profile_confirmed": bool(BATTERY_PROFILE.get("confirmed"))}
        STATE.touch("battery")

    def spin(self) -> None:
        try:
            self.rclpy.spin(self.node)
        except Exception as exc:
            if exc.__class__.__name__ != "ExternalShutdownException":
                raise

    def close(self) -> None:
        if self.rclpy.ok():
            self.rclpy.shutdown()
        spin_thread = getattr(self, 'spin_thread', None)
        if spin_thread is not None and spin_thread is not threading.current_thread():
            spin_thread.join(timeout=5)
        try:
            self.node.destroy_node()
        except Exception:
            pass


ROS_GATEWAY: RosGateway | None = None


class DashboardHandler(SimpleHTTPRequestHandler):
    server_version = "RobotDashboard/0.1"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, directory=str(STATIC_DIR), **kwargs)

    def log_message(self, fmt: str, *args: Any) -> None:
        if os.environ.get("DASHBOARD_HTTP_LOG") == "1":
            super().log_message(fmt, *args)

    def _send_bytes(self, data: bytes, content_type: str, cache: str = "no-store") -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", cache)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, status: int, value: dict[str, Any]) -> None:
        data = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length < 1 or length > 4096:
            raise ValueError("请求内容长度无效")
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("请求必须是 JSON 对象")
        return value

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/api/state":
            payload = json.dumps(STATE.snapshot(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            self._send_bytes(payload, "application/json; charset=utf-8")
            return
        if path == "/api/map.png":
            with STATE.lock:
                dynamic = STATE.map_png
            data = dynamic if dynamic is not None else (STATIC_DIR / "assets" / "map.png").read_bytes()
            self._send_bytes(data, "image/png", "no-cache")
            return
        if path == "/api/camera.jpg":
            if MODE == 'simulation':
                camera = STATE.snapshot()['camera']
                if not camera['online'] and not camera['held']:
                    self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {'error':'frame_unavailable','message':'当前无有效仿真图像'})
                    return
            with STATE.lock:
                data = STATE.camera_jpeg
            if data is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "Camera frame is not available")
                return
            self._send_bytes(data, "image/jpeg")
            return
        if path == "/api/camera-stream":
            if MODE == 'simulation':
                self._send_json(HTTPStatus.NOT_FOUND, {'error':'not_available','message':'仿真模式不连接实车图像转发服务'})
                return
            remote_url = f"http://{ROBOT_ADDRESS}:8080/stream?topic=/astra_camera/rgb/image_raw"
            try:
                upstream = urllib.request.urlopen(remote_url, timeout=5)
                content_type = upstream.headers.get(
                    "Content-Type", "multipart/x-mixed-replace;boundary=boundarydonotcross")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.end_headers()
                while True:
                    chunk = upstream.read(65536)
                    if not chunk:
                        break
                    STATE.touch("camera")
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (urllib.error.URLError, TimeoutError, BrokenPipeError, ConnectionResetError, socket.timeout):
                return
            finally:
                if "upstream" in locals():
                    upstream.close()
            return
        if path == "/api/health":
            self._send_json(HTTPStatus.OK, {'ok':True,'mode':MODE,'ros_connected':ROS_GATEWAY is not None,
                                          'read_only':MODE == 'simulation' and not STATE.navigation_enabled})
            return
        super().do_GET()

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        if MODE == 'simulation' and (not STATE.navigation_enabled or path not in {'/api/navigation/start', '/api/navigation/cancel'}):
            self._send_json(HTTPStatus.FORBIDDEN, {'error':'read_only','message':'仿真监控模式未开放网页控制接口'})
            return
        if ROS_GATEWAY is None:
            self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "ros_unavailable", "message": "ROS 网关未启动"})
            return
        try:
            body = self._read_json()
        except (ValueError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid_request", "message": str(exc)})
            return
        if path == "/api/navigation/start":
            if MODE == 'simulation' and (set(body) != {'place_id'} or not isinstance(body['place_id'], str)):
                self._send_json(HTTPStatus.BAD_REQUEST, {'error': 'invalid_request', 'message': '仿真导航只接受预设 place_id，不接受自定义坐标'})
                return
            place_id = str(body.get("place_id", ""))
            status, response = ROS_GATEWAY.start_navigation(
                place_id, bool(body.get("acknowledge_unverified", False)))
            self._send_json(status, response)
            return
        if path == "/api/navigation/cancel":
            if MODE == 'simulation' and body:
                self._send_json(HTTPStatus.BAD_REQUEST, {'error': 'invalid_request', 'message': '只能取消当前网页任务'})
                return
            status, response = ROS_GATEWAY.cancel_navigation()
            self._send_json(status, response)
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not_found", "message": "接口不存在"})


def main() -> int:
    global ROS_GATEWAY, STATE, MODE, BATTERY_PROFILE
    parser = argparse.ArgumentParser(description="ROSLander ROS 2 web dashboard")
    parser.add_argument("--host", default=os.environ.get("DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("DASHBOARD_PORT", "8088")))
    parser.add_argument("--no-ros", action="store_true", help="serve the offline UI without joining ROS")
    parser.add_argument('--mode', choices=('simulation',), default='simulation')
    args = parser.parse_args()
    MODE = args.mode
    if MODE == 'simulation':
        STATE = SimulationState(ROOT.parent / 'maps/map_01.yaml', navigation_enabled=True)
    else:
        BATTERY_PROFILE = load_battery_profile()
        STATE = DashboardState()

    stop_event = threading.Event()
    network_thread = threading.Thread(target=gazebo_stats_loop if MODE == 'simulation' else network_probe_loop,
                                     args=(STATE, stop_event) if MODE == 'simulation' else (stop_event,), daemon=True)
    network_thread.start()

    gateway = None
    if not args.no_ros:
        try:
            gateway = RosGateway(simulation=MODE == 'simulation')
            ROS_GATEWAY = gateway
            gateway.spin_thread = threading.Thread(target=gateway.spin, name="ros-spin")
            gateway.spin_thread.start()
        except Exception as exc:
            print(f"[dashboard] ROS gateway unavailable; serving offline UI: {exc}", flush=True)

    server = ThreadingHTTPServer((args.host, args.port), DashboardHandler)
    print(f"[dashboard] UI: http://127.0.0.1:{args.port}", flush=True)
    print(f"[dashboard] mode: {MODE}; ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}; fixed-preset navigation enabled", flush=True)
    def request_shutdown(signum, frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        stop_event.set()
        network_thread.join(timeout=4)
        if gateway is not None:
            gateway.close()
        ROS_GATEWAY = None
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
