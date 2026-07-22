#!/usr/bin/env python3
import argparse
import base64
import binascii
import concurrent.futures
import gzip
import heapq
import ipaddress
import json
import math
import os
import platform
import re
import shlex
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import uuid
import zlib
from collections import deque
from dataclasses import dataclass
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple
from urllib.parse import unquote, urlparse


ROOT = Path(__file__).resolve().parent
WEB_ROOT = ROOT / "web"
DATA_ROOT = ROOT / "data"
MAP_DATA_ROOT = DATA_ROOT / "maps"
CONFIG_ROOT = ROOT / "config"
SETTINGS_PATH = CONFIG_ROOT / "dashboard_state.json"
RUN_LOG_ROOT = ROOT / "run_logs"
APP_VERSION = "2026-07-22.97"
ROBOT_SSH_OPERATION_LOCK = threading.RLock()
ROBOT_SSH_OPERATION_LOCKS: Dict[str, threading.Lock] = {}
FALLBACK_SENSOR_STARTUP_WAIT = 2.5
FALLBACK_SENSOR_PENDING_WAIT = 15.0
FALLBACK_RECOVERY_STOP_SECONDS = 0.25
FALLBACK_RECOVERY_TURN_SECONDS = 1.2
FALLBACK_RECOVERY_REVERSE_SECONDS = 0.6
FALLBACK_RECOVERY_FORWARD_SECONDS = 0.5
FALLBACK_RECOVERY_TURN_SPEED = 0.35
FALLBACK_RECOVERY_REVERSE_SPEED = 0.02
FALLBACK_RECOVERY_FORWARD_SPEED = 0.025
FALLBACK_RECOVERY_MAX_ATTEMPTS = 5
FALLBACK_RECOVERY_ESCAPE_MIN_IMPROVEMENT = 0.002
FALLBACK_RECOVERY_ESCAPE_MAX_WORSENING = 0.003
FALLBACK_RECOVERY_TRAPPED_RETRY_SECONDS = 0.75
FALLBACK_LIDAR_COAST_SECONDS = 0.8
FALLBACK_LIDAR_COAST_MIN_CLEARANCE = 0.12
FALLBACK_LIDAR_COAST_MAX_ANGULAR = 0.25
FALLBACK_REPLAN_RECOVERY_SECONDS = 3.0
FALLBACK_REPLAN_RECOVERY_ATTEMPTS = 5
FALLBACK_DYNAMIC_REPLAN_LOOKAHEAD = 0.20
FALLBACK_DYNAMIC_REPLAN_COOLDOWN = 1.5
ASTAR_SOFT_CELL_MULTIPLIER = 4.0
DYNAMIC_LIDAR_OBSTACLE_CELL_SIZE = 0.08
DYNAMIC_LIDAR_OBSTACLE_MIN_POINTS = 5
DYNAMIC_LIDAR_OBSTACLE_HOLD_SECONDS = 1.0
DYNAMIC_LIDAR_OBSTACLE_MIN_RANGE = 0.18
DYNAMIC_LIDAR_OBSTACLE_MAX_RANGE = 2.5
DYNAMIC_LIDAR_STATIC_WALL_TOLERANCE = 0.02
NANOSECONDS_PER_SECOND = 1_000_000_000
ROBOT_CLOCK_FRESH_SECONDS = 2.0
MAX_JSON_BODY_BYTES = 24 * 1024 * 1024
MAX_MAP_UPLOAD_BYTES = 18 * 1024 * 1024
TWIST_TYPE = "geometry_msgs/msg/Twist"
TWIST_STAMPED_TYPE = "geometry_msgs/msg/TwistStamped"
DEFAULT_TURTLEBOT_LENGTH = 0.18
DEFAULT_TURTLEBOT_WIDTH = 0.14
CAMERA_FPS_WINDOW_SECONDS = 5.0
CAMERA_FPS_MIN_WINDOW_SECONDS = 1.0
CAMERA_FPS_UPDATE_SECONDS = 0.5
CAMERA_FPS_SMOOTHING = 0.25
CAMERA_STREAM_MAX_FPS = 30.0
CAMERA_STREAM_BUFFER_FRAMES = 8


def normalized_camera_stream_mode(value: Any) -> str:
    return "raw" if str(value or "").lower() == "raw" else "compressed"


def camera_frame_rate(timestamps: list[float]) -> Optional[float]:
    if len(timestamps) < 2:
        return None
    elapsed = timestamps[-1] - timestamps[0]
    if elapsed <= 1e-6:
        return None
    return round((len(timestamps) - 1) / elapsed, 1)


def topics_for_namespace(namespace: str, camera_style: str = "color") -> Dict[str, str]:
    ns = namespace.rstrip("/")
    if camera_style == "plain":
        camera = f"{ns}/camera/image_raw"
        compressed_camera = f"{ns}/camera/image_raw/compressed"
    elif camera_style in ("camera_ros", "camera-ros"):
        camera = f"{ns}/camera/camera/image_raw"
        compressed_camera = f"{ns}/camera/camera/image_raw/compressed"
    elif camera_style == "realsense":
        camera = f"{ns}/camera/camera/color/image_raw"
        compressed_camera = f"{ns}/camera/camera/color/image_raw/compressed"
    else:
        camera = f"{ns}/camera/color/image_raw"
        compressed_camera = f"{ns}/camera/color/image_raw/compressed"
    return {
        "scan": f"{ns}/scan",
        "pose": f"{ns}/amcl_pose",
        "odom": f"{ns}/odom",
        "camera": camera,
        "compressedCamera": compressed_camera,
        "initialPose": f"{ns}/initialpose",
        "goalAction": f"{ns}/navigate_to_pose",
        "routeAction": f"{ns}/navigate_through_poses",
        "goalTopic": f"{ns}/goal_pose",
        "cmdVel": f"{ns}/cmd_vel",
        "mapFrame": "map",
        "baseFrame": "base_link",
    }


DEFAULT_ROBOT_PROFILES = {
    "tb3_2": {
        "label": "TurtleBot 2",
        "namespace": "/",
        "body": {"length": DEFAULT_TURTLEBOT_LENGTH, "width": DEFAULT_TURTLEBOT_WIDTH},
        "mapPose": {"enabled": False, "x": None, "y": None, "yaw": 0.0},
        "connection": {
            "robotIp": "192.168.20.7",
            "rosDomainId": os.environ.get("ROS_DOMAIN_ID", "1"),
            "rosLocalhostOnly": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
            "robotSshHost": "192.168.20.7",
            "robotSshUser": "kim",
            "robotSshPassword": "1234",
        },
        "topics": topics_for_namespace("/", "plain"),
    },
}


DEFAULT_MAP_LIBRARY = [
    {
        "id": "default-map",
        "name": "기본 맵",
        "imageUrl": "/data/Sprite-1.png",
        "resolution": 0.001,
        "originX": 0.0,
        "originY": 0.0,
        "originYaw": 0.0,
        "widthPixels": 1800,
        "heightPixels": 1800,
    }
]


DEFAULT_STATE: Dict[str, Any] = {
    "setup": {
        "map": {
            "id": "default-map",
            "name": "기본 맵",
            "imageUrl": "/data/Sprite-1.png",
            "resolution": 0.001,
            "originX": 0.0,
            "originY": 0.0,
            "originYaw": 0.0,
            "widthPixels": 1800,
            "heightPixels": 1800,
        },
        "mapLibrary": DEFAULT_MAP_LIBRARY,
        "initialPose": {"x": 0.188, "y": 0.224, "yaw": 1.571},
        "robot": {
            "length": DEFAULT_TURTLEBOT_LENGTH,
            "width": DEFAULT_TURTLEBOT_WIDTH,
        },
        "accessory": {"front": 0.0, "back": 0.0, "left": 0.0, "right": 0.0, "height": 0.0},
        "safety": {"margin": 0.0},
        "object": {"width": 0.13, "height": 0.13, "inflation": 0.0},
        "obstacles": [
            {
                "id": "obs-1783492514881-232",
                "x": 1.027,
                "y": 0.449,
                "width": 0.13,
                "height": 0.13,
            },
            {
                "id": "obs-1783492528497-285",
                "x": 1.218,
                "y": 1.342,
                "width": 0.13,
                "height": 0.10,
            },
        ],
        "planner": {
            "cellSize": 0.02,
            "hardClearance": 0.05,
            "showGrid": True,
            "showInflation": True,
            "showLidarPoints": True,
            "detectLidarObstacles": True,
            "detectBlackWalls": True,
            "blockedCells": [],
            "freeCells": [],
        },
        "fallbackNavigation": {
            "enabled": True,
            "maxLinear": 0.08,
            "minLinear": 0.04,
            "maxAngular": 0.6,
            "lookahead": 0.12,
            "goalTolerance": 0.04,
            "yawTolerance": 0.12,
            "softDistance": 0.10,
            "hardMargin": 0.03,
            "scanTimeout": 0.6,
            "odomTimeout": 0.6,
            "collisionHorizon": 1.5,
        },
        "network": {
            "robotIp": "192.168.20.7",
            "serverIp": "",
            "rosDomainId": os.environ.get("ROS_DOMAIN_ID", "1"),
            "rosLocalhostOnly": os.environ.get("ROS_LOCALHOST_ONLY", "0"),
            "robotSshHost": "192.168.20.7",
            "robotSshUser": "kim",
            "robotSshPassword": "1234",
        },
        "activeRobot": "tb3_2",
        "robotProfiles": DEFAULT_ROBOT_PROFILES,
        "topics": DEFAULT_ROBOT_PROFILES["tb3_2"]["topics"],
    },
    "runtime": {
        "rosConnected": False,
        "mode": "offline",
        "rosBridgeError": None,
        "appVersion": APP_VERSION,
        "pose": {"x": 0.188, "y": 0.224, "yaw": 1.571, "source": "setup", "stamp": None},
        "goal": None,
        "route": [],
        "routeIndex": 0,
        "routeRepeatEnabled": False,
        "routeRepeatCycle": 0,
        "routeRepeatPauseSeconds": 0.0,
        "routeRepeatStart": 1,
        "routeRepeatEnd": 1,
        "routeRepeatResumeAt": None,
        "routeCheckpoint": {
            "available": False,
            "route": [],
            "nextIndex": 0,
            "nextWaypointNumber": None,
            "completedWaypointNumbers": [],
            "lastCompletedWaypointNumber": None,
            "robotId": None,
            "forceFallback": True,
            "status": "empty",
            "interruptionReason": None,
            "resumedCount": 0,
            "updatedAt": None,
        },
        "robotPoses": {},
        "navStatus": "idle",
        "navMessage": "",
        "cameraEnabled": True,
        "cameraStream": "compressed",
        "cameraFps": None,
        "cameraBufferedFrames": 0,
        "cameraFrameGapMs": None,
        "cameraSubscriptionTopic": None,
        "cameraSubscriptionQueueDepth": None,
        "cameraExecutorThreads": None,
        "lastCameraAt": None,
        "lastScanAt": None,
        "lastPoseAt": None,
        "lastOdomAt": None,
        "lastCommandAt": None,
        "lastManualCommandAt": None,
        "cmdVelMessageType": None,
        "cmdVelStampSource": None,
        "cmdVelClockSkewMs": None,
        "cmdVelClockAgeMs": None,
        "lastCmdVelDeliveryCheckAt": None,
        "lastCmdVelDeliveryCheckOk": None,
        "odomAnchor": None,
        "fallbackActive": False,
        "fallbackPathIndex": 0,
        "fallbackPathLength": 0,
        "fallbackDisplayPath": [],
        "fallbackSpeedScale": None,
        "lidarMinClearance": None,
        "fallbackRecoveryPhase": None,
        "fallbackRecoveryAttempts": 0,
        "lidarConnection": "unknown",
        "lidarConnectionMessage": "LiDAR scan 대기 중",
        "lidarConnectionChangedAt": None,
        "lidarPoints": [],
        "dynamicLidarObstacles": [],
        "dynamicLidarObstacleCount": 0,
        "lidarPose": None,
        "lidarPointCount": 0,
        "lidarFrame": "",
        "scanAgeMs": None,
        "odomAgeMs": None,
        "connection": {
            "rosDomainId": os.environ.get("ROS_DOMAIN_ID", ""),
            "rosLocalhostOnly": os.environ.get("ROS_LOCALHOST_ONLY", ""),
            "serverIps": [],
            "actionAvailable": False,
            "routeActionAvailable": False,
        },
        "opencr": {
            "state": "unknown",
            "connected": False,
            "ready": False,
            "port": None,
            "detail": "OpenCR hardware has not been checked yet.",
            "checkedAt": None,
        },
    },
}


def deep_merge(base: Dict[str, Any], incoming: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in incoming.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def normalized_robot_body(body: Any) -> Dict[str, float]:
    raw = body if isinstance(body, dict) else {}
    try:
        length = float(raw.get("length", DEFAULT_TURTLEBOT_LENGTH))
    except (TypeError, ValueError):
        length = DEFAULT_TURTLEBOT_LENGTH
    try:
        width = float(raw.get("width", DEFAULT_TURTLEBOT_WIDTH))
    except (TypeError, ValueError):
        width = DEFAULT_TURTLEBOT_WIDTH
    return {
        "length": clamp(length, 0.05, 0.80),
        "width": clamp(width, 0.05, 0.80),
    }


def normalized_robot_map_pose(map_pose: Any) -> Dict[str, Any]:
    raw = map_pose if isinstance(map_pose, dict) else {}
    values: Dict[str, Any] = {"enabled": bool(raw.get("enabled", False)), "yaw": 0.0}
    for key in ("x", "y", "yaw"):
        try:
            value = float(raw.get(key))
        except (TypeError, ValueError):
            value = None if key in ("x", "y") else 0.0
        values[key] = value if value is not None and math.isfinite(value) else (None if key in ("x", "y") else 0.0)
    if values["x"] is None or values["y"] is None:
        values["enabled"] = False
    return values


ROBOT_CONNECTION_KEYS = (
    "robotIp",
    "rosDomainId",
    "rosLocalhostOnly",
    "robotSshHost",
    "robotSshUser",
    "robotSshPassword",
)


def normalized_robot_connection(connection: Any, fallback: Any = None) -> Dict[str, str]:
    """Normalize one robot's private ROS/SSH connection settings.

    `serverIp` deliberately stays global: it describes this dashboard host, while
    every other field describes the selected robot.
    """
    raw = connection if isinstance(connection, dict) else {}
    defaults = fallback if isinstance(fallback, dict) else {}
    result = {
        key: str(raw.get(key, defaults.get(key, "")) or "").strip()
        for key in ROBOT_CONNECTION_KEYS
    }
    try:
        domain = int(result["rosDomainId"])
    except (TypeError, ValueError):
        domain = 1
    result["rosDomainId"] = str(max(0, min(domain, 232)))
    result["rosLocalhostOnly"] = "1" if result["rosLocalhostOnly"] == "1" else "0"
    if not result["robotSshHost"]:
        result["robotSshHost"] = result["robotIp"]
    return result


def normalized_route_checkpoint(value: Any) -> Dict[str, Any]:
    raw = value if isinstance(value, dict) else {}
    route: list[Dict[str, Any]] = []
    for index, raw_pose in enumerate(raw.get("route", []) if isinstance(raw.get("route"), list) else []):
        if not isinstance(raw_pose, dict) or len(route) >= 20:
            continue
        try:
            x = float(raw_pose.get("x"))
            y = float(raw_pose.get("y"))
            yaw = float(raw_pose.get("yaw", 0.0))
            source_index_value = float(raw_pose.get("sourceIndex", index))
        except (TypeError, ValueError):
            continue
        source_index = int(source_index_value)
        if (
            not all(math.isfinite(number) for number in (x, y, yaw, source_index_value))
            or source_index_value != source_index
            or not 0 <= source_index < 20
        ):
            continue
        route.append(
            {
                "x": x,
                "y": y,
                "yaw": normalize_yaw(yaw),
                "sourceIndex": source_index,
                "final": bool(raw_pose.get("final", False)),
            }
        )

    try:
        next_index = int(float(raw.get("nextIndex", 0)))
    except (TypeError, ValueError):
        next_index = 0
    next_index = max(0, min(next_index, len(route)))
    completed: list[int] = []
    raw_completed = raw.get("completedWaypointNumbers", [])
    if isinstance(raw_completed, list):
        for raw_number in raw_completed:
            try:
                number_value = float(raw_number)
                number = int(number_value)
            except (TypeError, ValueError):
                continue
            if number_value == number and 1 <= number <= 20 and number not in completed:
                completed.append(number)
    completed.sort()
    available = bool(raw.get("available", bool(route))) and next_index < len(route)
    next_waypoint_number = (
        int(route[next_index]["sourceIndex"]) + 1 if available else None
    )
    try:
        resumed_count = max(0, int(float(raw.get("resumedCount", 0))))
    except (TypeError, ValueError):
        resumed_count = 0
    status = str(raw.get("status") or ("active" if available else "empty"))
    if not route:
        status = "empty"
    elif not available and next_index >= len(route):
        status = "completed"
    return {
        "available": available,
        "route": route,
        "nextIndex": next_index,
        "nextWaypointNumber": next_waypoint_number,
        "completedWaypointNumbers": completed,
        "lastCompletedWaypointNumber": completed[-1] if completed else None,
        "robotId": str(raw.get("robotId") or "") or None,
        "forceFallback": bool(raw.get("forceFallback", True)),
        "status": status,
        "interruptionReason": raw.get("interruptionReason"),
        "resumedCount": resumed_count,
        "updatedAt": raw.get("updatedAt"),
    }


def fresh_runtime_state(setup: Dict[str, Any], previous_runtime: Any = None) -> Dict[str, Any]:
    """Discard stale sensor/navigation data while retaining local camera preferences."""
    previous = previous_runtime if isinstance(previous_runtime, dict) else {}
    runtime = json.loads(json.dumps(DEFAULT_STATE["runtime"]))
    checkpoint = normalized_route_checkpoint(previous.get("routeCheckpoint"))
    if checkpoint["available"]:
        checkpoint["status"] = "interrupted"
        checkpoint["interruptionReason"] = "dashboard_restarted"
        checkpoint["updatedAt"] = now_iso()
    runtime.update(
        {
            "appVersion": APP_VERSION,
            "pose": None,
            "goal": None,
            "route": [],
            "routeIndex": 0,
            "routeRepeatEnabled": False,
            "routeRepeatCycle": 0,
            "routeRepeatResumeAt": None,
            "navStatus": "idle",
            "navMessage": "Dashboard restarted; waiting for ROS2 data.",
            "cameraEnabled": bool(previous.get("cameraEnabled", True)),
            "cameraStream": normalized_camera_stream_mode(
                previous.get("cameraStream", "compressed")
            ),
            "routeCheckpoint": checkpoint,
        }
    )
    _ = setup
    return runtime


def normalize_robot_profiles_state(state: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize directly registered robot profiles and migrate legacy settings."""
    setup = state.setdefault("setup", {})
    profiles = setup.get("robotProfiles")
    if not isinstance(profiles, dict):
        profiles = {}
    if "tb3_2" not in profiles:
        profiles["tb3_2"] = json.loads(json.dumps(DEFAULT_ROBOT_PROFILES["tb3_2"]))
    legacy_network = setup.get("network") if isinstance(setup.get("network"), dict) else {}
    for profile_id, profile in profiles.items():
        if not isinstance(profile, dict):
            continue
        profile["body"] = normalized_robot_body(profile.get("body"))
        profile["mapPose"] = normalized_robot_map_pose(profile.get("mapPose"))
        # Existing saved installations had a single setup.network.  Its settings
        # become TurtleBot 2's connection on first load without dropping a saved
        # SSH password.
        fallback_connection = legacy_network if profile_id == "tb3_2" else {}
        profile["connection"] = normalized_robot_connection(
            profile.get("connection"), fallback_connection
        )
    setup["robotProfiles"] = profiles
    robot = setup.get("robot") if isinstance(setup.get("robot"), dict) else {}
    try:
        legacy_length = float(robot.get("length", 0.0) or 0.0)
        legacy_width = float(robot.get("width", 0.0) or 0.0)
    except (TypeError, ValueError):
        legacy_length = legacy_width = 0.0
    if (
        not robot
        or (
            legacy_length == 0.25
            and legacy_width == 0.15
        )
    ):
        setup["robot"] = {
            "length": DEFAULT_TURTLEBOT_LENGTH,
            "width": DEFAULT_TURTLEBOT_WIDTH,
        }
    else:
        setup["robot"] = normalized_robot_body(robot)
    active_robot = str(setup.get("activeRobot") or "tb3_2")
    if active_robot not in profiles:
        active_robot = "tb3_2"
        setup["activeRobot"] = active_robot
    active_profile = profiles[active_robot]
    if not isinstance(active_profile.get("topics"), dict):
        fallback_topics = setup.get("topics")
        active_profile["topics"] = (
            dict(fallback_topics)
            if isinstance(fallback_topics, dict)
            else dict(DEFAULT_ROBOT_PROFILES["tb3_2"]["topics"])
        )
    # setup.topics remains an in-memory compatibility view. Robot profile topics
    # are the persisted source of truth for the selected robot.
    setup["topics"] = dict(active_profile["topics"])
    active_connection = active_profile.get("connection", {})
    setup_network = setup.get("network") if isinstance(setup.get("network"), dict) else {}
    for key in ROBOT_CONNECTION_KEYS:
        setup_network[key] = active_connection.get(key, "")
    setup["network"] = setup_network
    return state


def synchronize_active_robot_topics(
    setup_patch: Dict[str, Any], existing_setup: Dict[str, Any]
) -> None:
    """Persist edited topic fields with the active robot profile as well as setup.topics."""
    topics = setup_patch.get("topics")
    active_robot = str(
        setup_patch.get("activeRobot") or existing_setup.get("activeRobot") or "tb3_2"
    )
    incoming_profiles = setup_patch.get("robotProfiles")
    profiles_source = (
        incoming_profiles
        if isinstance(incoming_profiles, dict)
        else existing_setup.get("robotProfiles", {})
    )
    profiles = (
        json.loads(json.dumps(profiles_source))
        if isinstance(profiles_source, dict)
        else {}
    )
    profile = profiles.get(active_robot)
    if not isinstance(profile, dict):
        profile = {}
    if isinstance(topics, dict):
        profile_topics = profile.get("topics") if isinstance(profile.get("topics"), dict) else {}
        profile["topics"] = deep_merge(profile_topics, topics)
    network = setup_patch.get("network")
    if isinstance(network, dict):
        current_connection = profile.get("connection") if isinstance(profile.get("connection"), dict) else {}
        incoming_connection = {key: network[key] for key in ROBOT_CONNECTION_KEYS if key in network}
        profile["connection"] = deep_merge(current_connection, incoming_connection)
    profiles[active_robot] = profile
    setup_patch["robotProfiles"] = profiles


def normalized_namespace(value: Any) -> str:
    namespace = str(value or "/").strip()
    if not namespace or namespace == "/":
        return "/"
    return f"/{namespace.strip('/')}"


def detected_candidate_topics(
    candidate: Dict[str, Any], topic_names: list[str], action_names: list[str]
) -> Dict[str, str]:
    """Return only topics/actions which are present in the current ROS graph."""
    recommended = candidate.get("recommendedTopics")
    if not isinstance(recommended, dict):
        return {}
    topics = set(topic_names)
    actions = set(action_names)
    action_keys = {"goalAction", "routeAction"}
    detected: Dict[str, str] = {}
    for key, value in recommended.items():
        name = str(value or "").strip()
        if not name:
            continue
        if key in action_keys:
            if name in actions:
                detected[key] = name
        elif name in topics:
            detected[key] = name
    return detected


def finite_float(value: Any, field_name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a number.") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite.")
    return parsed


def stamp_to_nanoseconds(stamp: Any) -> Optional[int]:
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds < 0 or not 0 <= nanoseconds < NANOSECONDS_PER_SECOND:
        return None
    value = seconds * NANOSECONDS_PER_SECOND + nanoseconds
    return value if value > 0 else None


def extrapolate_clock_nanoseconds(anchor_ns: int, received_monotonic_ns: int, now_monotonic_ns: int) -> int:
    return int(anchor_ns) + max(0, int(now_monotonic_ns) - int(received_monotonic_ns))


def safe_child_path(root: Path, relative_path: str) -> Path:
    resolved_root = root.resolve()
    try:
        candidate = (resolved_root / relative_path).resolve()
        candidate.relative_to(resolved_root)
        return candidate
    except (OSError, RuntimeError, ValueError):
        return resolved_root / "__invalid_path__"


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())


def normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= math.tau
    while yaw < -math.pi:
        yaw += math.tau
    return yaw


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def normalized_fallback_settings(setup: Dict[str, Any]) -> Dict[str, Any]:
    raw = setup.get("fallbackNavigation", {}) or {}

    def bounded(name: str, default: float, lower: float, upper: float) -> float:
        try:
            value = finite_float(raw.get(name, default), name)
        except ValueError:
            value = default
        return clamp(value, lower, upper)

    max_linear = bounded("maxLinear", 0.08, 0.01, 0.12)
    min_linear = min(max_linear, bounded("minLinear", 0.04, 0.005, 0.08))
    return {
        "enabled": bool(raw.get("enabled", True)),
        "maxLinear": max_linear,
        "minLinear": min_linear,
        "maxAngular": bounded("maxAngular", 0.6, 0.1, 1.5),
        "lookahead": bounded("lookahead", 0.12, 0.05, 0.35),
        "goalTolerance": bounded("goalTolerance", 0.04, 0.02, 0.15),
        "yawTolerance": bounded("yawTolerance", 0.12, 0.05, 0.5),
        "softDistance": bounded("softDistance", 0.10, 0.0, 0.4),
        "hardMargin": bounded("hardMargin", 0.03, 0.01, 0.15),
        "scanTimeout": bounded("scanTimeout", 0.6, 0.5, 1.0),
        "odomTimeout": bounded("odomTimeout", 0.6, 0.5, 1.0),
        "collisionHorizon": bounded("collisionHorizon", 1.5, 0.4, 3.0),
    }


def effective_footprint_from_setup(setup: Dict[str, Any]) -> Dict[str, float]:
    robot = setup.get("robot", {}) or {}
    accessory = setup.get("accessory", {}) or {}
    safety = setup.get("safety", {}) or {}
    half_length = max(0.01, float(robot.get("length", 0.30))) / 2.0
    half_width = max(0.01, float(robot.get("width", 0.20))) / 2.0
    margin = max(0.0, float(safety.get("margin", 0.0)))
    return {
        "front": half_length + max(0.0, float(accessory.get("front", 0.0))) + margin,
        "back": half_length + max(0.0, float(accessory.get("back", 0.0))) + margin,
        "left": half_width + max(0.0, float(accessory.get("left", 0.0))) + margin,
        "right": half_width + max(0.0, float(accessory.get("right", 0.0))) + margin,
    }


def configured_clearance_from_setup(setup: Dict[str, Any]) -> float:
    """Return the user-visible red/yellow clearance around a wall or obstacle."""
    planner = setup.get("planner", {}) or {}
    object_config = setup.get("object", {}) or {}
    try:
        hard_clearance = max(0.05, float(planner.get("hardClearance") or 0.05))
    except (TypeError, ValueError):
        hard_clearance = 0.05
    try:
        obstacle_clearance = max(0.0, float(object_config.get("inflation") or 0.0))
    except (TypeError, ValueError):
        obstacle_clearance = 0.0
    return hard_clearance + obstacle_clearance


def planning_clearance_extents_from_setup(setup: Dict[str, Any]) -> Tuple[float, float]:
    """Return the axis-aligned body clearance used by the grid planner.

    Planning used to inflate walls by the body's circumscribed-circle radius.
    That made visible free cells unavailable even when the rectangular TurtleBot
    footprint would not contact a wall.  The map editor and the fallback A*
    planner instead use the actual body dimensions plus configured clearance.
    Runtime LiDAR collision handling remains orientation-aware.
    """
    footprint = effective_footprint_from_setup(setup)
    extra = configured_clearance_from_setup(setup)
    return (
        max(float(footprint["front"]), float(footprint["back"])) + extra,
        max(float(footprint["left"]), float(footprint["right"])) + extra,
    )


def robot_avoidance_obstacles(
    setup: Dict[str, Any], robot_poses: Optional[Dict[str, Any]] = None
) -> list[Dict[str, Any]]:
    profiles = setup.get("robotProfiles", {}) or {}
    active_robot = str(setup.get("activeRobot") or "")
    if not isinstance(profiles, dict):
        return []
    obstacles = []
    for profile_id, profile in profiles.items():
        if profile_id == active_robot or not isinstance(profile, dict):
            continue
        live_pose = (robot_poses or {}).get(profile_id)
        if isinstance(live_pose, dict) and live_pose.get("available"):
            pose = normalized_robot_map_pose({"enabled": True, **live_pose})
        else:
            pose = normalized_robot_map_pose(profile.get("mapPose"))
        if not pose["enabled"]:
            continue
        body = normalized_robot_body(profile.get("body"))
        cos_yaw = abs(math.cos(float(pose["yaw"])))
        sin_yaw = abs(math.sin(float(pose["yaw"])))
        obstacles.append(
            {
                "id": f"robot-{profile_id}",
                "label": str(profile.get("label") or profile_id),
                "x": pose["x"],
                "y": pose["y"],
                "yaw": pose["yaw"],
                "width": body["length"] * cos_yaw + body["width"] * sin_yaw,
                "height": body["length"] * sin_yaw + body["width"] * cos_yaw,
            }
        )
    return obstacles


def laser_scan_points(
    ranges: Any,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_points: int = 720,
) -> list[Tuple[float, float]]:
    values = list(ranges or [])
    if not values:
        return []
    stride = max(1, math.ceil(len(values) / max(1, int(max_points))))
    points: list[Tuple[float, float]] = []
    for index in range(0, len(values), stride):
        try:
            distance = float(values[index])
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        points.append((distance * math.cos(angle), distance * math.sin(angle)))
    return points


def sample_lidar_points(
    points: list[Tuple[float, float]], max_points: int = 240
) -> list[list[float]]:
    if not points or max_points <= 0:
        return []
    stride = max(1, math.ceil(len(points) / int(max_points)))
    return [
        [round(float(x), 3), round(float(y), 3)]
        for x, y in points[::stride][:max_points]
    ]


def sample_navigation_path(
    points: list[Dict[str, Any]], max_points: int = 320
) -> list[Dict[str, Any]]:
    if not points or max_points <= 0:
        return []
    stride = max(1, math.ceil(len(points) / int(max_points)))
    sampled = [
        {
            "x": round(float(point["x"]), 3),
            "y": round(float(point["y"]), 3),
            "slow": bool(point.get("slow", False)),
        }
        for point in points[::stride][:max_points]
    ]
    final = points[-1]
    if sampled and (sampled[-1]["x"] != round(float(final["x"]), 3) or sampled[-1]["y"] != round(float(final["y"]), 3)):
        sampled.append(
            {
                "x": round(float(final["x"]), 3),
                "y": round(float(final["y"]), 3),
                "slow": bool(final.get("slow", False)),
            }
        )
    return sampled


def lidar_dynamic_obstacle_cells(
    points: list[Tuple[float, float]],
    pose: Optional[Dict[str, Any]],
    cell_size: float = DYNAMIC_LIDAR_OBSTACLE_CELL_SIZE,
    min_points: int = DYNAMIC_LIDAR_OBSTACLE_MIN_POINTS,
    static_wall_match: Optional[Callable[[float, float], bool]] = None,
) -> list[Dict[str, Any]]:
    """Return dense LiDAR cells in map coordinates for short-lived obstacles."""
    if not pose or cell_size <= 0.0 or min_points < 1:
        return []
    try:
        pose_x = float(pose["x"])
        pose_y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))
    except (KeyError, TypeError, ValueError):
        return []
    if not all(math.isfinite(value) for value in (pose_x, pose_y, yaw)):
        return []

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    counts: Dict[Tuple[int, int], int] = {}
    for local_x, local_y in points:
        distance = math.hypot(local_x, local_y)
        if not DYNAMIC_LIDAR_OBSTACLE_MIN_RANGE <= distance <= DYNAMIC_LIDAR_OBSTACLE_MAX_RANGE:
            continue
        world_x = pose_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = pose_y + sin_yaw * local_x + cos_yaw * local_y
        if static_wall_match is not None and static_wall_match(world_x, world_y):
            continue
        key = (math.floor(world_x / cell_size), math.floor(world_y / cell_size))
        counts[key] = counts.get(key, 0) + 1

    return [
        {
            "id": f"lidar-{grid_x}-{grid_y}",
            "x": round((grid_x + 0.5) * cell_size, 3),
            "y": round((grid_y + 0.5) * cell_size, 3),
            "width": cell_size,
            "height": cell_size,
            "points": point_count,
        }
        for (grid_x, grid_y), point_count in counts.items()
        if point_count >= min_points
    ]


def rectangle_clearance(
    x: float, y: float, footprint: Dict[str, float]
) -> float:
    x_gap = max(-float(footprint["back"]) - x, 0.0, x - float(footprint["front"]))
    y_gap = max(-float(footprint["right"]) - y, 0.0, y - float(footprint["left"]))
    return math.hypot(x_gap, y_gap)


def rectangle_signed_clearance(
    x: float, y: float, footprint: Dict[str, float]
) -> float:
    minimum_x = -float(footprint["back"])
    maximum_x = float(footprint["front"])
    minimum_y = -float(footprint["right"])
    maximum_y = float(footprint["left"])
    x_gap = max(minimum_x - x, 0.0, x - maximum_x)
    y_gap = max(minimum_y - y, 0.0, y - maximum_y)
    if x_gap > 0.0 or y_gap > 0.0:
        return math.hypot(x_gap, y_gap)
    return -min(x - minimum_x, maximum_x - x, y - minimum_y, maximum_y - y)


def predicted_base_pose(linear: float, angular: float, elapsed: float) -> Tuple[float, float, float]:
    if abs(angular) < 1e-6:
        return linear * elapsed, 0.0, 0.0
    radius = linear / angular
    theta = angular * elapsed
    return radius * math.sin(theta), radius * (1.0 - math.cos(theta)), theta


def evaluate_lidar_safety(
    points: list[Tuple[float, float]],
    linear: float,
    angular: float,
    footprint: Dict[str, float],
    hard_margin: float,
    soft_distance: float,
    horizon: float,
) -> Dict[str, Any]:
    if not points:
        return {
            "collision": False,
            "scale": 1.0,
            "minClearance": None,
            "motionClearance": None,
            "slow": False,
            "recoveryRequired": False,
        }

    min_clearance = min(rectangle_clearance(x, y, footprint) for x, y in points)
    # Keep a nearby side or rear wall visible to the collision prediction, but
    # do not throttle straight movement solely because it is not in the travel
    # direction. This matters for dense waypoint routes along a wall.
    if linear > 1e-4:
        motion_points = [(x, y) for x, y in points if x > float(footprint["front"])]
    elif linear < -1e-4:
        motion_points = [(x, y) for x, y in points if x < -float(footprint["back"])]
    else:
        motion_points = []
    motion_clearance = (
        min(rectangle_clearance(x, y, footprint) for x, y in motion_points)
        if motion_points
        else None
    )
    slow = bool(
        motion_clearance is not None
        and motion_clearance > hard_margin
        and motion_clearance < hard_margin + soft_distance
    )
    recovery_required = min_clearance <= hard_margin
    expanded = {
        "front": float(footprint["front"]) + hard_margin,
        "back": float(footprint["back"]) + hard_margin,
        "left": float(footprint["left"]) + hard_margin,
        "right": float(footprint["right"]) + hard_margin,
    }
    steps = max(4, int(math.ceil(horizon / 0.1)))
    collision = False
    for step in range(steps + 1):
        elapsed = horizon * step / steps
        base_x, base_y, base_yaw = predicted_base_pose(linear, angular, elapsed)
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        for point_x, point_y in points:
            delta_x = point_x - base_x
            delta_y = point_y - base_y
            local_x = cos_yaw * delta_x + sin_yaw * delta_y
            local_y = -sin_yaw * delta_x + cos_yaw * delta_y
            if (
                -expanded["back"] <= local_x <= expanded["front"]
                and -expanded["right"] <= local_y <= expanded["left"]
            ):
                collision = True
                break
        if collision:
            break
    return {
        "collision": collision,
        "scale": 0.0 if recovery_required else 1.0,
        "minClearance": min_clearance,
        "motionClearance": motion_clearance,
        "slow": slow,
        "recoveryRequired": recovery_required,
    }


def lidar_coast_allowed(
    scan_age: float,
    odom_age: float,
    pose_age: float,
    settings: Dict[str, Any],
    last_safety: Optional[Dict[str, Any]],
) -> bool:
    """Allow a short, low-speed coast only after a recently clear LiDAR scan."""
    if not last_safety or not math.isfinite(scan_age):
        return False
    if scan_age <= settings["scanTimeout"]:
        return False
    if scan_age > settings["scanTimeout"] + FALLBACK_LIDAR_COAST_SECONDS:
        return False
    if odom_age > settings["odomTimeout"] or pose_age > settings["odomTimeout"]:
        return False
    if bool(last_safety.get("recoveryRequired")) or bool(last_safety.get("collision")):
        return False
    clearance = last_safety.get("motionClearance")
    if clearance is None:
        clearance = last_safety.get("minClearance")
    try:
        clearance_value = float(clearance)
    except (TypeError, ValueError):
        return False
    required_clearance = max(
        FALLBACK_LIDAR_COAST_MIN_CLEARANCE,
        float(settings.get("softDistance") or 0.0),
    )
    return math.isfinite(clearance_value) and clearance_value >= required_clearance


def lidar_coast_command(command: Dict[str, Any], settings: Dict[str, Any]) -> Tuple[float, float]:
    requested_linear = float(command.get("linear") or 0.0)
    requested_angular = float(command.get("angular") or 0.0)
    if abs(requested_linear) <= 1e-4:
        linear = 0.0
    else:
        linear = math.copysign(
            min(float(settings["minLinear"]), abs(requested_linear)), requested_linear
        )
    angular_limit = min(float(settings["maxAngular"]), FALLBACK_LIDAR_COAST_MAX_ANGULAR)
    return linear, clamp(requested_angular, -angular_limit, angular_limit)


def fallback_replan_due(
    recovery_started_at: Optional[float], attempts: int, now: float, last_replan_at: float
) -> bool:
    if recovery_started_at is None:
        return False
    persisted = now - recovery_started_at >= FALLBACK_REPLAN_RECOVERY_SECONDS
    exhausted = attempts >= FALLBACK_REPLAN_RECOVERY_ATTEMPTS
    return (persisted or exhausted) and now - last_replan_at >= FALLBACK_REPLAN_RECOVERY_SECONDS


def fallback_path_hits_dynamic_obstacle(
    path: list[Dict[str, Any]],
    path_index: int,
    pose: Dict[str, Any],
    obstacles: list[Dict[str, Any]],
    lookahead: float = FALLBACK_DYNAMIC_REPLAN_LOOKAHEAD,
    clearance_x: float = 0.0,
    clearance_y: Optional[float] = None,
) -> bool:
    """Detect an active temporary LiDAR cell on the next part of a fallback path."""
    if not path or not obstacles or lookahead <= 0.0:
        return False
    try:
        previous_x = float(pose["x"])
        previous_y = float(pose["y"])
    except (KeyError, TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (previous_x, previous_y)):
        return False

    remaining = lookahead
    clearance_x = max(0.0, float(clearance_x))
    clearance_y = clearance_x if clearance_y is None else max(0.0, float(clearance_y))
    valid_obstacles = []
    for obstacle in obstacles:
        try:
            obstacle_x = float(obstacle["x"])
            obstacle_y = float(obstacle["y"])
            half_width = max(0.0, float(obstacle.get("width", 0.0)) / 2.0) + clearance_x
            half_height = max(0.0, float(obstacle.get("height", 0.0)) / 2.0) + clearance_y
        except (KeyError, TypeError, ValueError):
            continue
        valid_obstacles.append((obstacle_x, obstacle_y, half_width, half_height))

    for point in path[max(0, path_index) :]:
        try:
            next_x = float(point["x"])
            next_y = float(point["y"])
        except (KeyError, TypeError, ValueError):
            continue
        segment_length = math.hypot(next_x - previous_x, next_y - previous_y)
        inspected_length = min(segment_length, remaining)
        samples = max(1, int(math.ceil(inspected_length / 0.02)))
        for sample_index in range(samples + 1):
            ratio = min(1.0, inspected_length * sample_index / samples / max(segment_length, 1e-9))
            point_x = previous_x + (next_x - previous_x) * ratio
            point_y = previous_y + (next_y - previous_y) * ratio
            if any(
                abs(point_x - obstacle_x) <= half_width
                and abs(point_y - obstacle_y) <= half_height
                for obstacle_x, obstacle_y, half_width, half_height in valid_obstacles
            ):
                return True
        remaining -= segment_length
        if remaining <= 0.0:
            break
        previous_x, previous_y = next_x, next_y
    return False


def choose_lidar_recovery_turn(points: list[Tuple[float, float]]) -> int:
    left_clearance = 3.0
    right_clearance = 3.0
    for x, y in points:
        if x < -0.25:
            continue
        distance = math.hypot(x, y)
        if y >= 0:
            left_clearance = min(left_clearance, distance)
        else:
            right_clearance = min(right_clearance, distance)
    return 1 if left_clearance >= right_clearance else -1


def lidar_recovery_motion_safe(
    points: list[Tuple[float, float]],
    linear: float,
    angular: float,
    footprint: Dict[str, float],
    horizon: float,
) -> bool:
    if not points:
        return False
    result = evaluate_lidar_safety(
        points,
        linear,
        angular,
        footprint,
        hard_margin=0.0,
        soft_distance=0.01,
        horizon=horizon,
    )
    if not bool(result["collision"]):
        return True

    steps = max(4, int(math.ceil(horizon / 0.1)))
    current_clearance = min(
        rectangle_signed_clearance(x, y, footprint) for x, y in points
    )
    if current_clearance > 0.0:
        return False

    minimum_future_clearance = math.inf
    endpoint_clearance = current_clearance
    for step in range(1, steps + 1):
        elapsed = horizon * step / steps
        base_x, base_y, base_yaw = predicted_base_pose(linear, angular, elapsed)
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        step_clearance = math.inf
        for point_x, point_y in points:
            delta_x = point_x - base_x
            delta_y = point_y - base_y
            local_x = cos_yaw * delta_x + sin_yaw * delta_y
            local_y = -sin_yaw * delta_x + cos_yaw * delta_y
            step_clearance = min(
                step_clearance,
                rectangle_signed_clearance(local_x, local_y, footprint),
            )
        minimum_future_clearance = min(minimum_future_clearance, step_clearance)
        endpoint_clearance = step_clearance

    # When the configured footprint already overlaps a scan point, rejecting the
    # trajectory at t=0 also rejects every possible escape. Permit only motion
    # that does not deepen the overlap and measurably increases final clearance.
    return (
        minimum_future_clearance
        >= current_clearance - FALLBACK_RECOVERY_ESCAPE_MAX_WORSENING
        and endpoint_clearance
        >= current_clearance + FALLBACK_RECOVERY_ESCAPE_MIN_IMPROVEMENT
    )


def next_lidar_recovery_command(
    points: list[Tuple[float, float]],
    footprint: Dict[str, float],
    recovery: Dict[str, Any],
    now: float,
) -> Dict[str, Any]:
    phase = str(recovery.get("phase") or "none")
    started = float(recovery.get("started") or now)
    attempts = max(0, int(recovery.get("attempts") or 0))
    turn = int(recovery.get("turn") or 0)

    if phase == "trapped" and now - started >= FALLBACK_RECOVERY_TRAPPED_RETRY_SECONDS:
        phase = "stop"
        started = now
        attempts = 0
        turn = choose_lidar_recovery_turn(points)

    if phase == "none":
        turn = choose_lidar_recovery_turn(points)
        if lidar_recovery_motion_safe(
            points,
            0.0,
            turn * FALLBACK_RECOVERY_TURN_SPEED,
            footprint,
            horizon=0.6,
        ):
            phase = "turn"
        elif lidar_recovery_motion_safe(
            points,
            0.0,
            -turn * FALLBACK_RECOVERY_TURN_SPEED,
            footprint,
            horizon=0.6,
        ):
            turn *= -1
            phase = "turn"
        elif lidar_recovery_motion_safe(
            points,
            -FALLBACK_RECOVERY_REVERSE_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        ):
            phase = "reverse"
        elif lidar_recovery_motion_safe(
            points,
            FALLBACK_RECOVERY_FORWARD_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        ):
            phase = "forward"
        else:
            phase = "trapped"
        started = now

    if phase == "stop" and now - started >= FALLBACK_RECOVERY_STOP_SECONDS:
        if attempts >= FALLBACK_RECOVERY_MAX_ATTEMPTS:
            phase = "trapped"
            started = now
        elif lidar_recovery_motion_safe(
            points,
            0.0,
            turn * FALLBACK_RECOVERY_TURN_SPEED,
            footprint,
            horizon=0.6,
        ):
            phase = "turn"
            started = now
        elif lidar_recovery_motion_safe(
            points,
            0.0,
            -turn * FALLBACK_RECOVERY_TURN_SPEED,
            footprint,
            horizon=0.6,
        ):
            turn *= -1
            phase = "turn"
            started = now
        elif lidar_recovery_motion_safe(
            points,
            -FALLBACK_RECOVERY_REVERSE_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        ):
            phase = "reverse"
            started = now
        elif lidar_recovery_motion_safe(
            points,
            FALLBACK_RECOVERY_FORWARD_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        ):
            phase = "forward"
            started = now
        else:
            phase = "trapped"
            started = now

    if phase == "turn":
        turn_safe = lidar_recovery_motion_safe(
            points,
            0.0,
            turn * FALLBACK_RECOVERY_TURN_SPEED,
            footprint,
            horizon=0.6,
        )
        if not turn_safe or now - started >= FALLBACK_RECOVERY_TURN_SECONDS:
            if turn_safe and lidar_recovery_motion_safe(
                points,
                FALLBACK_RECOVERY_FORWARD_SPEED,
                0.0,
                footprint,
                horizon=0.8,
            ):
                phase = "forward"
                started = now
            elif lidar_recovery_motion_safe(
                points,
                -FALLBACK_RECOVERY_REVERSE_SPEED,
                0.0,
                footprint,
                horizon=0.8,
            ):
                phase = "reverse"
                started = now
            else:
                attempts += 1
                turn *= -1
                phase = "stop" if attempts < FALLBACK_RECOVERY_MAX_ATTEMPTS else "trapped"
                started = now

    if phase == "forward":
        forward_safe = lidar_recovery_motion_safe(
            points,
            FALLBACK_RECOVERY_FORWARD_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        )
        if not forward_safe or now - started >= FALLBACK_RECOVERY_FORWARD_SECONDS:
            attempts += 1
            turn *= -1
            phase = "stop" if attempts < FALLBACK_RECOVERY_MAX_ATTEMPTS else "trapped"
            started = now

    if phase == "reverse":
        reverse_safe = lidar_recovery_motion_safe(
            points,
            -FALLBACK_RECOVERY_REVERSE_SPEED,
            0.0,
            footprint,
            horizon=0.8,
        )
        if not reverse_safe or now - started >= FALLBACK_RECOVERY_REVERSE_SECONDS:
            attempts += 1
            turn *= -1
            phase = "stop" if attempts < FALLBACK_RECOVERY_MAX_ATTEMPTS else "trapped"
            started = now

    if phase == "forward":
        linear = FALLBACK_RECOVERY_FORWARD_SPEED
    elif phase == "reverse":
        linear = -FALLBACK_RECOVERY_REVERSE_SPEED
    else:
        linear = 0.0
    angular = turn * FALLBACK_RECOVERY_TURN_SPEED if phase == "turn" else 0.0
    return {
        "phase": phase,
        "started": started,
        "attempts": attempts,
        "turn": turn,
        "linear": linear,
        "angular": angular,
    }


def compute_fallback_command(
    pose: Dict[str, float],
    path: list[Dict[str, Any]],
    path_index: int,
    final_yaw: float,
    settings: Dict[str, Any],
    path_end_index: Optional[int] = None,
) -> Dict[str, Any]:
    if not path:
        return {"reached": False, "error": "Fallback path is empty."}
    pose_x = float(pose["x"])
    pose_y = float(pose["y"])
    pose_yaw = float(pose["yaw"])
    final = path[-1]
    bounded_path_end = len(path) - 1
    if path_end_index is not None:
        bounded_path_end = min(bounded_path_end, max(0, int(path_end_index)))
    bounded_path_end = max(bounded_path_end, min(len(path) - 1, int(path_index)))
    final_segment = bounded_path_end >= len(path) - 1
    final_distance = math.hypot(float(final["x"]) - pose_x, float(final["y"]) - pose_y)
    if final_segment and final_distance <= float(settings["goalTolerance"]):
        yaw_error = normalize_yaw(float(final_yaw) - pose_yaw)
        if abs(yaw_error) <= float(settings["yawTolerance"]):
            return {
                "reached": True,
                "linear": 0.0,
                "angular": 0.0,
                "pathIndex": len(path) - 1,
                "slow": bool(final.get("slow", False)),
            }
        return {
            "reached": False,
            "linear": 0.0,
            "angular": clamp(2.0 * yaw_error, -settings["maxAngular"], settings["maxAngular"]),
            "pathIndex": len(path) - 1,
            "slow": bool(final.get("slow", False)),
        }

    search_start = min(bounded_path_end, max(0, int(path_index) - 5))
    search_end = min(
        bounded_path_end + 1,
        max(search_start + 1, int(path_index) + 250),
    )
    nearest_index = min(
        range(search_start, search_end),
        key=lambda index: math.hypot(
            float(path[index]["x"]) - pose_x, float(path[index]["y"]) - pose_y
        ),
    )
    target_index = nearest_index
    lookahead = float(settings["lookahead"])
    while target_index < bounded_path_end:
        target = path[target_index]
        if math.hypot(float(target["x"]) - pose_x, float(target["y"]) - pose_y) >= lookahead:
            break
        target_index += 1
    target = path[target_index]
    target_angle = math.atan2(float(target["y"]) - pose_y, float(target["x"]) - pose_x)
    heading_error = normalize_yaw(target_angle - pose_yaw)
    slow = any(bool(point.get("slow", False)) for point in path[nearest_index : target_index + 1])
    if abs(heading_error) > 0.75:
        linear = 0.0
        angular = clamp(1.8 * heading_error, -settings["maxAngular"], settings["maxAngular"])
    else:
        requested_speed = settings["minLinear"] if slow else settings["maxLinear"]
        linear = float(requested_speed) * max(0.35, math.cos(heading_error))
        target_distance = max(
            0.02,
            math.hypot(float(target["x"]) - pose_x, float(target["y"]) - pose_y),
        )
        curvature = 2.0 * math.sin(heading_error) / target_distance
        angular = clamp(linear * curvature, -settings["maxAngular"], settings["maxAngular"])
    return {
        "reached": False,
        "linear": linear,
        "angular": angular,
        "pathIndex": target_index,
        "slow": slow,
        "distanceRemaining": final_distance,
    }


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def local_ip_addresses() -> list[str]:
    addresses = set()
    try:
        host_name = socket.gethostname()
        for info in socket.getaddrinfo(host_name, None, socket.AF_INET):
            address = info[4][0]
            if not address.startswith("127."):
                addresses.add(address)
    except Exception:
        pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            address = sock.getsockname()[0]
            if not address.startswith("127."):
                addresses.add(address)
    except Exception:
        pass
    return sorted(addresses)


def select_server_ip(
    robot_ip: str,
    addresses: list[str],
    routed_ip: Optional[str] = None,
) -> str:
    valid: list[str] = []
    for raw in [routed_ip, *addresses]:
        if not raw or raw in valid:
            continue
        try:
            address = ipaddress.ip_address(str(raw))
        except ValueError:
            continue
        if not isinstance(address, ipaddress.IPv4Address):
            continue
        if address.is_loopback or address.is_link_local or address.is_unspecified:
            continue
        valid.append(str(address))
    if not valid:
        return ""

    try:
        robot = ipaddress.ip_address(str(robot_ip))
        robot_network = ipaddress.ip_network(f"{robot}/24", strict=False)
        same_subnet = [address for address in valid if ipaddress.ip_address(address) in robot_network]
        if same_subnet:
            return same_subnet[0]
    except ValueError:
        pass

    if routed_ip and str(routed_ip) in valid:
        return str(routed_ip)
    private = [address for address in valid if ipaddress.ip_address(address).is_private]
    return private[0] if private else valid[0]


def detect_server_ip(robot_ip: str = "") -> str:
    routed_ip = ""
    target = str(robot_ip or "").strip() or "8.8.8.8"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((target, 9))
            routed_ip = str(sock.getsockname()[0] or "")
    except Exception:
        pass
    return select_server_ip(target, local_ip_addresses(), routed_ip)


def dashboard_access_advice(server_ip: str) -> list[str]:
    advice = ["서버 PC에서는 http://localhost:8080 으로 접속하세요."]
    detected = str(server_ip or "").strip()
    if detected and not detected.startswith("127."):
        advice.append(
            f"같은 네트워크의 다른 기기에서는 http://{detected}:8080 으로 접속하세요."
        )
    return advice


def ros_bridge_error_text(exc: BaseException) -> str:
    message = str(exc).strip()
    return f"{type(exc).__name__}: {message}" if message else type(exc).__name__


def same_subnet_hosts(address: str) -> tuple[Optional[ipaddress.IPv4Network], list[str]]:
    """Return a bounded private /24 scan range without including the local host."""
    try:
        source = ipaddress.ip_address(str(address).strip())
    except ValueError:
        return None, []
    if not isinstance(source, ipaddress.IPv4Address) or not source.is_private:
        return None, []
    subnet = ipaddress.ip_network(f"{source}/24", strict=False)
    return subnet, [str(host) for host in subnet.hosts() if host != source]


def discover_same_subnet_ssh_hosts(network: Dict[str, Any]) -> Dict[str, Any]:
    """Probe SSH only on the dashboard's private /24; ROS confirms robot identity later."""
    source = str(network.get("serverIp") or network.get("robotIp") or "").strip()
    subnet, hosts = same_subnet_hosts(source)
    if subnet is None:
        return {
            "ok": False,
            "subnet": "",
            "hosts": [],
            "message": "A private server or robot IPv4 address is required for network discovery.",
        }

    configured_hosts = {
        str(network.get("robotIp") or "").strip(),
        str(network.get("robotSshHost") or "").strip(),
    }

    def ssh_open(host: str) -> Optional[str]:
        try:
            with socket.create_connection((host, 22), timeout=0.18):
                return host
        except OSError:
            return None

    open_hosts: list[str] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=32) as executor:
        for host in executor.map(ssh_open, hosts):
            if host:
                open_hosts.append(host)
    return {
        "ok": True,
        "subnet": str(subnet),
        "hosts": [
            {"ip": host, "configured": host in configured_hosts}
            for host in sorted(open_hosts, key=lambda item: tuple(int(part) for part in item.split(".")))
        ],
        "message": "SSH-reachable devices found. ROS topics are required before adding a TurtleBot profile.",
    }


def base_connection_info(
    action_available: bool = False,
    route_action_available: bool = False,
    network: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    configured = network if isinstance(network, dict) else {}
    return {
        "rosDomainId": str(configured.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID", "")),
        "rosLocalhostOnly": str(configured.get("rosLocalhostOnly") or os.environ.get("ROS_LOCALHOST_ONLY", "")),
        "rmwImplementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "serverIps": local_ip_addresses(),
        "actionAvailable": action_available,
        "routeActionAvailable": route_action_available,
    }


def redacted_network(network: Dict[str, Any]) -> Dict[str, Any]:
    redacted = dict(network or {})
    if redacted.get("robotSshPassword"):
        redacted["robotSshPassword"] = "***"
    return redacted


def public_network(network: Dict[str, Any]) -> Dict[str, Any]:
    public = dict(network or {})
    password_configured = bool(public.get("robotSshPassword"))
    public["robotSshPassword"] = ""
    public["robotSshPasswordConfigured"] = password_configured
    return public


def public_robot_profiles(profiles: Any) -> Dict[str, Any]:
    public = json.loads(json.dumps(profiles if isinstance(profiles, dict) else {}))
    for profile in public.values():
        if isinstance(profile, dict):
            profile["connection"] = public_network(profile.get("connection", {}))
    return public


def active_robot_profile(setup: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    profiles = setup.get("robotProfiles") if isinstance(setup.get("robotProfiles"), dict) else {}
    robot_id = str(setup.get("activeRobot") or "tb3_2")
    profile = profiles.get(robot_id)
    if not isinstance(profile, dict):
        robot_id = "tb3_2"
        profile = profiles.get(robot_id) if isinstance(profiles.get(robot_id), dict) else {}
    return robot_id, profile


def active_robot_network(setup: Dict[str, Any]) -> Dict[str, Any]:
    """Return the selected profile's private settings plus dashboard server IP."""
    network = dict(setup.get("network") or {})
    _robot_id, profile = active_robot_profile(setup)
    connection = profile.get("connection") if isinstance(profile.get("connection"), dict) else {}
    for key in ROBOT_CONNECTION_KEYS:
        if key in connection:
            network[key] = connection[key]
    return network


def robot_operation_lock(robot_id: str) -> threading.Lock:
    with ROBOT_SSH_OPERATION_LOCK:
        return ROBOT_SSH_OPERATION_LOCKS.setdefault(robot_id, threading.Lock())


BASE_DIAGNOSTIC_COMMANDS = [
    ["ros2", "topic", "list"],
    ["ros2", "topic", "info", "/tf", "-v"],
    ["ros2", "topic", "info", "/tf_static", "-v"],
    ["ros2", "action", "list"],
    ["ros2", "node", "list"],
]


def namespace_topic(namespace: str, suffix: str) -> str:
    if not namespace:
        return suffix
    return f"{namespace.rstrip('/')}{suffix}"


def namespace_from_topic(topic: str, suffix: str) -> Optional[str]:
    if topic == suffix:
        return ""
    if topic.endswith(suffix):
        namespace = topic[: -len(suffix)]
        if namespace.startswith("/"):
            return namespace
    return None


def dedupe_commands(commands: list[list[str]]) -> list[list[str]]:
    seen = set()
    unique = []
    for command in commands:
        key = tuple(command)
        if key in seen:
            continue
        seen.add(key)
        unique.append(command)
    return unique


def diagnostic_commands_for_topics(
    topics: Optional[Dict[str, Any]] = None,
    robot_profiles: Optional[Dict[str, Any]] = None,
) -> list[list[str]]:
    topics = topics or {}
    commands = list(BASE_DIAGNOSTIC_COMMANDS)

    def add_lifecycle_commands(namespace: str) -> None:
        ns = namespace.rstrip("/")
        for node in (
            "map_server",
            "amcl",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "behavior_server",
            "collision_monitor",
            "lifecycle_manager_navigation",
        ):
            node_name = f"{ns}/{node}" if ns else f"/{node}"
            commands.append(["ros2", "lifecycle", "get", node_name])

    def add_topic_commands(topic_config: Dict[str, Any], include_camera_rate: bool = False) -> None:
        for key in (
            "scan",
            "odom",
            "cmdVel",
            "pose",
            "camera",
            "compressedCamera",
            "initialPose",
            "goalTopic",
        ):
            topic = str(topic_config.get(key) or "").strip()
            if topic:
                commands.append(["ros2", "topic", "info", topic, "-v"])
        for key in ("scan", "odom"):
            topic = str(topic_config.get(key) or "").strip()
            if topic:
                commands.append(["ros2", "topic", "hz", topic, "--window", "5"])
        if include_camera_rate:
            topic = diagnostic_camera_rate_topic(topic_config)
            if topic:
                commands.append(["ros2", "topic", "hz", topic, "--window", "5"])
        action_name = str(topic_config.get("goalAction") or "").strip()
        if action_name:
            commands.append(["ros2", "action", "info", action_name])
            namespace = namespace_from_topic(action_name, "/navigate_to_pose")
            if namespace is not None:
                add_lifecycle_commands(namespace)
        route_action_name = str(topic_config.get("routeAction") or "").strip()
        if route_action_name:
            commands.append(["ros2", "action", "info", route_action_name])

    add_topic_commands(topics, include_camera_rate=True)
    for profile in (robot_profiles or {}).values():
        if isinstance(profile, dict):
            add_topic_commands(profile.get("topics", {}))
    return dedupe_commands(commands)


def diagnostic_camera_rate_topic(topics: Optional[Dict[str, Any]] = None) -> str:
    topic_config = topics or {}
    return str(topic_config.get("compressedCamera") or topic_config.get("camera") or "").strip()


def _text_output(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").strip()
    return str(value).strip()


def run_diagnostic_command(args: list[str], timeout: float = 6.0) -> Dict[str, Any]:
    command = " ".join(args)
    try:
        completed = subprocess.run(
            args,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout": _text_output(completed.stdout),
            "stderr": _text_output(completed.stderr),
            "ok": completed.returncode == 0,
        }
    except FileNotFoundError:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": "command not found: ros2",
            "ok": False,
        }
    except subprocess.TimeoutExpired as exc:
        stderr = _text_output(exc.stderr)
        timeout_message = f"timeout after {timeout:.1f}s"
        return {
            "command": command,
            "returncode": None,
            "stdout": _text_output(exc.stdout),
            "stderr": f"{timeout_message}\n{stderr}".strip(),
            "ok": False,
        }
    except Exception as exc:
        return {
            "command": command,
            "returncode": None,
            "stdout": "",
            "stderr": f"{type(exc).__name__}: {exc}",
            "ok": False,
        }


def collect_diagnostic_commands(
    topics: Optional[Dict[str, Any]] = None,
    robot_profiles: Optional[Dict[str, Any]] = None,
) -> list[Dict[str, Any]]:
    commands = diagnostic_commands_for_topics(topics, robot_profiles)
    results: list[Optional[Dict[str, Any]]] = [None] * len(commands)
    camera_rate_topic = diagnostic_camera_rate_topic(topics)
    camera_rate_indexes = [
        index
        for index, args in enumerate(commands)
        if len(args) >= 4
        and args[1:3] == ["topic", "hz"]
        and bool(camera_rate_topic)
        and args[3] == camera_rate_topic
    ]
    regular_indexes = [index for index in range(len(commands)) if index not in camera_rate_indexes]
    if regular_indexes:
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(regular_indexes)) as executor:
            futures = {
                index: executor.submit(run_diagnostic_command, commands[index])
                for index in regular_indexes
            }
            for index, future in futures.items():
                results[index] = future.result()
    # A camera rate probe creates another DDS image subscriber. Run only one and
    # keep it separate from the parallel probes so diagnostics do not stall video.
    for index in camera_rate_indexes:
        results[index] = run_diagnostic_command(commands[index])
    return [result for result in results if result is not None]


def redact_secret(text: str, secret: str) -> str:
    if not secret:
        return text
    return text.replace(secret, "***")


def configured_map_path(map_config: Dict[str, Any]) -> Optional[Path]:
    image_url = str(map_config.get("imageUrl") or "").strip()
    if not image_url:
        return None
    if image_url.startswith("/data/"):
        return ROOT / image_url.lstrip("/")
    path = Path(image_url)
    if path.is_absolute():
        return path
    return ROOT / image_url.lstrip("/")


def decode_pgm_gray(path: Path) -> Tuple[int, int, bytearray]:
    data = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            while index < len(data) and data[index] not in b"\r\n":
                index += 1
            return token()
        start = index
        while index < len(data) and data[index] not in b" \t\r\n":
            index += 1
        return data[start:index]

    magic = token()
    if magic not in (b"P5", b"P2"):
        raise ValueError("unsupported PGM format")
    width = int(token())
    height = int(token())
    max_value = int(token())
    if width <= 0 or height <= 0 or max_value <= 0:
        raise ValueError("invalid PGM dimensions")
    while index < len(data) and data[index] in b" \t\r\n":
        index += 1
    if magic == b"P5":
        expected = width * height
        pixels = data[index : index + expected]
        if len(pixels) < expected:
            raise ValueError("truncated PGM")
        if max_value == 255:
            return width, height, bytearray(pixels)
        return width, height, bytearray(min(255, max(0, round(value * 255 / max_value))) for value in pixels)
    values = []
    for _ in range(width * height):
        values.append(min(255, max(0, round(int(token()) * 255 / max_value))))
    return width, height, bytearray(values)


def decode_png_gray(path: Path) -> Tuple[int, int, bytearray]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG file")
    offset = 8
    width = height = bit_depth = color_type = interlace = None
    palette: list[Tuple[int, int, int]] = []
    transparency: Dict[int, int] = {}
    idat = bytearray()
    channels_by_type = {0: 1, 2: 3, 3: 1, 4: 2, 6: 4}
    while offset + 12 <= len(data):
        length = struct.unpack(">I", data[offset : offset + 4])[0]
        chunk_type = data[offset + 4 : offset + 8]
        chunk = data[offset + 8 : offset + 8 + length]
        offset += 12 + length
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _compression, _filter, interlace = struct.unpack(
                ">IIBBBBB", chunk
            )
        elif chunk_type == b"PLTE":
            palette = [tuple(chunk[i : i + 3]) for i in range(0, len(chunk), 3) if len(chunk[i : i + 3]) == 3]
        elif chunk_type == b"tRNS":
            transparency = {index: alpha for index, alpha in enumerate(chunk)}
        elif chunk_type == b"IDAT":
            idat.extend(chunk)
        elif chunk_type == b"IEND":
            break
    if width is None or height is None or bit_depth != 8 or color_type not in channels_by_type or interlace != 0:
        raise ValueError("unsupported PNG format; use 8-bit non-interlaced PNG")

    channels = channels_by_type[int(color_type)]
    row_len = int(width) * channels
    raw = zlib.decompress(bytes(idat))
    result = bytearray(int(width) * int(height))
    previous = bytearray(row_len)
    src = 0
    bpp = channels

    def paeth(a: int, b: int, c: int) -> int:
        p = a + b - c
        pa = abs(p - a)
        pb = abs(p - b)
        pc = abs(p - c)
        if pa <= pb and pa <= pc:
            return a
        if pb <= pc:
            return b
        return c

    for y in range(int(height)):
        filter_type = raw[src]
        src += 1
        row = bytearray(raw[src : src + row_len])
        src += row_len
        for i in range(row_len):
            left = row[i - bpp] if i >= bpp else 0
            up = previous[i]
            up_left = previous[i - bpp] if i >= bpp else 0
            if filter_type == 1:
                row[i] = (row[i] + left) & 0xFF
            elif filter_type == 2:
                row[i] = (row[i] + up) & 0xFF
            elif filter_type == 3:
                row[i] = (row[i] + ((left + up) // 2)) & 0xFF
            elif filter_type == 4:
                row[i] = (row[i] + paeth(left, up, up_left)) & 0xFF
            elif filter_type != 0:
                raise ValueError(f"unsupported PNG filter {filter_type}")
        for x in range(int(width)):
            i = x * channels
            alpha = 255
            if color_type == 0:
                gray = row[i]
            elif color_type == 2:
                r, g, b = row[i], row[i + 1], row[i + 2]
                gray = (299 * r + 587 * g + 114 * b) // 1000
            elif color_type == 3:
                palette_index = row[i]
                r, g, b = palette[palette_index] if palette_index < len(palette) else (255, 255, 255)
                alpha = transparency.get(palette_index, 255)
                gray = (299 * r + 587 * g + 114 * b) // 1000
            elif color_type == 4:
                gray, alpha = row[i], row[i + 1]
            else:
                r, g, b, alpha = row[i], row[i + 1], row[i + 2], row[i + 3]
                gray = (299 * r + 587 * g + 114 * b) // 1000
            result[y * int(width) + x] = 205 if alpha < 128 else gray
        previous = row
    return int(width), int(height), result


def load_map_gray(path: Path) -> Tuple[int, int, bytearray]:
    suffix = path.suffix.lower()
    if suffix == ".png":
        return decode_png_gray(path)
    if suffix == ".pgm":
        return decode_pgm_gray(path)
    raise ValueError(f"unsupported map image type: {suffix or path.name}")


def static_map_wall_near(
    map_data: Dict[str, Any], x: float, y: float,
    tolerance: float = DYNAMIC_LIDAR_STATIC_WALL_TOLERANCE,
) -> bool:
    """Return whether a world-space LiDAR point is on a known occupied map area."""
    try:
        width = int(map_data["width"])
        height = int(map_data["height"])
        pixels = map_data["pixels"]
        resolution = float(map_data["resolution"])
        origin_x = float(map_data["originX"])
        origin_y = float(map_data["originY"])
    except (KeyError, TypeError, ValueError):
        return False
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False
    min_grid_x = math.floor((x - tolerance - origin_x) / resolution)
    max_grid_x = math.floor((x + tolerance - origin_x) / resolution)
    min_grid_y = math.floor((y - tolerance - origin_y) / resolution)
    max_grid_y = math.floor((y + tolerance - origin_y) / resolution)
    for grid_y in range(max(0, min_grid_y), min(height - 1, max_grid_y) + 1):
        pixel_y = height - 1 - grid_y
        for grid_x in range(max(0, min_grid_x), min(width - 1, max_grid_x) + 1):
            if pixels[pixel_y * width + grid_x] < 80:
                return True
    return False


def paint_world_rect(
    pixels: bytearray,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    value: int,
) -> None:
    if resolution <= 0:
        return
    x0 = max(0, min(width, math.floor((min_x - origin_x) / resolution)))
    x1 = max(0, min(width, math.ceil((max_x - origin_x) / resolution)))
    y_top = max(0, min(height, math.floor(height - ((max_y - origin_y) / resolution))))
    y_bottom = max(0, min(height, math.ceil(height - ((min_y - origin_y) / resolution))))
    if x1 <= x0 or y_bottom <= y_top:
        return
    fill = bytes([max(0, min(255, int(value)))]) * (x1 - x0)
    for py in range(y_top, y_bottom):
        start = py * width + x0
        pixels[start : start + (x1 - x0)] = fill


def parse_cell_key(key: Any) -> Optional[Tuple[int, int]]:
    try:
        x_text, y_text = str(key).split(",", 1)
        return int(x_text), int(y_text)
    except Exception:
        return None


def apply_setup_overlays_to_map(
    pixels: bytearray,
    width: int,
    height: int,
    setup: Dict[str, Any],
) -> None:
    map_config = setup.get("map", {})
    planner = setup.get("planner", {})
    resolution = float(map_config.get("resolution") or 0.05)
    origin_x = float(map_config.get("originX") or 0.0)
    origin_y = float(map_config.get("originY") or 0.0)
    cell_size = float(planner.get("cellSize") or 0.02)

    for key in planner.get("blockedCells", []) or []:
        cell = parse_cell_key(key)
        if not cell:
            continue
        x, y = cell
        min_x = origin_x + x * cell_size
        min_y = origin_y + y * cell_size
        paint_world_rect(pixels, width, height, resolution, origin_x, origin_y, min_x, min_y, min_x + cell_size, min_y + cell_size, 0)

    object_inflation = float((setup.get("object") or {}).get("inflation") or 0.0)
    for obstacle in setup.get("obstacles", []) or []:
        try:
            cx = float(obstacle.get("x"))
            cy = float(obstacle.get("y"))
            obstacle_width = float(obstacle.get("width")) + object_inflation * 2.0
            obstacle_height = float(obstacle.get("height")) + object_inflation * 2.0
        except Exception:
            continue
        paint_world_rect(
            pixels,
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            cx - obstacle_width / 2.0,
            cy - obstacle_height / 2.0,
            cx + obstacle_width / 2.0,
            cy + obstacle_height / 2.0,
            0,
        )

    for key in planner.get("freeCells", []) or []:
        cell = parse_cell_key(key)
        if not cell:
            continue
        x, y = cell
        min_x = origin_x + x * cell_size
        min_y = origin_y + y * cell_size
        paint_world_rect(pixels, width, height, resolution, origin_x, origin_y, min_x, min_y, min_x + cell_size, min_y + cell_size, 254)


def build_dashboard_map_package(setup: Dict[str, Any]) -> Dict[str, Any]:
    map_config = setup.get("map", {})
    path = configured_map_path(map_config)
    if path is None or not path.exists():
        raise FileNotFoundError(f"configured map image not found: {map_config.get('imageUrl') or '-'}")
    width, height, pixels = load_map_gray(path)
    apply_setup_overlays_to_map(pixels, width, height, setup)
    resolution = float(map_config.get("resolution") or 0.05)
    origin_x = float(map_config.get("originX") or 0.0)
    origin_y = float(map_config.get("originY") or 0.0)
    origin_yaw = float(map_config.get("originYaw") or 0.0)
    pgm = f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    yaml_text = (
        "image: dashboard_map.pgm\n"
        "mode: trinary\n"
        f"resolution: {resolution:.9g}\n"
        f"origin: [{origin_x:.9g}, {origin_y:.9g}, {origin_yaw:.9g}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    return {
        "source": str(path),
        "width": width,
        "height": height,
        "resolution": resolution,
        "yaml": yaml_text,
        "pgmGzipBase64": base64.b64encode(gzip.compress(pgm, compresslevel=9)).decode("ascii"),
    }


def fallback_grid_metrics(setup: Dict[str, Any], width: int, height: int) -> Dict[str, Any]:
    map_config = setup.get("map", {}) or {}
    planner = setup.get("planner", {}) or {}
    resolution = max(0.0001, float(map_config.get("resolution") or 0.05))
    cell_size = max(0.005, float(planner.get("cellSize") or 0.02))
    return {
        "resolution": resolution,
        "cellSize": cell_size,
        "originX": float(map_config.get("originX") or 0.0),
        "originY": float(map_config.get("originY") or 0.0),
        "cols": max(1, math.ceil(width * resolution / cell_size)),
        "rows": max(1, math.ceil(height * resolution / cell_size)),
    }


def fallback_world_to_cell(
    x: float, y: float, metrics: Dict[str, Any]
) -> Optional[Tuple[int, int]]:
    cell_x = math.floor((x - float(metrics["originX"])) / float(metrics["cellSize"]))
    cell_y = math.floor((y - float(metrics["originY"])) / float(metrics["cellSize"]))
    if cell_x < 0 or cell_y < 0 or cell_x >= int(metrics["cols"]) or cell_y >= int(metrics["rows"]):
        return None
    return cell_x, cell_y


def fallback_cell_to_world(cell: Tuple[int, int], metrics: Dict[str, Any]) -> Dict[str, float]:
    return {
        "x": float(metrics["originX"]) + (cell[0] + 0.5) * float(metrics["cellSize"]),
        "y": float(metrics["originY"]) + (cell[1] + 0.5) * float(metrics["cellSize"]),
    }


def fallback_astar_cells(
    start: Tuple[int, int],
    goal: Tuple[int, int],
    blocked: set[Tuple[int, int]],
    cols: int,
    rows: int,
    soft: Optional[set[Tuple[int, int]]] = None,
) -> list[Tuple[int, int]]:
    if goal in blocked or not (0 <= start[0] < cols and 0 <= start[1] < rows):
        return []
    blocked = set(blocked)
    soft = set(soft or ())
    blocked.discard(start)
    queue: list[Tuple[float, float, Tuple[int, int]]] = []
    heapq.heappush(queue, (math.hypot(goal[0] - start[0], goal[1] - start[1]), 0.0, start))
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    scores: Dict[Tuple[int, int], float] = {start: 0.0}
    closed: set[Tuple[int, int]] = set()
    while queue:
        _priority, score, current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return list(reversed(path))
        closed.add(current)
        for delta_x, delta_y in (
            (-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)
        ):
            next_cell = (current[0] + delta_x, current[1] + delta_y)
            if (
                next_cell[0] < 0
                or next_cell[1] < 0
                or next_cell[0] >= cols
                or next_cell[1] >= rows
                or next_cell in blocked
                or next_cell in closed
            ):
                continue
            if delta_x and delta_y and (
                (current[0] + delta_x, current[1]) in blocked
                or (current[0], current[1] + delta_y) in blocked
            ):
                continue
            soft_multiplier = ASTAR_SOFT_CELL_MULTIPLIER if next_cell in soft else 1.0
            next_score = score + math.hypot(delta_x, delta_y) * soft_multiplier
            if next_score >= scores.get(next_cell, math.inf):
                continue
            scores[next_cell] = next_score
            came_from[next_cell] = current
            heuristic = math.hypot(goal[0] - next_cell[0], goal[1] - next_cell[1])
            heapq.heappush(queue, (next_score + heuristic, next_score, next_cell))
    return []


def fallback_inflated_cells(
    blocked: set[Tuple[int, int]], radius: float, metrics: Dict[str, Any]
) -> set[Tuple[int, int]]:
    cell_size = float(metrics["cellSize"])
    radius = max(0.0, float(radius))
    radius_cells = math.ceil(radius / cell_size)
    inflated = set(blocked)
    for cell_x, cell_y in blocked:
        for offset_x in range(-radius_cells, radius_cells + 1):
            for offset_y in range(-radius_cells, radius_cells + 1):
                if math.hypot(offset_x, offset_y) * cell_size > radius:
                    continue
                next_cell = cell_x + offset_x, cell_y + offset_y
                if 0 <= next_cell[0] < int(metrics["cols"]) and 0 <= next_cell[1] < int(metrics["rows"]):
                    inflated.add(next_cell)
    return inflated


def fallback_inflated_cells_for_extents(
    blocked: set[Tuple[int, int]],
    clearance_x: float,
    clearance_y: float,
    metrics: Dict[str, Any],
) -> set[Tuple[int, int]]:
    """Expand occupied cells by the rectangular robot footprint in map axes."""
    cell_size = float(metrics["cellSize"])
    clearance_x = max(0.0, float(clearance_x))
    clearance_y = max(0.0, float(clearance_y))
    x_cells = math.ceil(clearance_x / cell_size)
    y_cells = math.ceil(clearance_y / cell_size)
    inflated = set(blocked)
    for cell_x, cell_y in blocked:
        for offset_x in range(-x_cells, x_cells + 1):
            for offset_y in range(-y_cells, y_cells + 1):
                if abs(offset_x) * cell_size > clearance_x or abs(offset_y) * cell_size > clearance_y:
                    continue
                next_cell = cell_x + offset_x, cell_y + offset_y
                if 0 <= next_cell[0] < int(metrics["cols"]) and 0 <= next_cell[1] < int(metrics["rows"]):
                    inflated.add(next_cell)
    return inflated


def fallback_route_path_ends(
    path: list[Dict[str, Any]], route_length: int
) -> Dict[int, int]:
    ends: Dict[int, int] = {}
    for index, point in enumerate(path):
        marker = point.get("routeIndex")
        if isinstance(marker, int) and 0 <= marker < route_length:
            ends[marker] = index
    return ends


def fallback_next_route_index(
    route_index: int, route_length: int, route_path_ends: Dict[int, int]
) -> Optional[int]:
    if route_index >= route_length - 1:
        return None
    if not route_path_ends:
        return route_index + 1
    candidates = sorted(index for index in route_path_ends if index > route_index)
    return candidates[0] if candidates else None


def fallback_replan_path(
    setup: Dict[str, Any],
    pose: Dict[str, Any],
    route: list[Dict[str, Any]],
    route_index: int,
    dynamic_obstacles: list[Dict[str, Any]],
    robot_poses: Optional[Dict[str, Any]] = None,
) -> Tuple[list[Dict[str, Any]], str]:
    map_config = setup.get("map", {}) or {}
    source = configured_map_path(map_config)
    if source is None or not source.exists():
        return [], "configured map image is unavailable"
    try:
        width, height, pixels = load_map_gray(source)
        apply_setup_overlays_to_map(pixels, width, height, setup)
        metrics = fallback_grid_metrics(setup, width, height)
        pose_x = float(pose["x"])
        pose_y = float(pose["y"])
    except (KeyError, TypeError, ValueError, OSError) as exc:
        return [], f"map preparation failed: {exc}"
    start = fallback_world_to_cell(pose_x, pose_y, metrics)
    if start is None:
        return [], "current pose is outside the map"

    blocked: set[Tuple[int, int]] = set()
    for pixel_y in range(height):
        grid_y = math.floor(((height - 1 - pixel_y) * float(metrics["resolution"])) / float(metrics["cellSize"]))
        if grid_y < 0 or grid_y >= int(metrics["rows"]):
            continue
        row_start = pixel_y * width
        for pixel_x in range(width):
            if pixels[row_start + pixel_x] >= 80:
                continue
            grid_x = math.floor((pixel_x * float(metrics["resolution"])) / float(metrics["cellSize"]))
            if 0 <= grid_x < int(metrics["cols"]):
                blocked.add((grid_x, grid_y))
    for obstacle in [*robot_avoidance_obstacles(setup, robot_poses), *dynamic_obstacles]:
        try:
            center_x = float(obstacle["x"])
            center_y = float(obstacle["y"])
            obstacle_width = float(obstacle.get("width", DYNAMIC_LIDAR_OBSTACLE_CELL_SIZE))
            obstacle_height = float(obstacle.get("height", DYNAMIC_LIDAR_OBSTACLE_CELL_SIZE))
        except (KeyError, TypeError, ValueError):
            continue
        min_cell = fallback_world_to_cell(
            center_x - obstacle_width / 2.0, center_y - obstacle_height / 2.0, metrics
        )
        max_cell = fallback_world_to_cell(
            center_x + obstacle_width / 2.0, center_y + obstacle_height / 2.0, metrics
        )
        if min_cell is None or max_cell is None:
            continue
        for grid_x in range(min_cell[0], max_cell[0] + 1):
            for grid_y in range(min_cell[1], max_cell[1] + 1):
                blocked.add((grid_x, grid_y))

    clearance_x, clearance_y = planning_clearance_extents_from_setup(setup)
    visible_clearance = configured_clearance_from_setup(setup)
    soft_distance = normalized_fallback_settings(setup)["softDistance"]
    inflated = fallback_inflated_cells_for_extents(
        blocked, clearance_x, clearance_y, metrics
    )
    soft_inflated = fallback_inflated_cells_for_extents(
        blocked,
        visible_clearance + soft_distance,
        visible_clearance + soft_distance,
        metrics,
    )

    remaining_route = route[max(0, int(route_index)) :]
    if not remaining_route:
        return [], "no remaining route target"
    path: list[Dict[str, Any]] = []
    current = start
    for marker, target in enumerate(remaining_route, start=max(0, int(route_index))):
        try:
            target_cell = fallback_world_to_cell(float(target["x"]), float(target["y"]), metrics)
        except (KeyError, TypeError, ValueError):
            return [], "route contains an invalid target"
        if target_cell is None:
            if marker < len(route) - 1:
                continue
            return [], f"final route target {marker + 1} is outside the map"
        segment = fallback_astar_cells(
            current,
            target_cell,
            inflated,
            int(metrics["cols"]),
            int(metrics["rows"]),
            soft_inflated,
        )
        if not segment:
            if marker < len(route) - 1:
                continue
            return [], f"no safe detour to final route target {marker + 1}"
        for cell in segment[1 if path else 0 :]:
            path.append(
                {
                    **fallback_cell_to_world(cell, metrics),
                    "slow": cell in soft_inflated,
                    "routeIndex": marker,
                }
            )
        current = target_cell
    return path, ""


def build_robot_bringup_script(network: Dict[str, Any], map_package: Optional[Dict[str, Any]] = None) -> str:
    domain = shlex.quote(str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"))
    localhost_only = shlex.quote(str(network.get("rosLocalhostOnly") or "0"))
    ssh_password = str(network.get("robotSshPassword") or "")
    sudo_password_b64 = base64.b64encode((ssh_password + "\n").encode("utf-8")).decode("ascii") if ssh_password else ""
    sudo_password_b64_q = shlex.quote(sudo_password_b64)
    map_upload_script = ""
    if map_package:
        map_upload_script = f"""
DASHBOARD_MAP_DIR="$HOME/maps/dashboard"
mkdir -p "$DASHBOARD_MAP_DIR"
printf '%s' {shlex.quote(str(map_package["pgmGzipBase64"]))} | base64 -d | gzip -dc > "$DASHBOARD_MAP_DIR/dashboard_map.pgm"
cat > "$DASHBOARD_MAP_DIR/dashboard_map.yaml" <<'DASHBOARD_MAP_YAML'
{map_package["yaml"]}DASHBOARD_MAP_YAML
DASHBOARD_NAV_MAP="$DASHBOARD_MAP_DIR/dashboard_map.yaml"
echo "[OK] dashboard map uploaded: $DASHBOARD_NAV_MAP ({map_package['width']}x{map_package['height']} @ {map_package['resolution']} m/px)"
"""
    return f"""#!/usr/bin/env bash
set +e
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
export ROS2CLI_NO_DAEMON=1
export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"
export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"
LOG_DIR="$HOME/turtlebot_dashboard_logs"
mkdir -p "$LOG_DIR"
USER_UID="$(id -u)"
export XDG_RUNTIME_DIR="/run/user/$USER_UID"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"

source_ros() {{
  source /opt/ros/jazzy/setup.bash || return 10
  [ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
  export ROS_DOMAIN_ID={domain}
  export ROS_LOCALHOST_ONLY={localhost_only}
  export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
  export ROS2CLI_NO_DAEMON=1
  export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"
  export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"
}}

source_ros || {{ echo "[FAIL] ROS2 Jazzy setup.bash not found"; exit 10; }}
{map_upload_script}

process_running() {{
  pgrep -af "$1" 2>/dev/null | grep -v pgrep >/dev/null 2>&1
}}

pkg_exists() {{
  timeout -k 2 4 ros2 pkg prefix "$1" >/dev/null 2>&1
}}

shutdown_nav2_lifecycle_manager() {{
  local manager="$1"
  local response
  response="$(timeout -k 1 8 ros2 service call "$manager/manage_nodes" nav2_msgs/srv/ManageLifecycleNodes "{{command: 4}}" 2>&1)"
  if printf '%s\n' "$response" | grep -Eiq 'success[=:] *true'; then
    echo "[STOP] Nav2 lifecycle shutdown: $manager"
    return 0
  fi
  echo "[INFO] Nav2 lifecycle manager unavailable or already stopped: $manager"
  return 1
}}

stop_dashboard_tmux_session() {{
  local session="$1"
  local attempt
  if ! tmux has-session -t "$session" 2>/dev/null; then
    return 0
  fi
  tmux send-keys -t "$session" C-c >/dev/null 2>&1 || true
  echo "[STOP] Ctrl+C sent to dashboard tmux $session"
  for attempt in $(seq 1 12); do
    if ! tmux has-session -t "$session" 2>/dev/null; then
      echo "[OK] tmux session exited after Ctrl+C: $session"
      return 0
    fi
    sleep 1
  done
  echo "[WARN] tmux session did not exit after Ctrl+C: $session"
  tmux kill-session -t "$session" >/dev/null 2>&1 || true
  echo "[FALLBACK] tmux session force-closed: $session"
  return 1
}}

start_detached() {{
  local name="$1"
  shift
  local script="$LOG_DIR/${{name}}.sh"
  local log="$LOG_DIR/${{name}}.log"
  local runner="$LOG_DIR/${{name}}_runner.sh"
  local pid_file="$LOG_DIR/${{name}}.pid"
  {{
    echo '#!/usr/bin/env bash'
    echo 'set +e'
    echo 'source /opt/ros/jazzy/setup.bash'
    echo '[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"'
    echo 'export ROS_DOMAIN_ID={domain}'
    echo 'export ROS_LOCALHOST_ONLY={localhost_only}'
    echo 'export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"'
    echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"'
    echo 'export ROS2CLI_NO_DAEMON=1'
    echo 'export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"'
    echo 'export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"'
    printf '%s\\n' "$*"
  }} > "$script"
  chmod +x "$script"
  : > "$log"
  {{
    echo '#!/usr/bin/env bash'
    printf 'bash %q > %q 2>&1\n' "$script" "$log"
    echo 'code=$?'
    printf 'echo "[EXIT] %s code=$code at $(date -Is)" >> %q\n' "$name" "$log"
    echo 'sleep 2'
    echo 'exit $code'
  }} > "$runner"
  chmod +x "$runner"
  if command -v tmux >/dev/null 2>&1; then
    if tmux has-session -t "$name" 2>/dev/null; then
      if [ "$name" = "tb3_nav2" ]; then
        shutdown_nav2_lifecycle_manager /lifecycle_manager_navigation || true
        shutdown_nav2_lifecycle_manager /lifecycle_manager_localization || true
      fi
      stop_dashboard_tmux_session "$name" || true
    fi
    tmux new-session -d -s "$name" "bash $runner"
    echo "[START] $name via tmux"
  else
    if [ -s "$pid_file" ]; then
      local previous_pid
      previous_pid="$(cat "$pid_file" 2>/dev/null || true)"
      if [[ "$previous_pid" =~ ^[0-9]+$ ]] && kill -0 "$previous_pid" 2>/dev/null; then
        echo "[FAIL] Previous dashboard process for $name is still running (pid $previous_pid)."
        echo "[FIX] Use Bringup Stop first; it sends SIGINT before any fallback signal."
        return 16
      fi
      rm -f "$pid_file"
    fi
    nohup setsid bash "$script" > "$log" 2>&1 </dev/null &
    echo "$!" > "$pid_file"
    echo "[START] $name via nohup (process group $!)"
  fi
  echo "[LOG] $log"
}}

BASE_SERVICE="turtlebot-dashboard-base.service"
SUDO_PASSWORD_B64={sudo_password_b64_q}

ensure_user_linger() {{
  local linger
  linger="$(loginctl show-user "$USER" -p Linger --value 2>/dev/null || true)"
  if [ "$linger" = "yes" ]; then
    echo "[OK] systemd user lingering is already enabled for $USER."
    unset SUDO_PASSWORD_B64
    return 0
  fi

  echo "[INFO] Enabling systemd user lingering for $USER."
  if sudo -n loginctl enable-linger "$USER" >/dev/null 2>&1; then
    :
  elif [ -n "$SUDO_PASSWORD_B64" ]; then
    if ! printf '%s' "$SUDO_PASSWORD_B64" | base64 -d | sudo -S -p '' loginctl enable-linger "$USER" >/dev/null; then
      unset SUDO_PASSWORD_B64
      echo "[FAIL] Could not enable systemd user lingering with the configured SSH password."
      echo "[FIX] Check sudo permission/password, or run once on the robot: sudo loginctl enable-linger $USER"
      return 14
    fi
  else
    unset SUDO_PASSWORD_B64
    echo "[FAIL] systemd user lingering is disabled and no SSH password is configured for sudo."
    echo "[FIX] Configure the SSH password or run once on the robot: sudo loginctl enable-linger $USER"
    return 14
  fi
  unset SUDO_PASSWORD_B64

  linger="$(loginctl show-user "$USER" -p Linger --value 2>/dev/null || true)"
  if [ "$linger" != "yes" ]; then
    echo "[FAIL] systemd user lingering is still disabled for $USER."
    echo "[FIX] Run once on the robot: sudo loginctl enable-linger $USER"
    return 14
  fi
  echo "[OK] systemd user lingering enabled for $USER."
}}

start_base_service() {{
  local opencr_port="$1"
  local unit_dir="$HOME/.config/systemd/user"
  local unit_file="$unit_dir/$BASE_SERVICE"
  local base_script="$LOG_DIR/tb3_base.sh"
  local base_log="$LOG_DIR/tb3_base.log"
  ensure_user_linger || return 14
  local manager_output
  if ! manager_output="$(systemctl --user show-environment 2>&1)"; then
    echo "[FAIL] systemd user manager is unavailable for $USER."
    echo "[DETAIL] ${{manager_output:-user bus did not respond at $DBUS_SESSION_BUS_ADDRESS}}"
    echo "[FIX] Log in as this robot user and make systemctl --user available before dashboard bringup."
    return 13
  fi
  mkdir -p "$unit_dir"
  cat > "$base_script" <<BASE_SCRIPT
#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
export ROS2CLI_NO_DAEMON=1
export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"
export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"
exec ros2 launch turtlebot3_bringup robot.launch.py usb_port:=$opencr_port
BASE_SCRIPT
  chmod +x "$base_script"
  cat > "$unit_file" <<BASE_UNIT
[Unit]
Description=TurtleBot Dashboard base bringup

[Service]
Type=simple
ExecStart=/bin/bash $base_script
Restart=no
KillMode=control-group
KillSignal=SIGINT
TimeoutStopSec=12
SendSIGKILL=yes
StandardOutput=append:$base_log
StandardError=append:$base_log
BASE_UNIT
  systemctl --user daemon-reload || return 13
  systemctl --user stop "$BASE_SERVICE" >/dev/null 2>&1 || true
  systemctl --user reset-failed "$BASE_SERVICE" >/dev/null 2>&1 || true
  systemctl --user start "$BASE_SERVICE" || return 15
  if systemctl --user is-active --quiet "$BASE_SERVICE"; then
    echo "[START] $BASE_SERVICE via systemd user service"
    echo "[LOG] $base_log"
    return 0
  fi
  echo "[FAIL] $BASE_SERVICE did not become active"
  systemctl --user status "$BASE_SERVICE" --no-pager -l 2>&1 || true
  return 15
}}

echo "== dashboard robot bringup =="
echo "host: $(hostname)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"
echo
BRINGUP_FAIL=0

echo "== base bringup =="
is_opencr_port() {{
  local port="$1"
  [ -c "$port" ] || return 1
  local properties
  properties="$(udevadm info -q property -n "$port" 2>/dev/null || true)"
  printf '%s\n' "$properties" | grep -Eiq 'Arduino|Uno' && return 1
  # ROBOTIS' OpenCR CDC rule identifies the normal firmware device as
  # STM32 0483:5740.  Keep the OpenCR text check as a compatibility fallback.
  printf '%s\n' "$properties" | grep -Eiq '^ID_VENDOR_ID=0483$' || {{
    printf '%s\n' "$properties" | grep -Eiq '(^ID_MODEL(_ENC)?=.*OpenCR|^ID_SERIAL(_SHORT)?=.*OpenCR)' || return 1
    return 0
  }}
  printf '%s\n' "$properties" | grep -Eiq '^ID_MODEL_ID=5740$'
}}

OPENCR_PORT=""
shopt -s nullglob
OPENCR_CANDIDATES=(/dev/ttyACM*)
if [ "${{#OPENCR_CANDIDATES[@]}}" -eq 0 ]; then
  echo "[INFO] No /dev/ttyACM candidate is present."
fi
for candidate in "${{OPENCR_CANDIDATES[@]}}"; do
  echo "[CHECK] OpenCR candidate: $candidate"
  udevadm info -q property -n "$candidate" 2>/dev/null | grep -E 'DEVNAME|ID_VENDOR|ID_MODEL|ID_SERIAL' || true
  if is_opencr_port "$candidate"; then
    OPENCR_PORT="$candidate"
    echo "[OK] verified ROBOTIS OpenCR: $OPENCR_PORT"
    break
  fi
  echo "[REJECT] $candidate is not a verified OpenCR device; it will not be used."
done

if [ -z "$OPENCR_PORT" ]; then
  echo "[FAIL] No verified ROBOTIS OpenCR (STM32 0483:5740) found on /dev/ttyACM*."
  echo "[FAIL] Refusing to launch turtlebot3_node against Arduino Uno or an unknown serial device."
  BRINGUP_FAIL=11
elif systemctl --user is-active --quiet "$BASE_SERVICE"; then
  if timeout -k 1 3 ros2 node list 2>/dev/null | grep -Eq '(^|/)turtlebot3_node$'; then
    echo "[OK] $BASE_SERVICE is active and turtlebot3_node is alive on $OPENCR_PORT"
  else
    echo "[FAIL] $BASE_SERVICE is active but turtlebot3_node is absent from the ROS graph"
    BRINGUP_FAIL=12
  fi
elif process_running "[t]urtlebot3_ros.*-i $OPENCR_PORT"; then
  echo "[FAIL] TurtleBot base is already running outside $BASE_SERVICE."
  echo "[FIX] Stop the manually started ros2 launch once, then start it from the dashboard."
  BRINGUP_FAIL=16
else
  start_base_service "$OPENCR_PORT"
  base_start_code=$?
  if [ "$base_start_code" -ne 0 ]; then
    BRINGUP_FAIL="$base_start_code"
  else
    echo "[INFO] base bringup start requested with verified port $OPENCR_PORT via $BASE_SERVICE"
    BASE_READY=0
    for attempt in $(seq 1 12); do
      if process_running "[t]urtlebot3_ros.*-i $OPENCR_PORT" && timeout -k 1 3 ros2 node list 2>/dev/null | grep -Eq '(^|/)turtlebot3_node$'; then
        BASE_READY=1
        break
      fi
      sleep 1
    done
    if [ "$BASE_READY" = "1" ]; then
      echo "[OK] OpenCR handshake verified: turtlebot3_node is alive on $OPENCR_PORT"
    else
      echo "[FAIL] turtlebot3_node did not become ready on verified OpenCR port $OPENCR_PORT"
      echo "[INFO] recent base log:"
      tail -60 "$LOG_DIR/tb3_base.log" 2>/dev/null || true
      BRINGUP_FAIL=12
    fi
  fi
fi

if [ "$BRINGUP_FAIL" -ne 0 ]; then
  echo "[FAIL] Base bringup failed; Nav2 and camera startup were skipped."
  exit "$BRINGUP_FAIL"
fi

echo
echo "== nav2/amcl bringup =="
if [ -n "${{DASHBOARD_NAV_MAP:-}}" ]; then
  echo "[INFO] dashboard map is configured; restarting Nav2 tmux session with $DASHBOARD_NAV_MAP"
  shutdown_nav2_lifecycle_manager /lifecycle_manager_navigation || true
  shutdown_nav2_lifecycle_manager /lifecycle_manager_localization || true
  stop_dashboard_tmux_session tb3_nav2 || true
  if process_running "[m]ap_server|[a]mcl|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[c]ollision_monitor|[l]ifecycle_manager_(navigation|localization)|[t]urtlebot3_navigation2|[n]av2_bringup"; then
    echo "[FAIL] Nav2 process remained after graceful dashboard shutdown; refusing to start a duplicate stack."
    BRINGUP_FAIL=22
  fi
fi
if [ "$BRINGUP_FAIL" -ne 0 ]; then
  echo "[SKIP] Nav2 start because a previous dashboard-managed Nav2 stack did not stop cleanly."
elif process_running "[m]ap_server|[a]mcl|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[t]urtlebot3_navigation2|[n]av2_bringup" && [ -z "${{DASHBOARD_NAV_MAP:-}}" ]; then
  echo "[OK] Nav2 appears to be already running"
else
  NAV_MAP="${{DASHBOARD_NAV_MAP:-}}"
  if [ -z "$NAV_MAP" ]; then
    for candidate in "$HOME/maps/arena_shared/arena_shared.yaml" "$HOME/maps/map.yaml" "$HOME/map.yaml"; do
      if [ -f "$candidate" ]; then
        NAV_MAP="$candidate"
        break
      fi
    done
  fi
  if [ -n "$NAV_MAP" ]; then
    if pkg_exists turtlebot3_navigation2; then
      start_detached tb3_nav2 "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False map:=$NAV_MAP"
      echo "[INFO] nav2 map: $NAV_MAP"
      echo "[INFO] Nav2 start requested asynchronously. Use SSH check after 10-20 seconds for readiness."
    else
      echo "[FAIL] turtlebot3_navigation2 package not found on robot. Install ros-jazzy-turtlebot3-navigation2 or source the workspace containing it."
      BRINGUP_FAIL=20
    fi
  else
    echo "[SKIP] Nav2 map yaml not found. Checked ~/maps/arena_shared/arena_shared.yaml, ~/maps/map.yaml, ~/map.yaml"
    BRINGUP_FAIL=21
  fi
fi

echo
echo "== camera bringup =="
if process_running "[c]amera_ros|[c]amera_node|[c]amera.launch.py|[r]ealsense|[u]sb_cam|[v]4l2"; then
  echo "[OK] camera process already running"
elif pkg_exists turtlebot3_bringup && pkg_exists camera_ros; then
  start_detached tb3_camera "export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:${{LD_LIBRARY_PATH:-}}; export LIBCAMERA_IPA_MODULE_PATH=/usr/local/lib/aarch64-linux-gnu/libcamera/ipa; ros2 launch turtlebot3_bringup camera.launch.py format:=RGB888"
  echo "[INFO] TurtleBot3 Pi camera launch requested asynchronously"
elif pkg_exists realsense2_camera; then
  start_detached tb3_camera "ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false rgb_camera.color_profile:=640x480x15"
  echo "[INFO] RealSense camera start requested asynchronously"
else
  echo "[SKIP] TurtleBot3 Pi camera packages not found. Install turtlebot3_bringup and camera_ros, or bring up usb_cam, v4l2_camera, or realsense2_camera separately."
fi

echo
echo "== async sessions =="
tmux ls 2>/dev/null || true
echo "== systemd base service =="
systemctl --user status "$BASE_SERVICE" --no-pager -l 2>&1 || true
echo
for f in "$LOG_DIR"/tb3_base.log "$LOG_DIR"/tb3_nav2.log "$LOG_DIR"/tb3_camera.log "$LOG_DIR"/tb3_base.sh "$LOG_DIR"/tb3_nav2.sh "$LOG_DIR"/tb3_camera.sh; do
  [ -e "$f" ] || continue
  echo "== $f =="
  tail -40 "$f" 2>/dev/null || true
done
echo
echo "[INFO] Bringup commands were dispatched asynchronously. Run Robot Check or SSH Check after the nodes have had time to start."
exit "$BRINGUP_FAIL"
"""


def build_robot_ssh_diagnostics_script(
    network: Dict[str, Any],
    topics: Dict[str, Any],
    detailed: bool = False,
) -> str:
    domain = shlex.quote(str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"))
    localhost_only = shlex.quote(str(network.get("rosLocalhostOnly") or "0"))
    topic_candidates: list[str] = []
    for key in (
        "scan",
        "odom",
        "cmdVel",
        "pose",
        "camera",
        "compressedCamera",
        "initialPose",
        "goalTopic",
    ):
        topic = str(topics.get(key) or "").strip()
        if topic:
            topic_candidates.append(topic)
    topic_candidates.extend(
        [
            "/tf",
            "/tf_static",
            "/navigate_to_pose/_action/status",
            "/navigate_through_poses/_action/status",
            "/camera/camera/image_raw",
            "/camera/camera/image_raw/compressed",
            "/camera/color/image_raw",
            "/camera/color/image_raw/compressed",
            "/camera/image_raw",
            "/camera/image_raw/compressed",
            "/camera/camera/color/image_raw",
            "/camera/camera/color/image_raw/compressed",
            "/tb3_2/camera/image_raw",
            "/tb3_2/camera/image_raw/compressed",
        ]
    )
    unique_topics = []
    for topic in topic_candidates:
        if topic and topic not in unique_topics:
            unique_topics.append(topic)
    topic_array = " ".join(shlex.quote(topic) for topic in unique_topics)
    hz_topics = []
    for topic in (
        topics.get("scan"),
        topics.get("odom"),
        topics.get("compressedCamera") or topics.get("camera"),
    ):
        normalized = str(topic or "").strip()
        if normalized and normalized not in hz_topics:
            hz_topics.append(normalized)
    hz_topic_array = " ".join(shlex.quote(topic) for topic in hz_topics)
    action_name = str(topics.get("goalAction") or "/navigate_to_pose").strip() or "/navigate_to_pose"
    action_name_q = shlex.quote(action_name)
    route_action_name = (
        str(topics.get("routeAction") or "/navigate_through_poses").strip()
        or "/navigate_through_poses"
    )
    route_action_name_q = shlex.quote(route_action_name)
    lifecycle_nodes = [
        "/map_server",
        "/amcl",
        "/controller_server",
        "/planner_server",
        "/bt_navigator",
        "/behavior_server",
        "/collision_monitor",
        "/lifecycle_manager_navigation",
    ]
    lifecycle_array = " ".join(shlex.quote(node) for node in lifecycle_nodes)
    detailed_flag = "1" if detailed else "0"
    return f"""#!/usr/bin/env bash
set +e
DETAILED={detailed_flag}
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
export ROS2CLI_NO_DAEMON=1
export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"
export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"
USER_UID="$(id -u)"
export XDG_RUNTIME_DIR="/run/user/$USER_UID"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"

run() {{
  echo
  echo "$ $*"
  timeout -k 2 8 bash -lc "$*"
  code=$?
  echo "returncode: $code"
}}

source_ros() {{
  source /opt/ros/jazzy/setup.bash || return 10
  [ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
  export ROS_DOMAIN_ID={domain}
  export ROS_LOCALHOST_ONLY={localhost_only}
  export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
  export ROS2CLI_NO_DAEMON=1
  export TURTLEBOT3_MODEL="${{TURTLEBOT3_MODEL:-burger}}"
  export LDS_MODEL="${{LDS_MODEL:-LDS-03}}"
}}

echo "== robot ssh diagnostics =="
echo "generatedAt: $(date -Is)"
echo "mode: $([ "$DETAILED" = "1" ] && echo detailed || echo quick)"
echo "host: $(hostname)"
echo "user: $(whoami)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"
source_ros || {{ echo "[FAIL] ROS2 Jazzy setup.bash not found"; exit 10; }}
echo "after source: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"

run 'ip -brief addr || true'
run 'date -Is; date +%s%N; timedatectl show -p NTPSynchronized -p NTP 2>/dev/null || true'
run 'pgrep -af "turtlebot3|robot_state_publisher|diff_drive|lidar|ld08|amcl|nav2|map_server|controller_server|planner_server|bt_navigator|collision_monitor|camera|realsense|usb_cam|v4l2|libcamera" | grep -v pgrep || true'
run 'tmux ls || true'
run 'ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true'
run 'for d in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$d" ] || continue; echo "-- $d"; udevadm info -q property -n "$d" 2>/dev/null | grep -E "DEVNAME|ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH" || true; done'
run 'ls -l "$HOME/maps/dashboard" "$HOME/maps" 2>/dev/null || true'
run 'sed -n "1,80p" "$HOME/maps/dashboard/dashboard_map.yaml" 2>/dev/null || true'
run 'for s in tb3_base tb3_nav2 tb3_camera bringup ros_tcp slam camera nav2; do if tmux has-session -t "$s" 2>/dev/null; then echo "== tmux $s =="; tmux capture-pane -pt "$s" -S -140 2>/dev/null || true; fi; done'
run 'loginctl show-user "$USER" -p Linger 2>&1 || true; systemctl --user status turtlebot-dashboard-base.service --no-pager -l 2>&1 || true'
run 'for f in "$HOME"/turtlebot_dashboard_logs/*.log "$HOME"/turtlebot_dashboard_logs/*.sh; do [ -e "$f" ] || continue; echo "== $f =="; tail -100 "$f" 2>/dev/null || true; done'
run 'for pkg in turtlebot3_bringup turtlebot3_navigation2 nav2_bringup camera_ros realsense2_camera; do echo "-- $pkg"; ros2 pkg prefix "$pkg" 2>&1 || true; done'

run 'ROS2CLI_NO_DAEMON=1 ros2 node list | sort'

echo
echo "$ ros2 topic list | sort"
ALL_TOPICS="$(timeout -k 2 8 ros2 topic list 2>/dev/null || true)"
printf '%s\n' "$ALL_TOPICS" | sort
echo "returncode: 0"

echo
echo "$ ros2 action list | sort"
timeout -k 2 8 ros2 action list 2>/dev/null | sort || true
echo "returncode: 0"

echo
echo "$ ros2 action info {action_name_q}"
timeout -k 2 8 ros2 action info {action_name_q} || true

echo
echo "$ ros2 action info {route_action_name_q}"
timeout -k 2 8 ros2 action info {route_action_name_q} || true

if [ "$DETAILED" != "1" ]; then
  echo
  echo "[INFO] Quick SSH check skipped topic info -v, topic hz, cmd_vel echo, and lifecycle loops."
  echo "[INFO] Use detailed SSH check or diagnostics copy when deeper ROS CLI output is needed."
  exit 0
fi

TOPICS=({topic_array})
for topic in "${{TOPICS[@]}}"; do
  if printf '%s\n' "$ALL_TOPICS" | grep -qx "$topic"; then
    echo
    echo "$ ros2 topic info $topic -v"
    timeout -k 2 6 ros2 topic info "$topic" -v || true
  else
    echo
    echo "$ ros2 topic info $topic -v"
    echo "topic not found"
  fi
done

HZ_TOPICS=({hz_topic_array})
for topic in "${{HZ_TOPICS[@]}}"; do
  if printf '%s\n' "$ALL_TOPICS" | grep -qx "$topic"; then
    echo
    echo "$ ros2 topic hz $topic --window 5"
    timeout -k 2 6 ros2 topic hz "$topic" --window 5 || true
  fi
done

if printf '%s\n' "$ALL_TOPICS" | grep -qx {shlex.quote(str(topics.get("cmdVel") or "/cmd_vel"))}; then
  echo
  echo "$ ros2 topic echo {shlex.quote(str(topics.get("cmdVel") or "/cmd_vel"))} --once"
  timeout -k 2 6 ros2 topic echo {shlex.quote(str(topics.get("cmdVel") or "/cmd_vel"))} --once || true
fi

LIFECYCLE_NODES=({lifecycle_array})
for node in "${{LIFECYCLE_NODES[@]}}"; do
  echo
  echo "$ ros2 lifecycle get $node"
  timeout -k 2 5 ros2 lifecycle get "$node" || true
done
"""


def build_cmd_vel_delivery_check_script(
    network: Dict[str, Any], topic: str, message_type: str
) -> str:
    if message_type not in (TWIST_TYPE, TWIST_STAMPED_TYPE):
        raise ValueError(f"unsupported cmd_vel message type: {message_type}")
    domain = shlex.quote(str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"))
    localhost_only = shlex.quote(str(network.get("rosLocalhostOnly") or "0"))
    topic_q = shlex.quote(str(topic or "/cmd_vel"))
    message_type_q = shlex.quote(message_type)
    return f"""#!/usr/bin/env bash
set +e
source /opt/ros/jazzy/setup.bash || exit 10
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export RMW_IMPLEMENTATION="${{RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${{ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}}"
export ROS2CLI_NO_DAEMON=1

echo "== dashboard cmd_vel delivery check =="
echo "robotTime: $(date -Is)"
echo "robotEpochNs: $(date +%s%N 2>/dev/null || true)"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY RMW=$RMW_IMPLEMENTATION"
echo "topic: {topic_q}"
echo "messageType: {message_type_q}"
echo "DASHBOARD_CMD_VEL_WAITING"
timeout -k 2 10 ros2 topic echo {topic_q} {message_type_q} --once
code=$?
echo "DASHBOARD_CMD_VEL_ECHO_RC=$code"
exit "$code"
"""


def build_robot_hardware_check_script(network: Dict[str, Any], topics: Dict[str, Any]) -> str:
    """Return a machine-readable OpenCR/base-driver health probe for one robot."""
    domain = shlex.quote(str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"))
    localhost_only = shlex.quote(str(network.get("rosLocalhostOnly") or "0"))
    odom_topic = shlex.quote(str(topics.get("odom") or "/odom"))
    cmd_vel_topic = shlex.quote(str(topics.get("cmdVel") or "/cmd_vel"))
    return f"""#!/usr/bin/env bash
set +e
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export ROS2CLI_NO_DAEMON=1
USER_UID="$(id -u)"
export XDG_RUNTIME_DIR="/run/user/$USER_UID"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"

is_opencr_port() {{
  local port="$1" properties
  [ -c "$port" ] || return 1
  properties="$(udevadm info -q property -n "$port" 2>/dev/null || true)"
  printf '%s\n' "$properties" | grep -Eiq 'Arduino|Uno' && return 1
  if printf '%s\n' "$properties" | grep -Eiq '^ID_VENDOR_ID=0483$'; then
    printf '%s\n' "$properties" | grep -Eiq '^ID_MODEL_ID=5740$' && return 0
  fi
  printf '%s\n' "$properties" | grep -Eiq '(^ID_MODEL(_ENC)?=.*OpenCR|^ID_SERIAL(_SHORT)?=.*OpenCR)'
}}

OPENCR_PORT=""
ARDUINO_PORT=""
UNKNOWN_PORTS=""
shopt -s nullglob
for port in /dev/ttyACM*; do
  properties="$(udevadm info -q property -n "$port" 2>/dev/null || true)"
  if is_opencr_port "$port"; then
    OPENCR_PORT="$port"
    break
  elif printf '%s\n' "$properties" | grep -Eiq 'Arduino|Uno'; then
    ARDUINO_PORT="${{ARDUINO_PORT:+$ARDUINO_PORT,}}$port"
  else
    UNKNOWN_PORTS="${{UNKNOWN_PORTS:+$UNKNOWN_PORTS,}}$port"
  fi
done

OPENCR_STATE="absent"
OPENCR_DETAIL="No verified OpenCR CDC device was found."
if [ -n "$OPENCR_PORT" ]; then
  OPENCR_STATE="verified"
  OPENCR_DETAIL="Verified OpenCR CDC device (STM32 0483:5740 or OpenCR identity)."
elif command -v lsusb >/dev/null 2>&1 && lsusb -d 0483:df11 >/dev/null 2>&1; then
  OPENCR_STATE="bootloader"
  OPENCR_DETAIL="OpenCR is in STM32 DFU bootloader mode; TurtleBot firmware is not running."
elif [ -n "$ARDUINO_PORT" ]; then
  OPENCR_STATE="arduino_only"
  OPENCR_DETAIL="Arduino device found, but no verified OpenCR. Arduino is never used as the TurtleBot base port."
elif [ -n "$UNKNOWN_PORTS" ]; then
  OPENCR_STATE="unknown_acm"
  OPENCR_DETAIL="Unknown ACM serial device found; it is not accepted as OpenCR."
fi

ROS_READY=0
TURTLEBOT_NODE="missing"
BASE_SERVICE="unknown"
ODOM_PUBLISHERS=0
CMD_VEL_SUBSCRIBERS=0
if source /opt/ros/jazzy/setup.bash 2>/dev/null; then
  [ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
  ROS_READY=1
  systemctl --user is-active --quiet turtlebot-dashboard-base.service && BASE_SERVICE="active" || BASE_SERVICE="inactive"
  NODES="$(timeout -k 1 5 ros2 node list 2>/dev/null || true)"
  printf '%s\n' "$NODES" | grep -Eq '(^|/)turtlebot3_node$' && TURTLEBOT_NODE="present"
  ODOM_INFO="$(timeout -k 1 5 ros2 topic info {odom_topic} -v 2>/dev/null || true)"
  CMD_INFO="$(timeout -k 1 5 ros2 topic info {cmd_vel_topic} -v 2>/dev/null || true)"
  ODOM_PUBLISHERS="$(printf '%s\n' "$ODOM_INFO" | sed -n 's/^Publisher count: *//p' | head -1)"
  CMD_VEL_SUBSCRIBERS="$(printf '%s\n' "$CMD_INFO" | sed -n 's/^Subscription count: *//p' | head -1)"
fi
[[ "$ODOM_PUBLISHERS" =~ ^[0-9]+$ ]] || ODOM_PUBLISHERS=0
[[ "$CMD_VEL_SUBSCRIBERS" =~ ^[0-9]+$ ]] || CMD_VEL_SUBSCRIBERS=0
READY="false"
if [ "$OPENCR_STATE" = "verified" ] && [ "$TURTLEBOT_NODE" = "present" ] && [ "$ODOM_PUBLISHERS" -gt 0 ] && [ "$CMD_VEL_SUBSCRIBERS" -gt 0 ]; then
  READY="true"
fi

echo "DASHBOARD_OPENCR_STATE=$OPENCR_STATE"
echo "DASHBOARD_OPENCR_PORT=$OPENCR_PORT"
echo "DASHBOARD_OPENCR_ARDUINO_PORTS=$ARDUINO_PORT"
echo "DASHBOARD_OPENCR_UNKNOWN_PORTS=$UNKNOWN_PORTS"
echo "DASHBOARD_OPENCR_ROS_READY=$ROS_READY"
echo "DASHBOARD_OPENCR_BASE_SERVICE=$BASE_SERVICE"
echo "DASHBOARD_OPENCR_NODE=$TURTLEBOT_NODE"
echo "DASHBOARD_OPENCR_ODOM_PUBLISHERS=$ODOM_PUBLISHERS"
echo "DASHBOARD_OPENCR_CMD_VEL_SUBSCRIBERS=$CMD_VEL_SUBSCRIBERS"
echo "DASHBOARD_OPENCR_READY=$READY"
echo "DASHBOARD_OPENCR_DETAIL=$OPENCR_DETAIL"
"""


def parse_robot_hardware_check_output(output: Any) -> Dict[str, Any]:
    text = str(output or "")

    def marker(name: str, default: str = "") -> str:
        match = re.search(rf"^DASHBOARD_OPENCR_{name}=(.*)$", text, re.MULTILINE)
        return match.group(1).strip() if match else default

    def marker_int(name: str) -> int:
        try:
            return max(0, int(marker(name, "0")))
        except (TypeError, ValueError):
            return 0

    state = marker("STATE", "unknown")
    connected = state == "verified"
    ready = marker("READY").lower() == "true"
    return {
        "state": state,
        "connected": connected,
        "ready": ready,
        "port": marker("PORT") or None,
        "arduinoPorts": [value for value in marker("ARDUINO_PORTS").split(",") if value],
        "unknownPorts": [value for value in marker("UNKNOWN_PORTS").split(",") if value],
        "rosReady": marker("ROS_READY") == "1",
        "baseService": marker("BASE_SERVICE", "unknown"),
        "turtlebotNode": marker("NODE", "missing") == "present",
        "odomPublishers": marker_int("ODOM_PUBLISHERS"),
        "cmdVelSubscribers": marker_int("CMD_VEL_SUBSCRIBERS"),
        "detail": marker("DETAIL", "OpenCR status markers were not returned."),
        "checkedAt": now_iso(),
    }


def run_ssh_script(network: Dict[str, Any], script: str, timeout: float = 70.0) -> Dict[str, Any]:
    host = str(network.get("robotSshHost") or network.get("robotIp") or "").strip()
    user = str(network.get("robotSshUser") or "").strip()
    password = str(network.get("robotSshPassword") or "")
    if not host or not user:
        return {
            "ok": False,
            "returncode": None,
            "stdout": "",
            "stderr": "Robot SSH host/user is not configured.",
        }
    target = f"{user}@{host}"
    encoded_script = base64.b64encode(script.encode("utf-8")).decode("ascii")
    remote_command = f"printf %s {shlex.quote(encoded_script)} | base64 -d | bash"
    ssh_args = [
        "ssh",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "ConnectTimeout=8",
        "-o",
        "ServerAliveInterval=10",
        "-o",
        "ServerAliveCountMax=2",
        target,
        remote_command,
    ]
    command_label = f"ssh {target} <dashboard_bringup_script>"

    if password and shutil.which("sshpass"):
        env = {**os.environ, "SSHPASS": password}
        try:
            completed = subprocess.run(
                ["sshpass", "-e", *ssh_args],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=timeout,
                env=env,
            )
        except subprocess.TimeoutExpired as exc:
            return {
                "ok": False,
                "command": command_label,
                "returncode": None,
                "stdout": redact_secret(_text_output(exc.stdout), password),
                "stderr": redact_secret(f"timeout after {timeout:.1f}s\n{_text_output(exc.stderr)}".strip(), password),
            }
        stdout = redact_secret(completed.stdout or "", password)
        stderr = redact_secret(completed.stderr or "", password)
        return {
            "ok": completed.returncode == 0,
            "command": command_label,
            "returncode": completed.returncode,
            "stdout": stdout.strip(),
            "stderr": stderr.strip(),
        }

    if password and os.name == "posix":
        return run_ssh_script_with_pty(ssh_args, password, command_label, timeout)

    no_password_args = [*ssh_args[:1], "-o", "BatchMode=yes", *ssh_args[1:]]
    try:
        completed = subprocess.run(
            no_password_args,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "ok": False,
            "command": command_label,
            "returncode": None,
            "stdout": _text_output(exc.stdout),
            "stderr": f"timeout after {timeout:.1f}s\n{_text_output(exc.stderr)}".strip(),
        }
    return {
        "ok": completed.returncode == 0,
        "command": command_label,
        "returncode": completed.returncode,
        "stdout": (completed.stdout or "").strip(),
        "stderr": (completed.stderr or "").strip(),
    }


def run_ssh_script_with_pty(
    ssh_args: list[str], password: str, command_label: str, timeout: float
) -> Dict[str, Any]:
    try:
        import pty
        import select
        import signal
    except Exception as exc:
        return {
            "ok": False,
            "command": command_label,
            "returncode": None,
            "stdout": "",
            "stderr": f"Password SSH needs sshpass or POSIX pty support: {exc}",
        }

    output = bytearray()
    start = time.time()
    password_sends = 0
    pid, fd = pty.fork()
    if pid == 0:
        os.execvp(ssh_args[0], ssh_args)

    returncode: Optional[int] = None
    try:
        while True:
            if time.time() - start > timeout:
                try:
                    os.kill(pid, signal.SIGTERM)
                except Exception:
                    pass
                returncode = None
                output.extend(f"\ntimeout after {timeout:.1f}s\n".encode("utf-8"))
                break
            ready, _, _ = select.select([fd], [], [], 0.2)
            if fd in ready:
                try:
                    chunk = os.read(fd, 4096)
                except OSError:
                    chunk = b""
                if chunk:
                    output.extend(chunk)
                    tail = bytes(output[-4096:]).lower()
                    if password_sends < 3 and (b"password:" in tail or b"password for" in tail):
                        os.write(fd, (password + "\n").encode("utf-8"))
                        password_sends += 1
                else:
                    pass
            try:
                done_pid, status = os.waitpid(pid, os.WNOHANG)
                if done_pid == pid:
                    returncode = os.waitstatus_to_exitcode(status)
                    break
            except ChildProcessError:
                break
    finally:
        try:
            os.close(fd)
        except Exception:
            pass

    text = redact_secret(output.decode("utf-8", errors="replace"), password).strip()
    return {
        "ok": returncode == 0,
        "command": command_label,
        "returncode": returncode,
        "stdout": text,
        "stderr": "" if returncode == 0 else text,
    }


def run_robot_bringup_from_state(state: "DashboardState") -> Dict[str, Any]:
    setup = state.get_setup()
    robot_id, profile = active_robot_profile(setup)
    lock = robot_operation_lock(robot_id)
    if not lock.acquire(blocking=False):
        return {
            "ok": False,
            "error": "A bringup or stop operation is already in progress for this robot. Wait for it to finish.",
        }
    try:
        return _run_robot_bringup_from_state(state, setup=setup, robot_id=robot_id, profile=profile)
    finally:
        lock.release()


def _run_robot_bringup_from_state(
    state: "DashboardState",
    setup: Optional[Dict[str, Any]] = None,
    robot_id: Optional[str] = None,
    profile: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    setup = setup or state.get_setup()
    robot_id, selected_profile = active_robot_profile(setup) if robot_id is None else (robot_id, profile or {})
    network = active_robot_network(setup)
    map_package: Optional[Dict[str, Any]] = None
    map_error = ""
    try:
        map_package = build_dashboard_map_package(setup)
    except Exception as exc:
        map_error = f"{type(exc).__name__}: {exc}"
    script = build_robot_bringup_script(network, map_package)
    started_at = now_iso()
    result = run_ssh_script(network, script)
    stdout = result.get("stdout", "")
    if map_error:
        stdout = f"[WARN] dashboard map export failed: {map_error}\n{stdout}".strip()
    state.update_runtime(
        {
            "navStatus": "robot_bringup_started" if result.get("ok") else "robot_bringup_failed",
            "navMessage": (
                "Robot SSH bringup commands were dispatched. Run Robot Check or SSH Check after nodes start."
                if result.get("ok")
                else "Robot SSH bringup failed."
            ),
            "lastRobotBringupAt": started_at,
            "lastRobotBringupMap": {
                "source": map_package.get("source"),
                "width": map_package.get("width"),
                "height": map_package.get("height"),
                "resolution": map_package.get("resolution"),
            }
            if map_package
            else None,
        }
    )
    return {
        "ok": bool(result.get("ok")),
        "robotId": robot_id,
        "robotLabel": selected_profile.get("label") or robot_id,
        "startedAt": started_at,
        "map": {
            "ok": map_package is not None,
            "source": map_package.get("source") if map_package else None,
            "width": map_package.get("width") if map_package else None,
            "height": map_package.get("height") if map_package else None,
            "resolution": map_package.get("resolution") if map_package else None,
            "error": map_error,
        },
        "command": result.get("command"),
        "returncode": result.get("returncode"),
        "stdout": stdout,
        "stderr": result.get("stderr", ""),
    }


def build_robot_bringup_stop_script(network: Dict[str, Any], topics: Dict[str, Any]) -> str:
    domain = shlex.quote(str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"))
    localhost_only = shlex.quote(str(network.get("rosLocalhostOnly") or "0"))
    cmd_vel_topic = str(topics.get("cmdVel") or "/cmd_vel").strip() or "/cmd_vel"
    cmd_vel = shlex.quote(cmd_vel_topic)
    cmd_vel_namespace = namespace_from_topic(cmd_vel_topic, "/cmd_vel") or ""
    cmd_vel_nav = shlex.quote(namespace_topic(cmd_vel_namespace, "/cmd_vel_nav"))
    scan = shlex.quote(str(topics.get("scan") or "/scan"))
    camera = shlex.quote(str(topics.get("camera") or "/camera/image_raw"))
    compressed_camera = shlex.quote(
        str(topics.get("compressedCamera") or "/camera/image_raw/compressed")
    )
    goal_action = str(topics.get("goalAction") or "/navigate_to_pose").strip() or "/navigate_to_pose"
    nav_namespace = namespace_from_topic(goal_action, "/navigate_to_pose") or ""
    navigation_manager = shlex.quote(
        namespace_topic(nav_namespace, "/lifecycle_manager_navigation")
    )
    localization_manager = shlex.quote(
        namespace_topic(nav_namespace, "/lifecycle_manager_localization")
    )
    return f"""#!/usr/bin/env bash
set +e
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS_DOMAIN_ID={domain}
export ROS_LOCALHOST_ONLY={localhost_only}
export ROS2CLI_NO_DAEMON=1
BASE_SERVICE="turtlebot-dashboard-base.service"
LOG_DIR="$HOME/turtlebot_dashboard_logs"
NAVIGATION_MANAGER={navigation_manager}
LOCALIZATION_MANAGER={localization_manager}
CMD_VEL={cmd_vel}
CMD_VEL_NAV={cmd_vel_nav}
USER_UID="$(id -u)"
export XDG_RUNTIME_DIR="/run/user/$USER_UID"
export DBUS_SESSION_BUS_ADDRESS="unix:path=$XDG_RUNTIME_DIR/bus"
remaining=0

publish_zero_velocity() {{
  local topic="$1"
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/Twist "{{linear: {{x: 0.0}}, angular: {{z: 0.0}}}}" >/dev/null 2>&1 || true
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/TwistStamped "{{twist: {{linear: {{x: 0.0}}, angular: {{z: 0.0}}}}}}" >/dev/null 2>&1 || true
}}

shutdown_nav2_lifecycle_manager() {{
  local manager="$1"
  local response
  response="$(timeout -k 1 8 ros2 service call "$manager/manage_nodes" nav2_msgs/srv/ManageLifecycleNodes "{{command: 4}}" 2>&1)"
  if printf '%s\n' "$response" | grep -Eiq 'success[=:] *true'; then
    echo "[STOP] Nav2 lifecycle shutdown: $manager"
    return 0
  fi
  echo "[INFO] Nav2 lifecycle manager unavailable or already stopped: $manager"
  return 1
}}

stop_dashboard_tmux_session() {{
  local session="$1"
  local attempt
  if ! tmux has-session -t "$session" 2>/dev/null; then
    return 0
  fi
  if tmux send-keys -t "$session" C-c >/dev/null 2>&1; then
    echo "[STOP] Ctrl+C sent to dashboard tmux $session"
  else
    echo "[WARN] Could not send Ctrl+C to dashboard tmux $session"
  fi
  for attempt in $(seq 1 12); do
    if ! tmux has-session -t "$session" 2>/dev/null; then
      echo "[OK] tmux session exited after Ctrl+C: $session"
      return 0
    fi
    sleep 1
  done
  echo "[WARN] tmux session did not exit after Ctrl+C: $session"
  tmux kill-session -t "$session" >/dev/null 2>&1 || true
  echo "[FALLBACK] tmux session force-closed: $session"
  return 1
}}

stop_dashboard_nohup_process() {{
  local name="$1"
  local pid_file="$LOG_DIR/${{name}}.pid"
  local pid
  local attempt
  if [ ! -s "$pid_file" ]; then
    return 0
  fi
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if ! [[ "$pid" =~ ^[0-9]+$ ]] || ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    return 0
  fi
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  echo "[STOP] SIGINT sent to dashboard nohup process group $name (pid $pid)"
  for attempt in $(seq 1 12); do
    if ! kill -0 "$pid" 2>/dev/null; then
      rm -f "$pid_file"
      echo "[OK] nohup process exited after SIGINT: $name"
      return 0
    fi
    sleep 1
  done
  echo "[WARN] nohup process did not exit after SIGINT: $name"
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  sleep 3
  if kill -0 "$pid" 2>/dev/null; then
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    echo "[FALLBACK] nohup process group force-closed: $name"
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
  fi
  return 1
}}

echo "== dashboard robot bringup stop =="
publish_zero_velocity "$CMD_VEL"
publish_zero_velocity "$CMD_VEL_NAV"

# Nav2 needs its lifecycle state machine to shut down before the launch process exits.
shutdown_nav2_lifecycle_manager "$NAVIGATION_MANAGER" || true
shutdown_nav2_lifecycle_manager "$LOCALIZATION_MANAGER" || true
stop_dashboard_tmux_session tb3_nav2 || true
stop_dashboard_tmux_session tb3_camera || true
stop_dashboard_nohup_process tb3_nav2 || true
stop_dashboard_nohup_process tb3_camera || true

if ! systemctl --user show-environment >/dev/null 2>&1; then
  echo "[FAIL] systemd user manager is unavailable; base service stop cannot be verified."
  remaining=1
elif systemctl --user is-active --quiet "$BASE_SERVICE"; then
  systemctl --user stop "$BASE_SERVICE"
  echo "[STOP] systemd $BASE_SERVICE (SIGINT control group)"
else
  echo "[INFO] $BASE_SERVICE is not active"
fi

# Compatibility cleanup for an older dashboard-managed tmux base session.
# A manually launched terminal session is intentionally not targeted here.
stop_dashboard_tmux_session tb3_base || true

echo "== dashboard bringup stop verification =="
if ! systemctl --user show-environment >/dev/null 2>&1; then
  echo "[FAIL] systemd user manager is unavailable during verification"
  remaining=1
elif systemctl --user is-active --quiet "$BASE_SERVICE"; then
  echo "[FAIL] systemd service still active: $BASE_SERVICE"
  systemctl --user status "$BASE_SERVICE" --no-pager -l 2>&1 || true
  remaining=1
else
  echo "[OK] systemd service stopped: $BASE_SERVICE"
fi
for session in tb3_nav2 tb3_camera; do
  if tmux has-session -t "$session" 2>/dev/null; then
    echo "[FAIL] tmux session still active: $session"
    remaining=1
  else
    echo "[OK] tmux session stopped: $session"
  fi
done
nav2_processes="$(pgrep -af '[m]ap_server|[a]mcl|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[c]ollision_monitor|[l]ifecycle_manager_(navigation|localization)|[t]urtlebot3_navigation2|[n]av2_bringup' || true)"
if [ -n "$nav2_processes" ]; then
  echo "[FAIL] Nav2 processes still active (not force-killed):"
  echo "$nav2_processes"
  remaining=1
else
  echo "[OK] Nav2 processes stopped"
fi
camera_processes="$(pgrep -af '[c]amera_ros|[c]amera_node|[c]amera.launch.py|[r]ealsense2_camera|[u]sb_cam|[v]4l2_camera' || true)"
if [ -n "$camera_processes" ]; then
  echo "[FAIL] Camera processes still active (not force-killed):"
  echo "$camera_processes"
  remaining=1
else
  echo "[OK] Camera processes stopped"
fi
scan_info="$(timeout -k 1 4 ros2 topic info {cmd_vel} -v 2>/dev/null || true)"
cmd_subscribers="$(printf '%s\n' "$scan_info" | sed -n 's/^Subscription count: *//p' | head -1)"
echo "[INFO] {cmd_vel} subscribers after stop: ${{cmd_subscribers:-unknown}}"
scan_info="$(timeout -k 1 4 ros2 topic info {scan} -v 2>/dev/null || true)"
scan_publishers="$(printf '%s\n' "$scan_info" | sed -n 's/^Publisher count: *//p' | head -1)"
echo "[INFO] {scan} publishers after stop: ${{scan_publishers:-unknown}}"
if [ -n "$scan_publishers" ] && [ "$scan_publishers" != "0" ]; then
  echo "[FAIL] {scan} still has publishers. It may be a manually started or external LiDAR driver."
  remaining=1
fi
camera_info="$(timeout -k 1 4 ros2 topic info {camera} -v 2>/dev/null || true)"
camera_publishers="$(printf '%s\n' "$camera_info" | sed -n 's/^Publisher count: *//p' | head -1)"
echo "[INFO] {camera} publishers after stop: ${{camera_publishers:-unknown}}"
if [ -n "$camera_publishers" ] && [ "$camera_publishers" != "0" ]; then
  echo "[FAIL] {camera} still has publishers"
  remaining=1
fi
compressed_camera_info="$(timeout -k 1 4 ros2 topic info {compressed_camera} -v 2>/dev/null || true)"
compressed_camera_publishers="$(printf '%s\n' "$compressed_camera_info" | sed -n 's/^Publisher count: *//p' | head -1)"
echo "[INFO] {compressed_camera} publishers after stop: ${{compressed_camera_publishers:-unknown}}"
if [ -n "$compressed_camera_publishers" ] && [ "$compressed_camera_publishers" != "0" ]; then
  echo "[FAIL] {compressed_camera} still has publishers"
  remaining=1
fi
if [ "$remaining" -ne 0 ]; then
  echo "[FAIL] dashboard bringup stop verification failed"
  exit 22
fi
echo "[OK] dashboard bringup stopped and verified"
"""


def dashboard_host_stop_script_template() -> str:
    """Return the safe fallback script stored at the dashboard repository top level."""
    return """#!/usr/bin/env bash
# TurtleBot Dashboard host-side shutdown helper.
# It is generated only when this repository has no top-level stop_all.sh.
set +e

# DASHBOARD_MANAGED_CONFIG_BEGIN
DASHBOARD_CMD_VEL=/cmd_vel
DASHBOARD_CMD_VEL_NAV=/cmd_vel_nav
DASHBOARD_SCAN=/scan
DASHBOARD_ODOM=/odom
DASHBOARD_CAMERA=/camera/image_raw
DASHBOARD_COMPRESSED_CAMERA=/camera/image_raw/compressed
DASHBOARD_NAVIGATION_MANAGER=/lifecycle_manager_navigation
DASHBOARD_LOCALIZATION_MANAGER=/lifecycle_manager_localization
# DASHBOARD_MANAGED_CONFIG_END

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
export ROS2CLI_NO_DAEMON=1
CMD_VEL="${DASHBOARD_CMD_VEL:-/cmd_vel}"
CMD_VEL_NAV="${DASHBOARD_CMD_VEL_NAV:-/cmd_vel_nav}"

publish_zero_velocity() {
  local topic="$1"
  command -v ros2 >/dev/null 2>&1 || return 0
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout -k 1 3 ros2 topic pub --once "$topic" geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1 || true
}

stop_helper() {
  local pattern="$1"
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  echo "[STOP] host helper: $pattern"
  pkill -INT -f "$pattern" >/dev/null 2>&1 || true
  sleep 1
  pgrep -f "$pattern" >/dev/null 2>&1 || return 0
  pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
}

echo "== dashboard host stop_all.sh =="
echo "[INFO] saved cmd_vel: $CMD_VEL"
echo "[INFO] saved cmd_vel_nav: $CMD_VEL_NAV"
echo "[INFO] official TurtleBot3 topics: scan=$DASHBOARD_SCAN odom=$DASHBOARD_ODOM camera=$DASHBOARD_CAMERA"
echo "[INFO] Nav2 lifecycle: $DASHBOARD_NAVIGATION_MANAGER, $DASHBOARD_LOCALIZATION_MANAGER"
publish_zero_velocity "$CMD_VEL"
publish_zero_velocity "$CMD_VEL_NAV"
for pattern in \
  'robot_yolo_viewer.py' \
  'view_yolo' \
  'view_geng_camera' \
  'rqt_image_view' \
  'webcam_yolo_preview' \
  'webcam_aihub_yolov5' \
  'launch_robot_test.sh' \
  'run_patrol.py' \
  'run_patrol_continuous.py' \
  'run_patrol.sh' \
  'park_.*\\.py' \
  'aruco_align_.*\\.py' \
  'rotate_t1_precise.py' \
  'rotate_map_yaw.py' \
  'dock_.*' \
  'go_nav2.sh' \
  'go_nav2_geng_rpp' \
  'nav2_rviz.sh' \
  'start_geng_camera'; do
  stop_helper "$pattern"
done
echo "[OK] dashboard host helper stop completed"
"""


def dashboard_host_stop_script_config(network: Dict[str, Any], topics: Dict[str, Any]) -> str:
    """Render the persistent topic values consumed by the managed host script."""
    cmd_vel_topic = str(topics.get("cmdVel") or "/cmd_vel").strip() or "/cmd_vel"
    cmd_vel_namespace = namespace_from_topic(cmd_vel_topic, "/cmd_vel") or ""
    goal_action = str(topics.get("goalAction") or "/navigate_to_pose").strip() or "/navigate_to_pose"
    nav_namespace = namespace_from_topic(goal_action, "/navigate_to_pose") or ""
    return "\n".join(
        (
            "# DASHBOARD_MANAGED_CONFIG_BEGIN",
            f"DASHBOARD_CMD_VEL={shlex.quote(cmd_vel_topic)}",
            f"DASHBOARD_CMD_VEL_NAV={shlex.quote(namespace_topic(cmd_vel_namespace, '/cmd_vel_nav'))}",
            f"DASHBOARD_SCAN={shlex.quote(str(topics.get('scan') or '/scan'))}",
            f"DASHBOARD_ODOM={shlex.quote(str(topics.get('odom') or '/odom'))}",
            f"DASHBOARD_CAMERA={shlex.quote(str(topics.get('camera') or '/camera/image_raw'))}",
            f"DASHBOARD_COMPRESSED_CAMERA={shlex.quote(str(topics.get('compressedCamera') or '/camera/image_raw/compressed'))}",
            f"DASHBOARD_NAVIGATION_MANAGER={shlex.quote(namespace_topic(nav_namespace, '/lifecycle_manager_navigation'))}",
            f"DASHBOARD_LOCALIZATION_MANAGER={shlex.quote(namespace_topic(nav_namespace, '/lifecycle_manager_localization'))}",
            "# DASHBOARD_MANAGED_CONFIG_END",
        )
    )


def update_dashboard_host_stop_script_config(
    script_path: Path, network: Dict[str, Any], topics: Dict[str, Any]
) -> Dict[str, Any]:
    """Update only the marked config block; never rewrite a user-owned script."""
    try:
        source = script_path.read_text(encoding="utf-8")
        block = dashboard_host_stop_script_config(network, topics)
        matcher = re.compile(
            r"(?ms)^# DASHBOARD_MANAGED_CONFIG_BEGIN\r?\n.*?^# DASHBOARD_MANAGED_CONFIG_END$"
        )
        if not matcher.search(source):
            return {
                "ok": True,
                "managed": False,
                "warning": "Existing stop_all.sh has no dashboard config block; keeping it unchanged.",
            }
        script_path.write_text(matcher.sub(block, source, count=1), encoding="utf-8")
        return {"ok": True, "managed": True}
    except OSError as exc:
        return {"ok": False, "error": f"Could not save stop_all.sh settings: {exc}"}


def ensure_dashboard_host_stop_script(path: Optional[Path] = None) -> Dict[str, Any]:
    """Find or create the editable top-level host stop script."""
    script_path = path or (ROOT / "stop_all.sh")
    try:
        if script_path.exists():
            if not script_path.is_file():
                return {"ok": False, "path": str(script_path), "error": "stop_all.sh is not a file."}
            return {"ok": True, "path": str(script_path), "created": False}
        script_path.write_text(dashboard_host_stop_script_template(), encoding="utf-8")
        try:
            script_path.chmod(script_path.stat().st_mode | 0o100)
        except OSError:
            pass
        return {"ok": True, "path": str(script_path), "created": True}
    except OSError as exc:
        return {"ok": False, "path": str(script_path), "error": f"Could not create stop_all.sh: {exc}"}


def run_dashboard_host_stop_script(network: Dict[str, Any], topics: Dict[str, Any]) -> Dict[str, Any]:
    """Run the repository-top-level helper before the robot SSH shutdown."""
    script_info = ensure_dashboard_host_stop_script()
    if not script_info.get("ok"):
        return {"ok": False, "stderr": str(script_info.get("error") or "stop_all.sh is unavailable.")}
    config_result = update_dashboard_host_stop_script_config(Path(script_info["path"]), network, topics)
    if not config_result.get("ok"):
        return {"ok": False, "stderr": str(config_result.get("error") or "Could not configure stop_all.sh.")}
    if os.name != "posix":
        return {
            "ok": True,
            "warning": f"Host stop script is ready at {script_info['path']}, but this Windows dashboard host cannot run Bash; continuing with robot SSH shutdown.",
        }
    bash_path = shutil.which("bash")
    if not bash_path:
        return {
            "ok": True,
            "warning": f"Host stop script is ready at {script_info['path']}, but Bash is unavailable; continuing with robot SSH shutdown.",
        }
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1"),
        "ROS_LOCALHOST_ONLY": str(network.get("rosLocalhostOnly") or "0"),
        "DASHBOARD_CMD_VEL": str(topics.get("cmdVel") or "/cmd_vel").strip() or "/cmd_vel",
        "DASHBOARD_CMD_VEL_NAV": namespace_topic(
            namespace_from_topic(str(topics.get("cmdVel") or "/cmd_vel").strip() or "/cmd_vel", "/cmd_vel") or "",
            "/cmd_vel_nav",
        ),
    }
    try:
        completed = subprocess.run(
            [bash_path, str(script_info["path"])],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=75.0,
            env=environment,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "stderr": "Host stop_all.sh timed out after 75 seconds."}
    result = {
        "ok": completed.returncode == 0,
        "stdout": (completed.stdout or "").strip(),
        "stderr": (completed.stderr or "").strip(),
    }
    if config_result.get("warning"):
        result["warning"] = config_result["warning"]
    return result


def run_robot_bringup_stop_from_state(state: "DashboardState") -> Dict[str, Any]:
    setup = state.get_setup()
    robot_id, profile = active_robot_profile(setup)
    lock = robot_operation_lock(robot_id)
    if not lock.acquire(blocking=False):
        return {
            "ok": False,
            "error": "A bringup or stop operation is already in progress for this robot. Wait for it to finish.",
        }
    try:
        return _run_robot_bringup_stop_from_state(state, setup=setup, robot_id=robot_id, profile=profile)
    finally:
        lock.release()


def _run_robot_bringup_stop_from_state(
    state: "DashboardState",
    setup: Optional[Dict[str, Any]] = None,
    robot_id: Optional[str] = None,
    profile: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    setup = setup or state.get_setup()
    robot_id, selected_profile = active_robot_profile(setup) if robot_id is None else (robot_id, profile or {})
    network = active_robot_network(setup)
    topics = selected_profile.get("topics") if isinstance(selected_profile.get("topics"), dict) else setup.get("topics", {})
    started_at = now_iso()
    local_stop = run_dashboard_host_stop_script(network, topics)
    result = run_ssh_script(
        network,
        build_robot_bringup_stop_script(network, topics),
        timeout=120.0,
    )
    local_stdout = str(local_stop.get("stdout") or "").strip()
    local_stderr = str(local_stop.get("stderr") or local_stop.get("warning") or "").strip()
    if local_stdout:
        result["stdout"] = f"== dashboard host stop_all.sh ==\n{local_stdout}\n\n== robot SSH stop ==\n{result.get('stdout', '')}".strip()
    if local_stderr:
        result["stderr"] = f"{local_stderr}\n{result.get('stderr', '')}".strip()
    result["ok"] = bool(result.get("ok")) and bool(local_stop.get("ok", True))
    state.update_runtime(
        {
            "navStatus": "robot_bringup_stopped" if result.get("ok") else "robot_bringup_stop_failed",
            "navMessage": (
                "Robot bringup stopped and verified."
                if result.get("ok")
                else "Robot bringup stop verification failed."
            ),
            "lastRobotBringupStopAt": started_at,
        }
    )
    return {**result, "stoppedAt": started_at, "robotId": robot_id, "robotLabel": selected_profile.get("label") or robot_id}


def run_robot_ssh_check_from_state(state: "DashboardState", detailed: bool = False) -> Dict[str, Any]:
    setup = state.get_setup()
    robot_id, profile = active_robot_profile(setup)
    network = active_robot_network(setup)
    topics = profile.get("topics") if isinstance(profile.get("topics"), dict) else setup.get("topics", {})
    script = build_robot_ssh_diagnostics_script(network, topics, detailed=detailed)
    started_at = now_iso()
    result = run_ssh_script(network, script, timeout=150.0 if detailed else 35.0)
    state.update_runtime(
        {
            "lastRobotSshCheckAt": started_at,
            "navMessage": (
                f"Robot SSH {'detailed ' if detailed else ''}diagnostics finished."
                if result.get("ok")
                else f"Robot SSH {'detailed ' if detailed else ''}diagnostics failed."
            ),
        }
    )
    return {
        "ok": bool(result.get("ok")),
        "robotId": robot_id,
        "robotLabel": profile.get("label") or robot_id,
        "detailed": detailed,
        "startedAt": started_at,
        "command": result.get("command"),
        "returncode": result.get("returncode"),
        "stdout": result.get("stdout", ""),
        "stderr": result.get("stderr", ""),
    }


def run_robot_hardware_check_from_state(state: "DashboardState") -> Dict[str, Any]:
    setup = state.get_setup()
    robot_id, profile = active_robot_profile(setup)
    network = active_robot_network(setup)
    topics = profile.get("topics") if isinstance(profile.get("topics"), dict) else setup.get("topics", {})
    started_at = now_iso()
    result = run_ssh_script(
        network,
        build_robot_hardware_check_script(network, topics),
        timeout=35.0,
    )
    hardware = parse_robot_hardware_check_output(result.get("stdout"))
    if not result.get("ok"):
        hardware.update(
            {
                "state": "ssh_error",
                "connected": False,
                "ready": False,
                "detail": result.get("stderr") or "Could not run the OpenCR check over SSH.",
                "checkedAt": started_at,
            }
        )
    state.update_runtime({"opencr": hardware, "lastOpencrCheckAt": started_at})
    return {
        "ok": bool(result.get("ok")),
        "robotId": robot_id,
        "robotLabel": profile.get("label") or robot_id,
        "startedAt": started_at,
        "hardware": hardware,
        "command": result.get("command"),
        "returncode": result.get("returncode"),
        "stdout": result.get("stdout", ""),
        "stderr": result.get("stderr", ""),
    }


def format_diagnostics_report(
    snapshot: Dict[str, Any],
    check: Dict[str, Any],
    graph_topics: Optional[list[str]] = None,
    nodes: Optional[list[str]] = None,
    actions: Optional[list[str]] = None,
    command_outputs: Optional[list[Dict[str, Any]]] = None,
    robot_ssh_output: Optional[Dict[str, Any]] = None,
) -> str:
    setup = snapshot.get("setup", {})
    runtime = snapshot.get("runtime", {})
    network = redacted_network(setup.get("network", {}))
    topics = setup.get("topics", {})
    active_robot = setup.get("activeRobot") or "-"
    # Diagnostics are frequently copied outside the robot network. Never include
    # stored SSH secrets in that export.
    robot_profiles = public_robot_profiles(setup.get("robotProfiles", {}))
    scan_topic = topics.get("scan") or "/scan"
    odom_topic = topics.get("odom") or "/odom"
    cmd_topic = topics.get("cmdVel") or "/cmd_vel"
    pose_topic = topics.get("pose") or "/amcl_pose"
    action_name = topics.get("goalAction") or "/navigate_to_pose"
    route_action_name = topics.get("routeAction") or "/navigate_through_poses"
    configured_domain = str(network.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or "1")
    configured_localhost = str(
        network.get("rosLocalhostOnly") or os.environ.get("ROS_LOCALHOST_ONLY") or "0"
    )
    connection = check.get("connection") or runtime.get("connection") or {}
    checks = check.get("checks", [])
    advice = check.get("advice", [])
    graph_topics = graph_topics if graph_topics is not None else check.get("graphTopics", [])
    nodes = nodes if nodes is not None else check.get("nodes", [])
    actions = actions if actions is not None else check.get("actions", [])
    command_outputs = command_outputs or []

    lines = [
        "# TurtleBot Dashboard Diagnostics",
        f"generatedAt: {now_iso()}",
        f"dashboardVersion: {APP_VERSION}",
        f"platform: {platform.platform()}",
        f"python: {platform.python_version()}",
        "",
        "## Summary",
        f"mode: {check.get('mode') or runtime.get('mode') or '-'}",
        f"ok: {check.get('ok')}",
        f"summary: {check.get('summary') or '-'}",
        f"navStatus: {runtime.get('navStatus') or '-'}",
        f"navMessage: {runtime.get('navMessage') or '-'}",
        f"rosBridgeError: {runtime.get('rosBridgeError') or '-'}",
        "",
        "## Environment",
        f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '-')}",
        f"ROS_LOCALHOST_ONLY: {os.environ.get('ROS_LOCALHOST_ONLY', '-')}",
        f"RMW_IMPLEMENTATION: {os.environ.get('RMW_IMPLEMENTATION', '-')}",
        "",
        "## Network Config",
        json.dumps(network, ensure_ascii=False, indent=2),
        "",
        "## Connection",
        json.dumps(connection, ensure_ascii=False, indent=2),
        "",
        "## Runtime",
        json.dumps(
            {
                "rosConnected": runtime.get("rosConnected"),
                "mode": runtime.get("mode"),
                "rosBridgeError": runtime.get("rosBridgeError"),
                "pose": runtime.get("pose"),
                "goal": runtime.get("goal"),
                "route": runtime.get("route"),
                "routeIndex": runtime.get("routeIndex"),
                "routeCheckpoint": runtime.get("routeCheckpoint"),
                "cameraEnabled": runtime.get("cameraEnabled"),
                "cameraStream": runtime.get("cameraStream"),
                "cameraFps": runtime.get("cameraFps"),
                "cameraBufferedFrames": runtime.get("cameraBufferedFrames"),
                "cameraFrameGapMs": runtime.get("cameraFrameGapMs"),
                "cameraSubscriptionTopic": runtime.get("cameraSubscriptionTopic"),
                "cameraSubscriptionQueueDepth": runtime.get("cameraSubscriptionQueueDepth"),
                "cameraExecutorThreads": runtime.get("cameraExecutorThreads"),
                "lastScanAt": runtime.get("lastScanAt"),
                "lastPoseAt": runtime.get("lastPoseAt"),
                "lastOdomAt": runtime.get("lastOdomAt"),
                "lastCameraAt": runtime.get("lastCameraAt"),
                "lastCommandAt": runtime.get("lastCommandAt"),
                "lastManualCommandAt": runtime.get("lastManualCommandAt"),
                "cmdVelMessageType": runtime.get("cmdVelMessageType"),
                "cmdVelStampSource": runtime.get("cmdVelStampSource"),
                "cmdVelClockSkewMs": runtime.get("cmdVelClockSkewMs"),
                "cmdVelClockAgeMs": runtime.get("cmdVelClockAgeMs"),
                "lastCmdVelDeliveryCheckAt": runtime.get("lastCmdVelDeliveryCheckAt"),
                "lastCmdVelDeliveryCheckOk": runtime.get("lastCmdVelDeliveryCheckOk"),
                "odomAnchor": runtime.get("odomAnchor"),
                "fallbackActive": runtime.get("fallbackActive"),
                "fallbackPathIndex": runtime.get("fallbackPathIndex"),
                "fallbackPathLength": runtime.get("fallbackPathLength"),
                "fallbackSpeedScale": runtime.get("fallbackSpeedScale"),
                "lidarMinClearance": runtime.get("lidarMinClearance"),
                "fallbackRecoveryPhase": runtime.get("fallbackRecoveryPhase"),
                "fallbackRecoveryAttempts": runtime.get("fallbackRecoveryAttempts"),
                "lidarPointCount": runtime.get("lidarPointCount"),
                "lidarFrame": runtime.get("lidarFrame"),
                "scanAgeMs": runtime.get("scanAgeMs"),
                "odomAgeMs": runtime.get("odomAgeMs"),
            },
            ensure_ascii=False,
            indent=2,
        ),
        "",
        "## Topic Config",
        f"activeRobot: {active_robot}",
        json.dumps(topics, ensure_ascii=False, indent=2),
        "",
        "## LiDAR Fallback Config",
        json.dumps(setup.get("fallbackNavigation", {}), ensure_ascii=False, indent=2),
        "",
        "## Robot Profiles",
        json.dumps(robot_profiles, ensure_ascii=False, indent=2),
        "",
        "## Robot Check",
    ]
    for item in checks:
        level = item.get("level") or ("ok" if item.get("ok") else "fail")
        marker = "OK" if level == "ok" else level.upper()
        lines.append(f"- [{marker}] {item.get('label') or item.get('id')}: {item.get('detail') or '-'}")
    if advice:
        lines.extend(["", "## Advice"])
        lines.extend(f"- {item}" for item in advice)
    lines.extend(
        [
            "",
            "## ROS2 CLI Command Outputs",
            "These commands are executed on the dashboard server host with the server process environment.",
        ]
    )
    for result in command_outputs:
        lines.extend(
            [
                "",
                f"$ {result.get('command') or '-'}",
                f"returncode: {result.get('returncode')}",
                "stdout:",
                result.get("stdout") or "-",
                "stderr:",
                result.get("stderr") or "-",
            ]
        )
    if robot_ssh_output:
        lines.extend(
            [
                "",
                "## Robot SSH Diagnostics",
                f"$ {robot_ssh_output.get('command') or '-'}",
                f"returncode: {robot_ssh_output.get('returncode')}",
                "stdout:",
                robot_ssh_output.get("stdout") or "-",
                "stderr:",
                robot_ssh_output.get("stderr") or "-",
            ]
        )
    lines.extend(
        [
            "",
            "## ROS Graph",
            f"topics({len(graph_topics or [])}):",
            "\n".join(graph_topics or ["-"]),
            "",
            f"nodes({len(nodes or [])}):",
            "\n".join(nodes or ["-"]),
            "",
            f"actions({len(actions or [])}):",
            "\n".join(actions or ["-"]),
            "",
            "## Manual Commands To Compare",
            f"server: export ROS_DOMAIN_ID={configured_domain} && export ROS_LOCALHOST_ONLY={configured_localhost} && ros2 topic list",
            f"server: ros2 topic info {cmd_topic} -v",
            f"server: ros2 topic info {scan_topic} -v",
            f"server: ros2 topic info {odom_topic} -v",
            f"server: ros2 topic info {pose_topic} -v",
            f"server: ros2 action info {action_name}",
            f"server: ros2 action info {route_action_name}",
            "server: ros2 topic info /tf -v",
            f"robot:  export ROS_DOMAIN_ID={configured_domain} && export ROS_LOCALHOST_ONLY={configured_localhost} && ros2 topic list",
            "",
        ]
    )
    return "\n".join(lines)


def sanitize_run_log_value(value: Any, depth: int = 0) -> Any:
    if depth > 5:
        return "<depth-limit>"
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, float):
        return round(value, 6) if math.isfinite(value) else str(value)
    if isinstance(value, str):
        return value[:2000]
    if isinstance(value, dict):
        sanitized: Dict[str, Any] = {}
        for key, item in list(value.items())[:100]:
            name = str(key)
            if "password" in name.lower() or "secret" in name.lower():
                sanitized[name] = "***"
            else:
                sanitized[name] = sanitize_run_log_value(item, depth + 1)
        return sanitized
    if isinstance(value, (list, tuple)):
        return [sanitize_run_log_value(item, depth + 1) for item in list(value)[:200]]
    return str(value)[:2000]


class RunLogStore:
    MAX_EVENTS = 4000
    MAX_FILE_BYTES = 5 * 1024 * 1024

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._events: deque[Dict[str, Any]] = deque(maxlen=self.MAX_EVENTS)
        self._sequence = 0
        self._file_bytes = 0
        self._file_limit_reached = False
        self._handle: Optional[Any] = None
        self.session_id = ""
        self.path = Path()
        self._start_session()

    def _start_session(self) -> None:
        RUN_LOG_ROOT.mkdir(exist_ok=True)
        self.session_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
        self.path = RUN_LOG_ROOT / f"dashboard_run_{self.session_id}.jsonl"
        self._handle = self.path.open("a", encoding="utf-8", buffering=1)
        self._file_bytes = self.path.stat().st_size if self.path.exists() else 0
        self._file_limit_reached = False

    def record(self, event_type: str, **data: Any) -> Dict[str, Any]:
        with self._lock:
            self._sequence += 1
            event = {
                "seq": self._sequence,
                "at": now_iso(),
                "monotonic": round(time.monotonic(), 3),
                "type": str(event_type)[:100],
                "data": sanitize_run_log_value(data),
            }
            self._events.append(event)
            encoded = (json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n").encode(
                "utf-8"
            )
            if (
                self._handle is not None
                and not self._file_limit_reached
                and self._file_bytes + len(encoded) <= self.MAX_FILE_BYTES
            ):
                self._handle.write(encoded.decode("utf-8"))
                self._file_bytes += len(encoded)
            elif not self._file_limit_reached:
                self._file_limit_reached = True
            return json.loads(json.dumps(event))

    def clear(self) -> Dict[str, Any]:
        with self._lock:
            if self._handle is not None:
                self._handle.close()
            self._events.clear()
            self._sequence = 0
            self._start_session()
        self.record("log_session_started", reason="user_clear", appVersion=APP_VERSION)
        return self.summary()

    def summary(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "sessionId": self.session_id,
                "eventCount": len(self._events),
                "fileBytes": self._file_bytes,
                "fileLimitBytes": self.MAX_FILE_BYTES,
                "fileLimitReached": self._file_limit_reached,
                "fileName": self.path.name,
            }

    def report(self, snapshot: Dict[str, Any]) -> str:
        with self._lock:
            events = list(self._events)
            summary = self.summary()
        setup = snapshot.get("setup", {}) or {}
        runtime = snapshot.get("runtime", {}) or {}
        network = public_network(setup.get("network", {}) or {})
        report_header = {
            "generatedAt": now_iso(),
            "dashboardVersion": APP_VERSION,
            "session": summary,
            "platform": platform.platform(),
            "network": network,
            "activeRobot": setup.get("activeRobot"),
            "topics": setup.get("topics", {}),
            "robot": setup.get("robot", {}),
            "accessory": setup.get("accessory", {}),
            "safety": setup.get("safety", {}),
            "fallbackNavigation": setup.get("fallbackNavigation", {}),
            "runtime": {
                "mode": runtime.get("mode"),
                "navStatus": runtime.get("navStatus"),
                "navMessage": runtime.get("navMessage"),
                "pose": runtime.get("pose"),
                "goal": runtime.get("goal"),
                "routeCheckpoint": runtime.get("routeCheckpoint"),
                "fallbackRecoveryPhase": runtime.get("fallbackRecoveryPhase"),
                "cameraEnabled": runtime.get("cameraEnabled"),
                "cameraStream": runtime.get("cameraStream"),
                "cameraFps": runtime.get("cameraFps"),
                "cameraBufferedFrames": runtime.get("cameraBufferedFrames"),
                "cameraFrameGapMs": runtime.get("cameraFrameGapMs"),
                "cameraSubscriptionTopic": runtime.get("cameraSubscriptionTopic"),
                "lastCameraAt": runtime.get("lastCameraAt"),
                "lastScanAt": runtime.get("lastScanAt"),
                "lastOdomAt": runtime.get("lastOdomAt"),
            },
        }
        lines = [
            "# TurtleBot Dashboard Run Log",
            "",
            "## Session",
            "```json",
            json.dumps(sanitize_run_log_value(report_header), ensure_ascii=False, indent=2),
            "```",
            "",
            "## Events (JSONL)",
            "```jsonl",
        ]
        lines.extend(json.dumps(event, ensure_ascii=False) for event in events)
        lines.extend(["```", ""])
        return "\n".join(lines)

    def close(self) -> None:
        with self._lock:
            if self._handle is not None:
                self._handle.close()
                self._handle = None


class DashboardState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = self._load()
        network = self._state.setdefault("setup", {}).setdefault("network", {})
        detected_server_ip = detect_server_ip(str(network.get("robotIp") or ""))
        if detected_server_ip:
            network["serverIp"] = detected_server_ip
        self.run_logs = RunLogStore()
        self.run_logs.record(
            "log_session_started",
            reason="server_start",
            appVersion=APP_VERSION,
            platform=platform.platform(),
            detectedServerIp=detected_server_ip or None,
        )
        self.camera_bytes: Optional[bytes] = None
        self.camera_content_type = "image/svg+xml"
        self._camera_frame_times: deque[float] = deque(maxlen=180)
        self._camera_frames: deque[Tuple[int, bytes, str]] = deque(
            maxlen=CAMERA_STREAM_BUFFER_FRAMES
        )
        self._camera_fps_estimate: Optional[float] = None
        self._camera_fps_updated_at = 0.0
        self._camera_metrics_logged_at = 0.0
        self._camera_condition = threading.Condition(self._lock)
        self._camera_sequence = 0
        self.save()

    def _load(self) -> Dict[str, Any]:
        if SETTINGS_PATH.exists():
            try:
                data = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
                state = normalize_robot_profiles_state(deep_merge(DEFAULT_STATE, data))
                legacy_setup = data.get("setup") if isinstance(data, dict) else {}
                legacy_topics = (
                    legacy_setup.get("topics") if isinstance(legacy_setup, dict) else None
                )
                if isinstance(legacy_topics, dict):
                    migration = {"topics": legacy_topics}
                    synchronize_active_robot_topics(migration, state["setup"])
                    state["setup"] = deep_merge(state["setup"], migration)
                    state = normalize_robot_profiles_state(state)
                state["runtime"] = fresh_runtime_state(
                    state["setup"], data.get("runtime") if isinstance(data, dict) else None
                )
                return state
            except Exception:
                state = normalize_robot_profiles_state(json.loads(json.dumps(DEFAULT_STATE)))
                state["runtime"] = fresh_runtime_state(state["setup"])
                return state
        state = normalize_robot_profiles_state(json.loads(json.dumps(DEFAULT_STATE)))
        state["runtime"] = fresh_runtime_state(state["setup"])
        return state

    def save(self) -> None:
        CONFIG_ROOT.mkdir(exist_ok=True)
        with self._lock:
            persisted = json.loads(json.dumps(self._state))
            persisted.setdefault("setup", {}).pop("topics", None)
            persisted_runtime = persisted.setdefault("runtime", {})
            persisted["runtime"] = {
                "cameraEnabled": bool(persisted_runtime.get("cameraEnabled", True)),
                "cameraStream": normalized_camera_stream_mode(
                    persisted_runtime.get("cameraStream", "compressed")
                ),
                "routeCheckpoint": normalized_route_checkpoint(
                    persisted_runtime.get("routeCheckpoint")
                ),
            }
            SETTINGS_PATH.write_text(
                json.dumps(persisted, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._state))

    def public_snapshot(self) -> Dict[str, Any]:
        snapshot = self.snapshot()
        setup = snapshot.setdefault("setup", {})
        setup["network"] = public_network(setup.get("network", {}))
        setup["robotProfiles"] = public_robot_profiles(setup.get("robotProfiles", {}))
        return snapshot

    def get_setup(self) -> Dict[str, Any]:
        with self._lock:
            return json.loads(json.dumps(self._state["setup"]))

    def refresh_server_ip(self) -> str:
        with self._lock:
            network = self._state.get("setup", {}).get("network", {}) or {}
            robot_ip = str(network.get("robotIp") or "")
            previous = str(network.get("serverIp") or "")
        detected = detect_server_ip(robot_ip)
        if not detected or detected == previous:
            return detected or previous
        with self._lock:
            self._state.setdefault("setup", {}).setdefault("network", {})["serverIp"] = detected
        self.save()
        if hasattr(self, "run_logs"):
            self.log_run_event(
                "server_ip_detected",
                previousServerIp=previous or None,
                serverIp=detected,
                robotIp=robot_ip or None,
            )
        return detected

    def update_setup(self, patch: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._state["setup"] = deep_merge(self._state["setup"], patch)
            self._state = normalize_robot_profiles_state(self._state)
            if "initialPose" in patch:
                pose = self._state["setup"]["initialPose"]
                self._state["runtime"]["pose"] = {
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "yaw": float(pose.get("yaw", 0.0)),
                    "source": "setup",
                    "stamp": now_iso(),
                }
            snapshot = json.loads(json.dumps(self._state))
        self.save()
        self.log_run_event(
            "setup_saved",
            robot=snapshot.get("setup", {}).get("robot"),
            planner=snapshot.get("setup", {}).get("planner"),
            fallbackNavigation=snapshot.get("setup", {}).get("fallbackNavigation"),
            obstacleCount=len(snapshot.get("setup", {}).get("obstacles", []) or []),
            activeRobot=snapshot.get("setup", {}).get("activeRobot"),
        )
        if "network" in patch:
            self.refresh_server_ip()
            return self.snapshot()
        return snapshot

    def replace_setup(self, setup: Dict[str, Any]) -> Dict[str, Any]:
        """Restore a setup snapshot after a failed live ROS reconfiguration."""
        with self._lock:
            self._state["setup"] = json.loads(json.dumps(setup))
            self._state = normalize_robot_profiles_state(self._state)
            snapshot = json.loads(json.dumps(self._state))
        self.save()
        self.log_run_event("setup_restored", activeRobot=snapshot["setup"].get("activeRobot"))
        return snapshot

    def update_runtime(self, patch: Dict[str, Any]) -> None:
        status_event: Optional[Dict[str, Any]] = None
        checkpoint_event: Optional[Dict[str, Any]] = None
        checkpoint_changed = False
        runtime_patch = dict(patch)
        raw_skipped_indices = runtime_patch.pop("_routeCheckpointSkippedIndices", [])
        checkpoint_skipped_indices: set[int] = set()
        if isinstance(raw_skipped_indices, list):
            for raw_index in raw_skipped_indices:
                try:
                    checkpoint_skipped_indices.add(int(raw_index))
                except (TypeError, ValueError):
                    continue
        with self._lock:
            previous_runtime = self._state.get("runtime", {})
            previous_status = previous_runtime.get("navStatus")
            previous_route = list(previous_runtime.get("route") or [])
            try:
                previous_route_index = int(previous_runtime.get("routeIndex") or 0)
            except (TypeError, ValueError):
                previous_route_index = 0
            self._state["runtime"] = deep_merge(self._state["runtime"], runtime_patch)
            runtime = self._state["runtime"]
            current_status = runtime.get("navStatus")
            checkpoint = normalized_route_checkpoint(runtime.get("routeCheckpoint"))
            if checkpoint["available"]:
                try:
                    current_route_index = int(runtime.get("routeIndex") or 0)
                except (TypeError, ValueError):
                    current_route_index = previous_route_index
                if previous_route and current_route_index > previous_route_index:
                    resolved_count = min(current_route_index, len(checkpoint["route"]))
                    completed = set(checkpoint["completedWaypointNumbers"])
                    for execution_index, pose in enumerate(
                        checkpoint["route"][:resolved_count]
                    ):
                        if execution_index not in checkpoint_skipped_indices:
                            completed.add(int(pose["sourceIndex"]) + 1)
                    checkpoint["completedWaypointNumbers"] = sorted(completed)
                    checkpoint["lastCompletedWaypointNumber"] = (
                        checkpoint["completedWaypointNumbers"][-1]
                        if checkpoint["completedWaypointNumbers"]
                        else None
                    )
                    checkpoint["nextIndex"] = max(checkpoint["nextIndex"], resolved_count)
                    checkpoint["nextWaypointNumber"] = (
                        int(checkpoint["route"][checkpoint["nextIndex"]]["sourceIndex"]) + 1
                        if checkpoint["nextIndex"] < len(checkpoint["route"])
                        else None
                    )
                    checkpoint["status"] = "active"
                    checkpoint["updatedAt"] = now_iso()
                    checkpoint_changed = True
                    checkpoint_event = {
                        "status": checkpoint["status"],
                        "nextIndex": checkpoint["nextIndex"],
                        "nextWaypointNumber": checkpoint["nextWaypointNumber"],
                        "completedWaypointNumbers": checkpoint["completedWaypointNumbers"],
                    }
                if current_status == "succeeded" and previous_route:
                    completed = set(checkpoint["completedWaypointNumbers"])
                    completed.update(
                        int(pose["sourceIndex"]) + 1 for pose in checkpoint["route"]
                    )
                    checkpoint.update(
                        {
                            "available": False,
                            "nextIndex": len(checkpoint["route"]),
                            "nextWaypointNumber": None,
                            "completedWaypointNumbers": sorted(completed),
                            "lastCompletedWaypointNumber": max(completed) if completed else None,
                            "status": "completed",
                            "interruptionReason": None,
                            "updatedAt": now_iso(),
                        }
                    )
                    checkpoint_changed = True
                    checkpoint_event = {
                        "status": "completed",
                        "nextIndex": checkpoint["nextIndex"],
                        "nextWaypointNumber": None,
                        "completedWaypointNumbers": checkpoint["completedWaypointNumbers"],
                    }
                elif "route" in runtime_patch and previous_route and not runtime.get("route"):
                    checkpoint.update(
                        {
                            "status": "interrupted",
                            "interruptionReason": str(current_status or "route_cleared"),
                            "updatedAt": now_iso(),
                        }
                    )
                    checkpoint_changed = True
                    checkpoint_event = {
                        "status": "interrupted",
                        "nextIndex": checkpoint["nextIndex"],
                        "nextWaypointNumber": checkpoint["nextWaypointNumber"],
                        "completedWaypointNumbers": checkpoint["completedWaypointNumbers"],
                        "reason": checkpoint["interruptionReason"],
                    }
            runtime["routeCheckpoint"] = checkpoint
            if "navStatus" in runtime_patch and current_status != previous_status:
                status_event = {
                    "status": current_status,
                    "previousStatus": previous_status,
                    "message": runtime.get("navMessage"),
                    "pose": runtime.get("pose"),
                    "goal": runtime.get("goal"),
                    "routeIndex": runtime.get("routeIndex"),
                    "fallbackActive": runtime.get("fallbackActive"),
                    "recoveryPhase": runtime.get("fallbackRecoveryPhase"),
                }
        if checkpoint_changed:
            self.save()
            if checkpoint_event:
                self.log_run_event("route_checkpoint_updated", **checkpoint_event)
        if status_event:
            self.log_run_event("nav_status", **status_event)

    def begin_route_checkpoint(
        self,
        poses: list[Dict[str, Any]],
        force_fallback: bool,
        resume: bool = False,
    ) -> Dict[str, Any]:
        with self._lock:
            previous = normalized_route_checkpoint(
                self._state.get("runtime", {}).get("routeCheckpoint")
            )
            seed = normalized_route_checkpoint(
                {
                    "available": True,
                    "route": poses,
                    "nextIndex": 0,
                    "completedWaypointNumbers": (
                        previous["completedWaypointNumbers"] if resume else []
                    ),
                    "robotId": self._state.get("setup", {}).get("activeRobot"),
                    "forceFallback": force_fallback,
                    "status": "active",
                    "resumedCount": previous["resumedCount"] + (1 if resume else 0),
                    "updatedAt": now_iso(),
                }
            )
            self._state.setdefault("runtime", {})["routeCheckpoint"] = seed
        self.save()
        self.log_run_event(
            "route_checkpoint_resumed" if resume else "route_checkpoint_started",
            routeLength=len(seed["route"]),
            nextWaypointNumber=seed["nextWaypointNumber"],
            completedWaypointNumbers=seed["completedWaypointNumbers"],
            forceFallback=seed["forceFallback"],
            robotId=seed["robotId"],
            resumedCount=seed["resumedCount"],
        )
        return seed

    def interrupt_route_checkpoint(self, reason: str) -> Dict[str, Any]:
        with self._lock:
            checkpoint = normalized_route_checkpoint(
                self._state.get("runtime", {}).get("routeCheckpoint")
            )
            if checkpoint["available"]:
                checkpoint.update(
                    {
                        "status": "interrupted",
                        "interruptionReason": str(reason or "interrupted"),
                        "updatedAt": now_iso(),
                    }
                )
                self._state.setdefault("runtime", {})["routeCheckpoint"] = checkpoint
        self.save()
        if checkpoint["available"]:
            self.log_run_event(
                "route_checkpoint_interrupted",
                reason=checkpoint["interruptionReason"],
                nextWaypointNumber=checkpoint["nextWaypointNumber"],
                completedWaypointNumbers=checkpoint["completedWaypointNumbers"],
            )
        return checkpoint

    def update_robot_pose(self, robot_id: str, pose: Dict[str, Any]) -> None:
        """Store a non-active robot pose without replacing the active robot pose."""
        with self._lock:
            runtime = self._state.setdefault("runtime", {})
            poses = runtime.setdefault("robotPoses", {})
            poses[robot_id] = pose

    def replace_robot_poses(self, poses: Dict[str, Any]) -> None:
        with self._lock:
            self._state.setdefault("runtime", {})["robotPoses"] = poses

    def log_run_event(self, event_type: str, **data: Any) -> None:
        self.run_logs.record(event_type, **data)

    def run_log_payload(self) -> Dict[str, Any]:
        snapshot = self.public_snapshot()
        summary = self.run_logs.summary()
        return {"ok": True, **summary, "report": self.run_logs.report(snapshot)}

    def clear_run_logs(self) -> Dict[str, Any]:
        return {"ok": True, **self.run_logs.clear()}

    def set_camera_enabled(self, enabled: bool) -> Dict[str, Any]:
        with self._lock:
            self._state["runtime"]["cameraEnabled"] = bool(enabled)
            if not enabled:
                self.camera_bytes = None
                self.camera_content_type = "image/svg+xml"
                self._camera_frame_times.clear()
                self._camera_frames.clear()
                self._camera_fps_estimate = None
                self._camera_fps_updated_at = 0.0
                self._state["runtime"]["cameraFps"] = None
                self._state["runtime"]["cameraBufferedFrames"] = 0
                self._state["runtime"]["cameraFrameGapMs"] = None
            self._camera_sequence += 1
            self._camera_condition.notify_all()
            return json.loads(json.dumps(self._state["runtime"]))

    def set_camera_stream_mode(self, mode: Any) -> Dict[str, Any]:
        stream_mode = normalized_camera_stream_mode(mode)
        with self._lock:
            runtime = self._state["runtime"]
            runtime["cameraStream"] = stream_mode
            self.camera_bytes = None
            self.camera_content_type = "image/svg+xml"
            runtime["lastCameraAt"] = None
            runtime["cameraFps"] = None
            runtime["cameraBufferedFrames"] = 0
            runtime["cameraFrameGapMs"] = None
            self._camera_frame_times.clear()
            self._camera_frames.clear()
            self._camera_fps_estimate = None
            self._camera_fps_updated_at = 0.0
            self._camera_sequence += 1
            self._camera_condition.notify_all()
            return json.loads(json.dumps(runtime))

    def camera_enabled(self) -> bool:
        with self._lock:
            return bool(self._state.get("runtime", {}).get("cameraEnabled", True))

    def camera_stream_mode(self) -> str:
        with self._lock:
            return normalized_camera_stream_mode(
                self._state.get("runtime", {}).get("cameraStream", "compressed")
            )

    def set_camera(self, content: bytes, content_type: str, stream_mode: str) -> None:
        metrics: Optional[Dict[str, Any]] = None
        with self._lock:
            runtime = self._state.get("runtime", {})
            if (
                not runtime.get("cameraEnabled", True)
                or normalized_camera_stream_mode(runtime.get("cameraStream"))
                != normalized_camera_stream_mode(stream_mode)
            ):
                return
            self.camera_bytes = content
            self.camera_content_type = content_type
            received = time.monotonic()
            previous_received = self._camera_frame_times[-1] if self._camera_frame_times else None
            self._camera_frame_times.append(received)
            while (
                self._camera_frame_times
                and received - self._camera_frame_times[0] > CAMERA_FPS_WINDOW_SECONDS
            ):
                self._camera_frame_times.popleft()
            runtime["lastCameraAt"] = now_iso()
            self._camera_sequence += 1
            self._camera_frames.append((self._camera_sequence, content, content_type))
            runtime["cameraBufferedFrames"] = len(self._camera_frames)
            if previous_received is not None:
                runtime["cameraFrameGapMs"] = round((received - previous_received) * 1000.0, 1)
            sample_window = (
                self._camera_frame_times[-1] - self._camera_frame_times[0]
                if len(self._camera_frame_times) >= 2
                else 0.0
            )
            if (
                sample_window >= CAMERA_FPS_MIN_WINDOW_SECONDS
                and received - self._camera_fps_updated_at >= CAMERA_FPS_UPDATE_SECONDS
            ):
                sample_rate = camera_frame_rate(list(self._camera_frame_times))
                if sample_rate is not None:
                    if self._camera_fps_estimate is None:
                        self._camera_fps_estimate = sample_rate
                    else:
                        self._camera_fps_estimate += CAMERA_FPS_SMOOTHING * (
                            sample_rate - self._camera_fps_estimate
                        )
                    runtime["cameraFps"] = round(self._camera_fps_estimate, 1)
                    self._camera_fps_updated_at = received
            if (
                runtime.get("cameraFps") is not None
                and received - self._camera_metrics_logged_at >= 10.0
            ):
                self._camera_metrics_logged_at = received
                metrics = {
                    "cameraFps": runtime.get("cameraFps"),
                    "cameraBufferedFrames": runtime.get("cameraBufferedFrames"),
                    "cameraFrameGapMs": runtime.get("cameraFrameGapMs"),
                    "cameraStream": runtime.get("cameraStream"),
                    "cameraSubscriptionTopic": runtime.get("cameraSubscriptionTopic"),
                }
            self._camera_condition.notify_all()
        if metrics is not None:
            self.log_run_event("camera_metrics", **metrics)

    def get_camera(self) -> Tuple[bytes, str]:
        with self._lock:
            if not self._state.get("runtime", {}).get("cameraEnabled", True):
                return camera_disabled_svg(), "image/svg+xml"
            if self.camera_bytes:
                return self.camera_bytes, self.camera_content_type
        return placeholder_svg(), "image/svg+xml"

    def wait_for_camera_frame(
        self, last_sequence: int, timeout: float = 1.0
    ) -> Tuple[bytes, str, int, bool, str]:
        with self._camera_condition:
            self._camera_condition.wait_for(
                lambda: self._camera_sequence != last_sequence,
                timeout=max(0.0, float(timeout)),
            )
            runtime = self._state.get("runtime", {})
            enabled = bool(runtime.get("cameraEnabled", True))
            stream_mode = normalized_camera_stream_mode(runtime.get("cameraStream"))
            if not enabled:
                content, content_type = camera_disabled_svg(), "image/svg+xml"
                sequence = self._camera_sequence
            elif self._camera_frames:
                # A camera preview prioritizes freshness over completeness.  If the
                # producer outruns the MJPEG client, sending queued older frames
                # would make the display increasingly stale; skip directly to the
                # newest frame instead.
                sequence, content, content_type = self._camera_frames[-1]
            elif self.camera_bytes:
                content, content_type = self.camera_bytes, self.camera_content_type
                sequence = self._camera_sequence
            else:
                content, content_type = placeholder_svg(), "image/svg+xml"
                sequence = self._camera_sequence
            return content, content_type, sequence, enabled, stream_mode


def placeholder_svg() -> bytes:
    text = """<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">
<rect width="960" height="540" fill="#172026"/>
<path d="M0 424 C160 382 312 470 486 414 C668 355 768 410 960 362 L960 540 L0 540 Z" fill="#25313a"/>
<circle cx="482" cy="252" r="66" fill="none" stroke="#87a1ad" stroke-width="10"/>
<rect x="348" y="198" width="268" height="168" rx="18" fill="none" stroke="#87a1ad" stroke-width="10"/>
<circle cx="404" cy="252" r="16" fill="#87a1ad"/>
<circle cx="560" cy="252" r="16" fill="#87a1ad"/>
<text x="480" y="432" fill="#c7d3d8" font-family="Arial, sans-serif" font-size="28" text-anchor="middle">No camera frame</text>
</svg>"""
    return text.encode("utf-8")


def camera_disabled_svg() -> bytes:
    text = """<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">
<rect width="960" height="540" fill="#10171b"/>
<rect x="300" y="176" width="360" height="208" rx="20" fill="none" stroke="#6f7f86" stroke-width="10"/>
<path d="M340 414 L620 126" stroke="#b66b6b" stroke-width="14" stroke-linecap="round"/>
<text x="480" y="448" fill="#c7d3d8" font-family="Arial, sans-serif" font-size="28" text-anchor="middle">Camera off</text>
</svg>"""
    return text.encode("utf-8")


def save_data_url(
    data_url: str,
    directory: Path = DATA_ROOT,
    url_prefix: str = "/data/",
) -> str:
    if "," not in data_url:
        raise ValueError("invalid data url")
    meta, payload = data_url.split(",", 1)
    if ";base64" not in meta.lower():
        raise ValueError("map upload must use base64 encoding")
    mime = "application/octet-stream"
    if meta.startswith("data:") and ";" in meta:
        mime = meta[5:].split(";", 1)[0]
    extension = {
        "image/png": ".png",
        "image/jpeg": ".jpg",
        "image/jpg": ".jpg",
        "image/gif": ".gif",
        "image/bmp": ".bmp",
        "image/x-portable-graymap": ".pgm",
    }.get(mime, ".bin")
    directory.mkdir(parents=True, exist_ok=True)
    filename = f"map-{uuid.uuid4().hex}{extension}"
    path = directory / filename
    try:
        content = base64.b64decode(payload, validate=True)
    except (ValueError, binascii.Error) as exc:
        raise ValueError("invalid base64 map data") from exc
    if len(content) > MAX_MAP_UPLOAD_BYTES:
        raise ValueError(f"map upload exceeds {MAX_MAP_UPLOAD_BYTES // (1024 * 1024)} MiB")
    path.write_bytes(content)
    return f"{url_prefix.rstrip('/')}/{filename}"


def save_editor_map_data_url(data_url: str) -> str:
    return save_data_url(data_url, directory=MAP_DATA_ROOT, url_prefix="/data/maps")


def delete_editor_map_file(image_url: str) -> None:
    prefix = "/data/maps/"
    if not str(image_url).startswith(prefix):
        return
    path = safe_child_path(MAP_DATA_ROOT, str(image_url)[len(prefix) :])
    if path.is_file():
        path.unlink()


def parse_map_editor_dimensions(body: Dict[str, Any]) -> Tuple[int, int]:
    try:
        width_cm = finite_float(body.get("widthCm"), "widthCm")
        height_cm = finite_float(body.get("heightCm"), "heightCm")
    except ValueError as exc:
        raise ValueError("map widthCm and heightCm must be numbers") from exc
    width = int(round(width_cm))
    height = int(round(height_cm))
    if abs(width_cm - width) > 1e-6 or abs(height_cm - height) > 1e-6:
        raise ValueError("map dimensions must use whole centimeters")
    if not 10 <= width <= 2000 or not 10 <= height <= 2000:
        raise ValueError("map dimensions must be between 10 and 2000 cm")
    return width, height


def parse_map_editor_cm_per_pixel(body: Dict[str, Any]) -> float:
    try:
        value = finite_float(body.get("cmPerPixel", 1), "cmPerPixel")
    except ValueError as exc:
        raise ValueError("cmPerPixel must be a number") from exc
    if not 0.1 <= value <= 100.0:
        raise ValueError("cmPerPixel must be between 0.1 and 100")
    return value


def map_library_entry(
    map_id: str,
    name: str,
    image_url: str,
    width_pixels: int,
    height_pixels: int,
    resolution: float = 0.01,
) -> Dict[str, Any]:
    return {
        "id": str(map_id),
        "name": str(name).strip()[:80] or "새 맵",
        "imageUrl": str(image_url),
        "resolution": float(resolution),
        "originX": 0.0,
        "originY": 0.0,
        "originYaw": 0.0,
        "widthPixels": int(width_pixels),
        "heightPixels": int(height_pixels),
    }


def image_msg_to_bmp(msg: Any) -> Optional[bytes]:
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()
    data = bytes(msg.data)

    if width <= 0 or height <= 0:
        return None

    bytes_per_pixel = {
        "rgb8": 3,
        "bgr8": 3,
        "8uc3": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8uc1": 1,
    }.get(encoding)
    if bytes_per_pixel is None or step < width * bytes_per_pixel or len(data) < step * height:
        return None

    def pixel_to_bgr(row: bytes, pixel_index: int) -> bytes:
        if encoding in ("rgb8", "8uc3"):
            i = pixel_index * 3
            return bytes((row[i + 2], row[i + 1], row[i]))
        if encoding == "bgr8":
            i = pixel_index * 3
            return row[i : i + 3]
        if encoding == "rgba8":
            i = pixel_index * 4
            return bytes((row[i + 2], row[i + 1], row[i]))
        if encoding == "bgra8":
            i = pixel_index * 4
            return row[i : i + 3]
        if encoding in ("mono8", "8uc1"):
            value = row[pixel_index]
            return bytes((value, value, value))
        return b""

    row_stride = ((width * 3 + 3) // 4) * 4
    pixel_bytes = bytearray(row_stride * height)
    padding = row_stride - width * 3

    out_index = 0
    for row_number in range(height - 1, -1, -1):
        row_start = row_number * step
        row = data[row_start : row_start + step]
        for x in range(width):
            bgr = pixel_to_bgr(row, x)
            if not bgr:
                return None
            pixel_bytes[out_index : out_index + 3] = bgr
            out_index += 3
        if padding:
            out_index += padding

    file_size = 14 + 40 + len(pixel_bytes)
    header = struct.pack("<2sIHHI", b"BM", file_size, 0, 0, 54)
    dib = struct.pack(
        "<IIIHHIIIIII",
        40,
        width,
        height,
        1,
        24,
        0,
        len(pixel_bytes),
        2835,
        2835,
        0,
        0,
    )
    return header + dib + bytes(pixel_bytes)


class NullRosBridge:
    def __init__(self, state: DashboardState, bridge_error: str = "") -> None:
        self.state = state
        self.bridge_error = str(bridge_error or "ROS2 bridge initialization failed").strip()
        self._last_logged_manual_command: Optional[Tuple[float, float]] = None
        self._route_repeat_lock = threading.RLock()
        self._route_repeat_generation = 0
        self._route_repeat: Dict[str, Any] = {"enabled": False, "pauseSeconds": 0.0}
        self._route_repeat_resume_at = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.state.update_runtime(
            {
                "rosConnected": False,
                "mode": "offline-preview",
                "rosBridgeError": self.bridge_error,
                "navMessage": f"ROS2 bridge unavailable: {self.bridge_error}",
                "connection": base_connection_info(False, network=state.get_setup().get("network", {})),
            }
        )

    def set_bridge_error(self, bridge_error: str) -> None:
        self.bridge_error = str(bridge_error or "ROS2 bridge initialization failed").strip()
        self.state.update_runtime(
            {
                "rosConnected": False,
                "mode": "offline-preview",
                "rosBridgeError": self.bridge_error,
                "navMessage": f"ROS2 bridge unavailable: {self.bridge_error}",
            }
        )

    def set_camera_enabled(self, enabled: bool) -> Dict[str, Any]:
        return self.state.set_camera_enabled(enabled)

    def set_camera_stream_mode(self, mode: Any) -> Dict[str, Any]:
        return self.state.set_camera_stream_mode(mode)

    def set_initial_pose(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        self.state.log_run_event("initial_pose_requested", pose={"x": x, "y": y, "yaw": yaw})
        self.state.update_runtime(
            {
                "pose": {"x": x, "y": y, "yaw": yaw, "source": "preview", "stamp": now_iso()},
                "navStatus": "initial_pose_set",
                "navMessage": "Initial pose stored locally.",
            }
        )
        return {"ok": True, "mode": "offline-preview"}

    def send_goal(
        self,
        x: float,
        y: float,
        yaw: float,
        clear_route: bool = True,
        path: Optional[list[Dict[str, Any]]] = None,
        force_fallback: bool = False,
    ) -> Dict[str, Any]:
        if clear_route:
            self._cancel_route_repeat()
        self.state.log_run_event(
            "goal_requested",
            goal={"x": x, "y": y, "yaw": yaw},
            pathPointCount=len(path or []),
            clearRoute=clear_route,
            forceFallback=force_fallback,
            mode="offline-preview",
        )
        patch = {
            "goal": {"x": x, "y": y, "yaw": yaw, "stamp": now_iso()},
            "navStatus": "moving",
            "navMessage": "Preview goal accepted.",
            "lastCommandAt": now_iso(),
            "fallbackPathLength": len(path or []),
            "fallbackPathIndex": 0,
        }
        if clear_route:
            patch.update({"route": [], "routeIndex": 0})
        self.state.update_runtime(patch)
        return {
            "ok": True,
            "mode": "offline-preview",
            "transport": "lidar_fallback_preview" if force_fallback else "preview",
        }

    def send_route(
        self,
        poses: list[Dict[str, float]],
        path: Optional[list[Dict[str, Any]]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        force_fallback: bool = False,
        repeat: Optional[Dict[str, Any]] = None,
        resume: bool = False,
    ) -> Dict[str, Any]:
        if not poses:
            return {"ok": False, "error": "Route is empty."}
        repeat_config = normalized_route_repeat(repeat, len(poses))
        with self._route_repeat_lock:
            self._route_repeat_generation += 1
            self._route_repeat = repeat_config
            self._route_repeat_resume_at = 0.0
        self.state.log_run_event(
            "route_requested",
            poses=poses,
            pathPointCount=len(path or []),
            repeatPathPointCount=len(repeat_path or []),
            pathRouteIndices=sorted(
                {
                    int(point["routeIndex"])
                    for point in (path or [])
                    if isinstance(point.get("routeIndex"), int)
                }
            ),
            forceFallback=force_fallback,
            repeat=repeat_config,
            mode="offline-preview",
        )
        route = [dict(pose, stamp=now_iso()) for pose in poses]
        self.state.begin_route_checkpoint(route, force_fallback, resume=resume)
        self.state.update_runtime(
            {
                "route": route,
                "routeIndex": 0,
                "routeRepeatEnabled": repeat_config["enabled"],
                "routeRepeatCycle": 1,
                "routeRepeatPauseSeconds": repeat_config["pauseSeconds"],
                "routeRepeatStart": repeat_config["sourceStart"],
                "routeRepeatEnd": repeat_config["sourceEnd"],
                "routeRepeatResumeAt": None,
            }
        )
        first = route[0]
        self.send_goal(
            float(first["x"]),
            float(first["y"]),
            float(first.get("yaw", 0.0)),
            clear_route=False,
            path=path,
            force_fallback=force_fallback,
        )
        self.state.update_runtime({"navMessage": f"Preview route accepted: 1/{len(route)}."})
        return {
            "ok": True,
            "mode": "offline-preview",
            "routeLength": len(route),
            "repeat": repeat_config,
            "transport": "lidar_fallback_preview" if force_fallback else "preview",
        }

    def cancel_goal(self) -> Dict[str, Any]:
        self._cancel_route_repeat()
        self.state.log_run_event("cancel_requested", mode="offline-preview")
        self.state.update_runtime(
            {"goal": None, "route": [], "routeIndex": 0, "navStatus": "canceled", "navMessage": "Goal canceled."}
        )
        return {"ok": True}

    def stop_robot(self) -> Dict[str, Any]:
        self._cancel_route_repeat()
        topic = self.state.get_setup().get("topics", {}).get("cmdVel", "/cmd_vel")
        self.state.log_run_event("stop_requested", topic=topic, mode="offline-preview")
        self.state.update_runtime(
            {
                "goal": None,
                "route": [],
                "routeIndex": 0,
                "navStatus": "stopped",
                "navMessage": f"Stop requested. cmd_vel topic: {topic}",
            }
        )
        return {"ok": True, "topic": topic}

    def manual_drive(self, linear: float, angular: float) -> Dict[str, Any]:
        self._cancel_route_repeat()
        linear = max(-0.22, min(0.22, finite_float(linear, "linear")))
        angular = max(-2.84, min(2.84, finite_float(angular, "angular")))
        active_command = abs(linear) > 0 or abs(angular) > 0
        topic = self.state.get_setup().get("topics", {}).get("cmdVel", "/cmd_vel")
        command = (round(linear, 3), round(angular, 3))
        if command != self._last_logged_manual_command:
            self._last_logged_manual_command = command
            self.state.log_run_event(
                "manual_command",
                linear=linear,
                angular=angular,
                topic=topic,
                mode="offline-preview",
            )
        self.state.update_runtime(
            {
                "goal": None,
                "route": [],
                "routeIndex": 0,
                "navStatus": "manual" if active_command else "manual_stop",
                "navMessage": f"Preview manual {'drive' if active_command else 'stop'} on {topic}: v={linear:.2f}, w={angular:.2f}",
                "lastManualCommandAt": now_iso(),
                "lastCommandAt": now_iso(),
            }
        )
        return {"ok": True, "mode": "offline-preview", "topic": topic}

    def connection_status(self) -> Dict[str, Any]:
        snapshot = self.state.snapshot()
        return {
            "ok": True,
            "mode": "offline-preview",
            "runtime": snapshot["runtime"],
            "connection": base_connection_info(False, network=snapshot["setup"].get("network", {})),
            "topics": snapshot["setup"]["topics"],
            "network": public_network(snapshot["setup"].get("network", {})),
        }

    def robot_check(self) -> Dict[str, Any]:
        detected_server_ip = self.state.refresh_server_ip()
        snapshot = self.state.snapshot()
        network = snapshot["setup"].get("network", {})
        bridge_error = str(
            snapshot.get("runtime", {}).get("rosBridgeError") or self.bridge_error
        )
        checks = [
            {
                "id": "ros_bridge",
                "label": "ROS2 브릿지",
                "ok": False,
                "level": "fail",
                "detail": (
                    "현재 서버가 offline-preview 모드입니다. "
                    f"ROS2 브릿지 초기화 오류: {bridge_error}"
                ),
            },
            {
                "id": "domain_config",
                "label": "설정된 ROS_DOMAIN_ID",
                "ok": bool(network.get("rosDomainId")),
                "level": "warn" if network.get("rosDomainId") else "fail",
                "detail": f"설정값: {network.get('rosDomainId') or '-'}",
            },
            {
                "id": "ssh_config",
                "label": "Robot SSH",
                "ok": bool(network.get("robotSshHost") and network.get("robotSshUser")),
                "level": "warn",
                "detail": f"{network.get('robotSshUser') or '-'}@{network.get('robotSshHost') or '-'}",
            },
            {
                "id": "camera_bringup",
                "label": "Camera bringup",
                "ok": False,
                "level": "fail",
                "detail": "offline-preview mode; ROS2 camera publishers cannot be inspected here.",
            },
        ]
        return {
            "ok": False,
            "mode": "offline-preview",
            "summary": "ROS2 브릿지가 연결되지 않았습니다.",
            "checks": checks,
            "advice": [
                f"서버 PC에서 source /opt/ros/jazzy/setup.bash 후 ROS_DOMAIN_ID={network.get('rosDomainId') or '1'}로 실행하세요.",
                f"ROS2 브릿지 예외 원문: {bridge_error}",
                *dashboard_access_advice(detected_server_ip or network.get("serverIp", "")),
            ],
            "runtime": snapshot["runtime"],
            "connection": base_connection_info(False, network=network),
            "network": public_network(network),
        }

    def diagnostics_report(self) -> Dict[str, Any]:
        snapshot = self.state.snapshot()
        check = self.robot_check()
        setup = snapshot.get("setup", {})
        command_outputs = collect_diagnostic_commands(
            setup.get("topics", {}),
            setup.get("robotProfiles", {}),
        )
        robot_ssh_output = self.robot_ssh_check(detailed=True)
        return {
            "ok": True,
            "generatedAt": now_iso(),
            "report": format_diagnostics_report(
                snapshot,
                check,
                command_outputs=command_outputs,
                robot_ssh_output=robot_ssh_output,
            ),
            "commands": command_outputs,
            "robotSsh": robot_ssh_output,
        }

    def robot_bringup(self) -> Dict[str, Any]:
        result = run_robot_bringup_from_state(self.state)
        result["check"] = self.robot_check()
        return result

    def robot_ssh_check(self, detailed: bool = False) -> Dict[str, Any]:
        return run_robot_ssh_check_from_state(self.state, detailed=detailed)

    def manual_drive_check(self) -> Dict[str, Any]:
        return {
            "ok": False,
            "mode": "offline-preview",
            "error": "ROS2 bridge is not active, so the dashboard cannot publish a cmd_vel test message.",
        }

    def discover_robots(self) -> Dict[str, Any]:
        snapshot = self.state.snapshot()
        network = snapshot["setup"].get("network", {})
        return {
            "ok": True,
            "mode": "offline-preview",
            "connection": base_connection_info(False, network=network),
            "topics": [],
            "nodes": [],
            "candidates": [],
            "message": "ROS2 rclpy not available; discovery needs a ROS2-sourced shell.",
            "network": public_network(network),
            "networkDiscovery": discover_same_subnet_ssh_hosts(network),
        }

    def detect_active_robot_topics(self, active_robot: str = "") -> Dict[str, Any]:
        _ = active_robot
        return {
            "ok": False,
            "mode": "offline-preview",
            "topics": {},
            "message": "ROS2 bridge is not active, so the dashboard cannot search the robot topic graph.",
        }

    def shutdown(self) -> None:
        self._cancel_route_repeat()
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            snapshot = self.state.snapshot()
            runtime = snapshot["runtime"]
            if runtime.get("navStatus") == "route_pause":
                with self._route_repeat_lock:
                    repeat_enabled = bool(self._route_repeat.get("enabled"))
                    resume_at = self._route_repeat_resume_at
                route = runtime.get("route") or []
                if repeat_enabled and route and resume_at and time.monotonic() >= resume_at:
                    next_cycle = int(runtime.get("routeRepeatCycle") or 1) + 1
                    first = route[0]
                    with self._route_repeat_lock:
                        self._route_repeat_resume_at = 0.0
                    self.state.log_run_event(
                        "route_repeat_resumed",
                        cycle=next_cycle,
                        mode="offline-preview",
                    )
                    self.state.update_runtime(
                        {
                            "goal": dict(first, stamp=now_iso()),
                            "routeIndex": 0,
                            "routeRepeatCycle": next_cycle,
                            "routeRepeatResumeAt": None,
                            "navStatus": "moving",
                            "navMessage": f"Preview repeat cycle {next_cycle}: 1/{len(route)}.",
                        }
                    )
                    time.sleep(0.2)
                    continue
            goal = snapshot["runtime"].get("goal")
            pose = snapshot["runtime"].get("pose", {})
            if goal and snapshot["runtime"].get("navStatus") == "moving":
                dx = float(goal["x"]) - float(pose.get("x", 0.0))
                dy = float(goal["y"]) - float(pose.get("y", 0.0))
                distance = math.hypot(dx, dy)
                if distance < 0.03:
                    route = snapshot["runtime"].get("route") or []
                    route_index = int(snapshot["runtime"].get("routeIndex") or 0)
                    if route and route_index < len(route) - 1:
                        next_index = route_index + 1
                        next_goal = route[next_index]
                        self.state.update_runtime(
                            {
                                "pose": {
                                    "x": float(goal["x"]),
                                    "y": float(goal["y"]),
                                    "yaw": float(goal["yaw"]),
                                    "source": "preview",
                                    "stamp": now_iso(),
                                },
                                "goal": {
                                    "x": float(next_goal["x"]),
                                    "y": float(next_goal["y"]),
                                    "yaw": float(next_goal.get("yaw", 0.0)),
                                    "stamp": now_iso(),
                                },
                                "routeIndex": next_index,
                                "navStatus": "moving",
                                "navMessage": f"Preview waypoint {next_index}/{len(route)} reached.",
                            }
                        )
                        time.sleep(0.2)
                        continue
                    final_pose = {
                        "x": float(goal["x"]),
                        "y": float(goal["y"]),
                        "yaw": float(goal["yaw"]),
                        "source": "preview",
                        "stamp": now_iso(),
                    }
                    with self._route_repeat_lock:
                        repeat_config = dict(self._route_repeat)
                    if route and repeat_config.get("enabled"):
                        pause_seconds = float(repeat_config["pauseSeconds"])
                        with self._route_repeat_lock:
                            self._route_repeat_resume_at = time.monotonic() + pause_seconds
                        cycle = int(snapshot["runtime"].get("routeRepeatCycle") or 1)
                        self.state.log_run_event(
                            "route_repeat_paused",
                            cycle=cycle,
                            pauseSeconds=pause_seconds,
                            mode="offline-preview",
                        )
                        self.state.update_runtime(
                            {
                                "pose": final_pose,
                                "goal": None,
                                "routeIndex": len(route) - 1,
                                "routeRepeatResumeAt": time.strftime(
                                    "%Y-%m-%dT%H:%M:%S",
                                    time.localtime(time.time() + pause_seconds),
                                ),
                                "navStatus": "route_pause",
                                "navMessage": (
                                    f"Preview cycle {cycle} complete. "
                                    f"Resuming in {pause_seconds:g} seconds."
                                ),
                            }
                        )
                    else:
                        self.state.update_runtime(
                            {
                                "pose": final_pose,
                                "goal": None,
                                "route": [],
                                "routeIndex": 0,
                                "routeRepeatEnabled": False,
                                "routeRepeatResumeAt": None,
                                "navStatus": "succeeded",
                                "navMessage": "Preview route complete." if route else "Preview goal reached.",
                            }
                        )
                else:
                    step = min(0.08, distance)
                    yaw = math.atan2(dy, dx)
                    self.state.update_runtime(
                        {
                            "pose": {
                                "x": float(pose.get("x", 0.0)) + math.cos(yaw) * step,
                                "y": float(pose.get("y", 0.0)) + math.sin(yaw) * step,
                                "yaw": yaw,
                                "source": "preview",
                                "stamp": now_iso(),
                            }
                        }
                    )
            time.sleep(0.25)

    def _cancel_route_repeat(self) -> None:
        with self._route_repeat_lock:
            self._route_repeat_generation += 1
            self._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
            self._route_repeat_resume_at = 0.0
        self.state.update_runtime(
            {
                "routeRepeatEnabled": False,
                "routeRepeatCycle": 0,
                "routeRepeatPauseSeconds": 0.0,
                "routeRepeatResumeAt": None,
            }
        )


def shutdown_ros_executor(executor: Any, node: Any, timeout_sec: float = 1.0) -> None:
    if executor is None:
        return
    try:
        executor.shutdown(timeout_sec=timeout_sec)
    except Exception:
        pass
    if node is not None:
        try:
            executor.remove_node(node)
        except Exception:
            pass


class RosBridge:
    def __init__(self, state: DashboardState) -> None:
        self.state = state
        self.setup = state.get_setup()
        self.topics = self.setup["topics"]
        self.node = None
        self.ros_context = None
        self.ros_executor = None
        self.goal_handle = None
        self.route_goal_handle = None
        self._last_odom_pose: Optional[Tuple[float, float, float]] = None
        self._last_odom_monotonic = 0.0
        self._last_pose_monotonic = 0.0
        self._odom_anchor: Optional[Dict[str, Tuple[float, float, float]]] = None
        self._pending_odom_anchor: Optional[Tuple[float, float, float]] = None
        self._last_raw_camera = 0.0
        self._camera_subscription_lock = threading.RLock()
        self._camera_subscription = None
        self._camera_subscription_mode: Optional[str] = None
        self._camera_callback_group = None
        self._last_amcl = 0.0
        self._last_lidar_display = 0.0
        self._last_sensor_run_log = 0.0
        self._dynamic_lidar_obstacle_tracks: Dict[str, Dict[str, Any]] = {}
        self._static_wall_map_cache: Dict[str, Any] = {}
        self._last_logged_manual_command: Optional[Tuple[float, float]] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._publisher_lock = threading.RLock()
        self._manual_lock = threading.Lock()
        self._manual_active_until = 0.0
        self._manual_target_linear = 0.0
        self._manual_target_angular = 0.0
        self._manual_watchdog_thread: Optional[threading.Thread] = None
        self._scan_lock = threading.Lock()
        self._latest_scan: Dict[str, Any] = {"points": [], "received": 0.0, "frame": ""}
        self._fallback_nav_lock = threading.RLock()
        self._fallback_nav_generation = 0
        self._fallback_nav_active = False
        self._fallback_nav_thread: Optional[threading.Thread] = None
        self._route_sequence_lock = threading.RLock()
        self._route_sequence_generation = 0
        self._route_repeat: Dict[str, Any] = {"enabled": False, "pauseSeconds": 0.0}
        self._route_transport = ""
        self._route_pause_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()
        self._robot_clock_lock = threading.Lock()
        self._robot_clock_anchor_ns: Optional[int] = None
        self._robot_clock_received_monotonic_ns: Optional[int] = None
        self._robot_clock_skew_ms: Optional[float] = None
        self._last_clock_runtime_update = 0.0
        try:
            self._init_ros()
        except Exception as exc:
            self._activate_fallback(exc)

    def _init_ros(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
            from nav_msgs.msg import Odometry
            from rclpy.action import ActionClient
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            from rclpy.context import Context
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy,
                HistoryPolicy,
                QoSProfile,
                ReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from sensor_msgs.msg import CompressedImage, Image, LaserScan

            try:
                from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
            except Exception:
                NavigateToPose = None
                NavigateThroughPoses = None
        except Exception as exc:
            self._activate_fallback(exc)
            return

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.Twist = Twist
        self.TwistStamped = TwistStamped
        self.Image = Image
        self.CompressedImage = CompressedImage
        self.NavigateToPose = NavigateToPose
        self.NavigateThroughPoses = NavigateThroughPoses
        self.sensor_qos = qos_profile_sensor_data
        self.camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        _robot_id, profile = active_robot_profile(self.setup)
        connection = profile.get("connection") if isinstance(profile.get("connection"), dict) else {}
        domain_id = int(connection.get("rosDomainId") or os.environ.get("ROS_DOMAIN_ID") or 1)
        self.ros_context = Context()
        self.ros_context.init(args=None, initialize_logging=False, domain_id=domain_id)

        class DashboardNode(Node):
            pass

        self.node = DashboardNode("turtlebot_web_dashboard", context=self.ros_context)
        self.ros_executor = MultiThreadedExecutor(num_threads=2, context=self.ros_context)
        self.ros_executor.add_node(self.node)
        self._camera_callback_group = MutuallyExclusiveCallbackGroup()
        qos = 10
        self.initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, self.topics["initialPose"], qos
        )
        self.goal_pub = self.node.create_publisher(PoseStamped, self.topics["goalTopic"], qos)
        self.cmd_vel_msg_type = self._detect_cmd_vel_message_type(self.topics["cmdVel"])
        cmd_vel_cls = TwistStamped if self.cmd_vel_msg_type == TWIST_STAMPED_TYPE else Twist
        self.cmd_vel_pub = self.node.create_publisher(cmd_vel_cls, self.topics["cmdVel"], qos)

        self.node.create_subscription(
            PoseWithCovarianceStamped, self.topics["pose"], self._on_pose, qos
        )
        self.node.create_subscription(Odometry, self.topics["odom"], self._on_odom, qos)
        self.node.create_subscription(
            LaserScan, self.topics.get("scan", "/scan"), self._on_scan, self.sensor_qos
        )
        self._configure_camera_subscription(self.state.camera_stream_mode())
        self.action_client = (
            ActionClient(self.node, NavigateToPose, self.topics["goalAction"])
            if NavigateToPose is not None
            else None
        )
        self.route_action_client = (
            ActionClient(self.node, NavigateThroughPoses, self.topics["routeAction"])
            if NavigateThroughPoses is not None
            else None
        )

        self.state.update_runtime(
            {
                "rosConnected": True,
                "mode": "ros2",
                "navMessage": f"ROS2 bridge started. cmd_vel: {self.topics['cmdVel']} ({self._cmd_vel_type_label()}).",
                "cmdVelMessageType": self.cmd_vel_msg_type,
                "connection": base_connection_info(
                    self._action_available(0.01),
                    self._route_action_available(0.01),
                    active_robot_network(self.setup),
                ),
                **self._cmd_vel_clock_status(),
            }
        )
        self._spin_thread = threading.Thread(
            target=self._spin_executor, args=(self.ros_executor,), daemon=True
        )
        self._spin_thread.start()
        self._start_manual_watchdog()

    def _spin_executor(self, executor: Any) -> None:
        try:
            executor.spin()
        except Exception as exc:
            if not self._shutdown_event.is_set():
                self.state.update_runtime(
                    {"rosConnected": False, "navStatus": "ros_error", "navMessage": f"ROS spin stopped: {exc}"}
                )

    def _activate_fallback(self, exc: Exception) -> None:
        self._fallback_reason = ros_bridge_error_text(exc)
        node = getattr(self, "node", None)
        executor = getattr(self, "ros_executor", None)
        shutdown_ros_executor(executor, node)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        context = getattr(self, "ros_context", None)
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                pass
        self.ros_context = None
        self.ros_executor = None
        self.node = None
        if not hasattr(self, "_fallback"):
            self._fallback = NullRosBridge(self.state, self._fallback_reason)
        else:
            self._fallback.set_bridge_error(self._fallback_reason)

    def _is_fallback(self) -> bool:
        return hasattr(self, "_fallback")

    def _configure_camera_subscription(self, mode: Any) -> None:
        stream_mode = normalized_camera_stream_mode(mode)
        active_topic: Optional[str] = None
        with self._camera_subscription_lock:
            if self.node is None:
                return
            enabled = self.state.camera_enabled()
            if (
                enabled
                and self._camera_subscription is not None
                and self._camera_subscription_mode == stream_mode
            ):
                return

            previous = self._camera_subscription
            self._camera_subscription = None
            if previous is not None:
                self.node.destroy_subscription(previous)

            self._camera_subscription_mode = stream_mode
            if enabled:
                if stream_mode == "raw":
                    message_type = self.Image
                    topic = self.topics["camera"]
                    callback = self._on_image
                else:
                    message_type = self.CompressedImage
                    topic = self.topics["compressedCamera"]
                    callback = self._on_compressed_image
                active_topic = topic
                self._camera_subscription = self.node.create_subscription(
                    message_type,
                    topic,
                    callback,
                    self.camera_qos,
                    callback_group=self._camera_callback_group,
                )
            if self.ros_executor is not None:
                self.ros_executor.wake()
        self.state.update_runtime(
            {
                "cameraSubscriptionTopic": active_topic,
                "cameraSubscriptionQueueDepth": 1 if enabled else None,
                "cameraExecutorThreads": 2,
            }
        )

    def set_camera_enabled(self, enabled: bool) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.set_camera_enabled(enabled)
        runtime = self.state.set_camera_enabled(enabled)
        self._configure_camera_subscription(runtime.get("cameraStream"))
        return runtime

    def set_camera_stream_mode(self, mode: Any) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.set_camera_stream_mode(mode)
        runtime = self.state.set_camera_stream_mode(mode)
        self._configure_camera_subscription(runtime.get("cameraStream"))
        return runtime

    def reload_setup(self) -> Dict[str, Any]:
        next_setup = self.state.get_setup()
        self._clear_manual_watchdog()
        self._stop_lidar_fallback(publish_stop=False)
        try:
            self._publish_cmd_vel(0.0, 0.0, repeats=6, interval=0.04)
        except Exception:
            pass
        old_node = self.node
        old_executor = self.ros_executor
        self.goal_handle = None
        self.route_goal_handle = None
        fallback = getattr(self, "_fallback", None)
        if fallback is not None:
            fallback.shutdown()
            del self._fallback
        shutdown_ros_executor(old_executor, old_node)
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        with self._publisher_lock:
            try:
                if old_node is not None:
                    old_node.destroy_node()
            except Exception:
                pass

        old_context = self.ros_context
        if old_context is not None:
            try:
                old_context.try_shutdown()
            except Exception:
                pass
        self.ros_context = None
        self.ros_executor = None
        self._spin_thread = None

        self.setup = next_setup
        self.topics = self.setup["topics"]
        self.node = None
        self.action_client = None
        self.route_action_client = None
        self._camera_subscription = None
        self._camera_subscription_mode = None
        self._camera_callback_group = None
        self._last_raw_camera = 0.0
        self._last_amcl = 0.0
        self._last_lidar_display = 0.0
        self._last_sensor_run_log = 0.0
        self._last_odom_pose = None
        self._last_odom_monotonic = 0.0
        self._last_pose_monotonic = 0.0
        self._odom_anchor = None
        self._pending_odom_anchor = None
        with self._scan_lock:
            self._latest_scan = {"points": [], "received": 0.0, "frame": ""}
        self._dynamic_lidar_obstacle_tracks.clear()
        self.state.update_runtime(
            {"dynamicLidarObstacles": [], "dynamicLidarObstacleCount": 0}
        )
        self._reset_robot_clock()
        try:
            self._init_ros()
        except Exception as exc:
            self._activate_fallback(exc)
        return {
            "ok": not self._is_fallback(),
            "mode": "offline-preview" if self._is_fallback() else "ros2",
            "message": (
                "ROS bridge reloaded with updated topics."
                if not self._is_fallback()
                else f"ROS bridge reload failed: {getattr(self, '_fallback_reason', 'unknown error')}"
            ),
        }

    def _on_pose(self, msg: Any) -> None:
        self._last_amcl = time.monotonic()
        self._last_pose_monotonic = self._last_amcl
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.state.update_runtime(
            {
                "pose": {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "yaw": yaw,
                    "source": self.topics["pose"],
                    "stamp": now_iso(),
                },
                "lastPoseAt": now_iso(),
            }
        )

    def _on_odom(self, msg: Any) -> None:
        self._capture_robot_clock(msg)
        self._last_odom_monotonic = time.monotonic()
        if time.monotonic() - self._last_amcl < 2.0:
            self.state.update_runtime({"lastOdomAt": now_iso()})
            return
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        odom_pose = (float(pose.position.x), float(pose.position.y), yaw)
        self._last_odom_pose = odom_pose
        if self._pending_odom_anchor is not None:
            self._anchor_odom_to_map(*self._pending_odom_anchor)
            self._pending_odom_anchor = None
        x, y, mapped_yaw, source = self._map_pose_from_odom(odom_pose)
        self._last_pose_monotonic = time.monotonic()
        self.state.update_runtime(
            {
                "pose": {
                    "x": x,
                    "y": y,
                    "yaw": mapped_yaw,
                    "source": source,
                    "stamp": now_iso(),
                },
                "lastOdomAt": now_iso(),
                **self._cmd_vel_clock_status(),
            }
        )

    def _static_wall_map_for_setup(self, setup: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        map_config = setup.get("map", {}) or {}
        planner = setup.get("planner", {}) or {}
        source = configured_map_path(map_config)
        source_state: Dict[str, Any] = {}
        if source is not None:
            try:
                stat = source.stat()
                source_state = {"path": str(source), "size": stat.st_size, "mtime": stat.st_mtime_ns}
            except OSError:
                source_state = {"path": str(source), "missing": True}
        signature = json.dumps(
            {
                "source": source_state,
                "map": {
                    key: map_config.get(key)
                    for key in ("imageUrl", "resolution", "originX", "originY")
                },
                "blockedCells": planner.get("blockedCells", []),
                "freeCells": planner.get("freeCells", []),
                "obstacles": setup.get("obstacles", []),
                "objectInflation": (setup.get("object", {}) or {}).get("inflation", 0.0),
            },
            sort_keys=True,
            default=str,
        )
        if self._static_wall_map_cache.get("signature") == signature:
            return self._static_wall_map_cache.get("mapData")

        map_data: Optional[Dict[str, Any]] = None
        try:
            if source is None or not source.exists():
                raise FileNotFoundError("configured map image is unavailable")
            width, height, pixels = load_map_gray(source)
            apply_setup_overlays_to_map(pixels, width, height, setup)
            map_data = {
                "width": width,
                "height": height,
                "pixels": pixels,
                "resolution": float(map_config.get("resolution") or 0.05),
                "originX": float(map_config.get("originX") or 0.0),
                "originY": float(map_config.get("originY") or 0.0),
            }
        except (OSError, TypeError, ValueError):
            map_data = None
        self._static_wall_map_cache = {"signature": signature, "mapData": map_data}
        return map_data

    def _static_wall_matcher(self, setup: Dict[str, Any]) -> Optional[Callable[[float, float], bool]]:
        map_data = self._static_wall_map_for_setup(setup)
        if map_data is None:
            return None
        return lambda x, y: static_map_wall_near(map_data, x, y)

    def _update_dynamic_lidar_obstacles(
        self, points: list[Tuple[float, float]], pose: Optional[Dict[str, Any]], received: float
    ) -> list[Dict[str, Any]]:
        setup = self.state.get_setup()
        planner = setup.get("planner", {}) or {}
        if not bool(planner.get("detectLidarObstacles", True)):
            self._dynamic_lidar_obstacle_tracks.clear()
            return []

        static_wall_match = self._static_wall_matcher(setup)
        for obstacle in lidar_dynamic_obstacle_cells(points, pose, static_wall_match=static_wall_match):
            track = dict(obstacle)
            track["lastSeen"] = received
            self._dynamic_lidar_obstacle_tracks[track["id"]] = track
        self._dynamic_lidar_obstacle_tracks = {
            obstacle_id: track
            for obstacle_id, track in self._dynamic_lidar_obstacle_tracks.items()
            if received - float(track.get("lastSeen", 0.0)) <= DYNAMIC_LIDAR_OBSTACLE_HOLD_SECONDS
        }
        return [
            {
                key: value
                for key, value in track.items()
                if key != "lastSeen"
            }
            for _, track in sorted(self._dynamic_lidar_obstacle_tracks.items())
        ]

    def _replan_lidar_fallback(
        self, pose: Dict[str, Any], route: list[Dict[str, Any]], route_index: int
    ) -> Tuple[list[Dict[str, Any]], str]:
        snapshot = self.state.snapshot()
        runtime = snapshot.get("runtime", {}) or {}
        dynamic_obstacles = runtime.get("dynamicLidarObstacles", []) or []
        return fallback_replan_path(
            snapshot.get("setup", {}) or {},
            pose,
            route,
            route_index,
            dynamic_obstacles,
            runtime.get("robotPoses", {}) or {},
        )

    def _on_scan(self, msg: Any) -> None:
        received = time.monotonic()
        points = laser_scan_points(
            msg.ranges,
            float(msg.angle_min),
            float(msg.angle_increment),
            float(msg.range_min),
            float(msg.range_max),
        )
        with self._scan_lock:
            self._latest_scan = {
                "points": points,
                "received": received,
                "frame": str(getattr(msg.header, "frame_id", "") or ""),
            }
        changed_at = now_iso()
        runtime = self.state.snapshot().get("runtime", {})
        dynamic_obstacles = self._update_dynamic_lidar_obstacles(
            points, runtime.get("pose"), received
        )
        patch: Dict[str, Any] = {"lastScanAt": changed_at, "scanAgeMs": 0}
        if runtime.get("lidarConnection") != "connected":
            patch.update(
                {
                    "lidarConnection": "connected",
                    "lidarConnectionMessage": "LiDAR 연결 정상 · 속도 복구",
                    "lidarConnectionChangedAt": changed_at,
                }
            )
        if received - self._last_lidar_display >= 0.2:
            self._last_lidar_display = received
            display_pose = runtime.get("pose")
            patch.update(
                {
                    "lidarPoints": sample_lidar_points(points),
                    "lidarPose": display_pose,
                    "lidarPointCount": len(points),
                    "lidarFrame": str(getattr(msg.header, "frame_id", "") or ""),
                    "dynamicLidarObstacles": dynamic_obstacles,
                    "dynamicLidarObstacleCount": len(dynamic_obstacles),
                }
            )
        self.state.update_runtime(patch)
        if received - self._last_sensor_run_log >= 1.0:
            self._last_sensor_run_log = received
            runtime = self.state.snapshot().get("runtime", {})
            odom_age = (
                received - self._last_odom_monotonic
                if self._last_odom_monotonic > 0
                else math.inf
            )
            pose_age = (
                received - self._last_pose_monotonic
                if self._last_pose_monotonic > 0
                else math.inf
            )
            self.state.log_run_event(
                "sensor_sample",
                scanPointCount=len(points),
                dynamicLidarObstacleCount=len(dynamic_obstacles),
                scanFrame=str(getattr(msg.header, "frame_id", "") or ""),
                scanAgeMs=0,
                odomAgeMs=None if not math.isfinite(odom_age) else round(odom_age * 1000),
                poseAgeMs=None if not math.isfinite(pose_age) else round(pose_age * 1000),
                pose=runtime.get("pose"),
            )

    def _on_image(self, msg: Any) -> None:
        if not self.state.camera_enabled() or self.state.camera_stream_mode() != "raw":
            return
        if time.monotonic() - self._last_raw_camera < 0.2:
            return
        self._last_raw_camera = time.monotonic()
        bmp = image_msg_to_bmp(msg)
        if bmp:
            self.state.set_camera(bmp, "image/bmp", "raw")

    def _on_compressed_image(self, msg: Any) -> None:
        if not self.state.camera_enabled() or self.state.camera_stream_mode() != "compressed":
            return
        fmt = str(msg.format).lower()
        content_type = "image/png" if "png" in fmt else "image/jpeg"
        self.state.set_camera(bytes(msg.data), content_type, "compressed")

    def _reset_robot_clock(self) -> None:
        with self._robot_clock_lock:
            self._robot_clock_anchor_ns = None
            self._robot_clock_received_monotonic_ns = None
            self._robot_clock_skew_ms = None
            self._last_clock_runtime_update = 0.0
        self.state.update_runtime(
            {
                "cmdVelStampSource": "server_clock",
                "cmdVelClockSkewMs": None,
                "cmdVelClockAgeMs": None,
            }
        )

    def _capture_robot_clock(self, msg: Any) -> None:
        stamp_ns = stamp_to_nanoseconds(getattr(getattr(msg, "header", None), "stamp", None))
        if stamp_ns is None:
            return
        received_monotonic_ns = time.monotonic_ns()
        try:
            local_clock_ns = int(self.node.get_clock().now().nanoseconds)
            skew_ms: Optional[float] = (stamp_ns - local_clock_ns) / 1_000_000.0
        except Exception:
            skew_ms = None
        should_update_runtime = False
        with self._robot_clock_lock:
            self._robot_clock_anchor_ns = stamp_ns
            self._robot_clock_received_monotonic_ns = received_monotonic_ns
            self._robot_clock_skew_ms = skew_ms
            now_monotonic = time.monotonic()
            if now_monotonic - self._last_clock_runtime_update >= 1.0:
                self._last_clock_runtime_update = now_monotonic
                should_update_runtime = True
        if should_update_runtime:
            self.state.update_runtime(self._cmd_vel_clock_status())

    def _cmd_vel_clock_status(self) -> Dict[str, Any]:
        now_monotonic_ns = time.monotonic_ns()
        with self._robot_clock_lock:
            anchor_ns = self._robot_clock_anchor_ns
            received_ns = self._robot_clock_received_monotonic_ns
            skew_ms = self._robot_clock_skew_ms
        age_ms: Optional[float] = None
        fresh = False
        if anchor_ns is not None and received_ns is not None:
            age_ms = max(0, now_monotonic_ns - received_ns) / 1_000_000.0
            fresh = age_ms <= ROBOT_CLOCK_FRESH_SECONDS * 1000.0
        if getattr(self, "cmd_vel_msg_type", TWIST_STAMPED_TYPE) != TWIST_STAMPED_TYPE:
            source = "unstamped"
        else:
            source = "robot_odom" if fresh else "server_clock"
        return {
            "cmdVelStampSource": source,
            "cmdVelClockSkewMs": round(skew_ms, 3) if skew_ms is not None else None,
            "cmdVelClockAgeMs": round(age_ms, 1) if age_ms is not None else None,
        }

    def _cmd_vel_stamp(self) -> Tuple[Any, str]:
        now_monotonic_ns = time.monotonic_ns()
        with self._robot_clock_lock:
            anchor_ns = self._robot_clock_anchor_ns
            received_ns = self._robot_clock_received_monotonic_ns
        if anchor_ns is not None and received_ns is not None:
            age_ns = max(0, now_monotonic_ns - received_ns)
            if age_ns <= int(ROBOT_CLOCK_FRESH_SECONDS * NANOSECONDS_PER_SECOND):
                estimated_ns = extrapolate_clock_nanoseconds(anchor_ns, received_ns, now_monotonic_ns)
                stamp = self.node.get_clock().now().to_msg()
                stamp.sec = estimated_ns // NANOSECONDS_PER_SECOND
                stamp.nanosec = estimated_ns % NANOSECONDS_PER_SECOND
                return stamp, "robot_odom"
        return self.node.get_clock().now().to_msg(), "server_clock"

    def _anchor_odom_to_map(self, map_x: float, map_y: float, map_yaw: float) -> Optional[Dict[str, Any]]:
        map_pose = (float(map_x), float(map_y), normalize_yaw(float(map_yaw)))
        if self._last_odom_pose is None:
            self._pending_odom_anchor = map_pose
            self._odom_anchor = None
            self.state.update_runtime(
                {
                    "odomAnchor": {
                        "pendingMap": {"x": map_pose[0], "y": map_pose[1], "yaw": map_pose[2]},
                        "stamp": now_iso(),
                    }
                }
            )
            return None
        self._odom_anchor = {"odom": self._last_odom_pose, "map": map_pose}
        anchor_payload = {
            "odom": {
                "x": self._last_odom_pose[0],
                "y": self._last_odom_pose[1],
                "yaw": self._last_odom_pose[2],
            },
            "map": {"x": map_pose[0], "y": map_pose[1], "yaw": map_pose[2]},
            "stamp": now_iso(),
        }
        self.state.update_runtime({"odomAnchor": anchor_payload})
        return anchor_payload

    def _map_pose_from_odom(self, odom_pose: Tuple[float, float, float]) -> Tuple[float, float, float, str]:
        if not self._odom_anchor:
            return odom_pose[0], odom_pose[1], odom_pose[2], self.topics["odom"]
        anchor_odom = self._odom_anchor["odom"]
        anchor_map = self._odom_anchor["map"]
        delta_x = odom_pose[0] - anchor_odom[0]
        delta_y = odom_pose[1] - anchor_odom[1]
        theta = anchor_map[2] - anchor_odom[2]
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        x = anchor_map[0] + cos_t * delta_x - sin_t * delta_y
        y = anchor_map[1] + sin_t * delta_x + cos_t * delta_y
        yaw = normalize_yaw(anchor_map[2] + normalize_yaw(odom_pose[2] - anchor_odom[2]))
        return x, y, yaw, f"{self.topics['odom']}+initial"

    def _action_available(self, timeout_sec: float = 0.05) -> bool:
        return bool(self.action_client is not None and self.action_client.wait_for_server(timeout_sec=timeout_sec))

    def _route_action_available(self, timeout_sec: float = 0.05) -> bool:
        return bool(
            self.route_action_client is not None
            and self.route_action_client.wait_for_server(timeout_sec=timeout_sec)
        )

    def set_initial_pose(self, x: float, y: float, yaw: float) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.set_initial_pose(x, y, yaw)
        self.state.log_run_event("initial_pose_requested", pose={"x": x, "y": y, "yaw": yaw})
        msg = self.PoseWithCovarianceStamped()
        msg.header.frame_id = self.topics["mapFrame"]
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        initial_pose_subscribers = self._endpoint_count(self.topics["initialPose"], "subscriptions")
        self.initial_pose_pub.publish(msg)
        anchor = self._anchor_odom_to_map(float(x), float(y), float(yaw))
        if initial_pose_subscribers > 0:
            nav_message = (
                "Initial pose published to AMCL/Nav2. Odom display anchored."
                if anchor
                else "Initial pose published to AMCL/Nav2. Waiting for odom to anchor display."
            )
            nav_status = "initial_pose_set"
        else:
            nav_message = (
                f"Initial pose published, but {self.topics['initialPose']} has no subscribers. "
                "Only dashboard odom display was anchored."
            )
            nav_status = "initial_pose_local_only"
        self.state.update_runtime(
            {
                "pose": {"x": x, "y": y, "yaw": yaw, "source": "initialpose", "stamp": now_iso()},
                "navStatus": nav_status,
                "navMessage": nav_message,
                "lastCommandAt": now_iso(),
            }
        )
        return {"ok": True, "odomAnchor": anchor, "subscribers": initial_pose_subscribers}

    def _pose_stamped(self, x: float, y: float, yaw: float) -> Any:
        pose = self.PoseStamped()
        pose.header.frame_id = self.topics["mapFrame"]
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _stop_lidar_fallback(self, publish_stop: bool = True) -> bool:
        with self._fallback_nav_lock:
            was_active = self._fallback_nav_active
            self._fallback_nav_generation += 1
            self._fallback_nav_active = False
        if was_active and publish_stop and not self._is_fallback() and self.node is not None:
            try:
                self._publish_cmd_vel(0.0, 0.0, repeats=6, interval=0.04)
            except Exception:
                pass
        if was_active:
            self.state.update_runtime(
                {
                    "fallbackActive": False,
                    "fallbackSpeedScale": None,
                    "lidarMinClearance": None,
                    "fallbackRecoveryPhase": None,
                    "fallbackRecoveryAttempts": 0,
                }
            )
        return was_active

    def _fallback_generation_current(self, generation: int) -> bool:
        with self._fallback_nav_lock:
            return self._fallback_nav_active and self._fallback_nav_generation == generation

    def _fallback_request_current(self, generation: int) -> bool:
        with self._fallback_nav_lock:
            return self._fallback_nav_generation == generation

    def _fallback_sensor_ages(self) -> Dict[str, float]:
        now = time.monotonic()
        with self._scan_lock:
            scan_received = float(self._latest_scan.get("received") or 0.0)
        return {
            "scan": max(0.0, now - scan_received) if scan_received > 0 else math.inf,
            "odom": (
                max(0.0, now - self._last_odom_monotonic)
                if self._last_odom_monotonic > 0
                else math.inf
            ),
            "pose": (
                max(0.0, now - self._last_pose_monotonic)
                if self._last_pose_monotonic > 0
                else math.inf
            ),
        }

    def _wait_for_fresh_fallback_sensors(
        self, settings: Dict[str, Any], timeout: float
    ) -> Dict[str, float]:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while True:
            ages = self._fallback_sensor_ages()
            if (
                ages["scan"] <= settings["scanTimeout"]
                and ages["odom"] <= settings["odomTimeout"]
                and ages["pose"] <= settings["odomTimeout"]
            ):
                return ages
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return ages
            time.sleep(min(0.05, remaining))

    @staticmethod
    def _sensor_age_text(age: float) -> str:
        return "missing" if not math.isfinite(age) else f"{age * 1000:.0f} ms"

    def _start_lidar_fallback(
        self,
        path: list[Dict[str, Any]],
        route: list[Dict[str, float]],
        queue_if_stale: bool = False,
        repeat: Optional[Dict[str, Any]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        repeat_cycle: int = 1,
    ) -> Dict[str, Any]:
        fallback_setup = self.state.get_setup()
        settings = normalized_fallback_settings(fallback_setup)
        if not settings["enabled"]:
            return {"ok": False, "error": "LiDAR fallback navigation is disabled."}
        if not path:
            return {"ok": False, "error": "A* path is required for LiDAR fallback navigation."}
        repeat_config = normalized_route_repeat(repeat, len(route))
        if repeat_config["enabled"] and not repeat_path:
            return {
                "ok": False,
                "error": "LiDAR route repetition requires a precomputed return A* path.",
            }
        ages = self._fallback_sensor_ages()
        if (
            ages["scan"] > settings["scanTimeout"]
            or ages["odom"] > settings["odomTimeout"]
            or ages["pose"] > settings["odomTimeout"]
        ):
            wait_timeout = max(
                0.0,
                float(
                    getattr(
                        self,
                        "_fallback_sensor_startup_wait",
                        FALLBACK_SENSOR_STARTUP_WAIT,
                    )
                ),
            )
            self.state.update_runtime(
                {
                    "navStatus": "fallback_waiting_sensors",
                    "navMessage": (
                        f"Waiting up to {wait_timeout:.1f}s for fresh "
                        f"{self.topics['scan']} and {self.topics['odom']}."
                    ),
                }
            )
            ages = self._wait_for_fresh_fallback_sensors(settings, wait_timeout)
        scan_age = ages["scan"]
        odom_age = ages["odom"]
        pose_age = ages["pose"]
        if scan_age > settings["scanTimeout"]:
            if queue_if_stale:
                return self._queue_lidar_fallback_until_fresh(
                    path, route, settings, fallback_setup, repeat_config, repeat_path, repeat_cycle
                )
            return {
                "ok": False,
                "error": (
                    f"LiDAR fallback refused: {self.topics['scan']} age "
                    f"{self._sensor_age_text(scan_age)} exceeds "
                    f"{settings['scanTimeout'] * 1000:.0f} ms."
                ),
            }
        if odom_age > settings["odomTimeout"] or pose_age > settings["odomTimeout"]:
            if queue_if_stale:
                return self._queue_lidar_fallback_until_fresh(
                    path, route, settings, fallback_setup, repeat_config, repeat_path, repeat_cycle
                )
            return {
                "ok": False,
                "error": (
                    f"LiDAR fallback refused: {self.topics['odom']} age "
                    f"{self._sensor_age_text(odom_age)}, mapped pose age "
                    f"{self._sensor_age_text(pose_age)}; limit "
                    f"{settings['odomTimeout'] * 1000:.0f} ms."
                ),
            }
        cmd_publishers = self._endpoint_count(self.topics["cmdVel"], "publishers")
        cmd_subscribers = self._endpoint_count(self.topics["cmdVel"], "subscriptions")
        if cmd_subscribers <= 0:
            return {
                "ok": False,
                "error": (
                    f"LiDAR fallback refused: {self.topics['cmdVel']} has no robot subscriber."
                ),
            }
        if cmd_publishers > 1:
            return {
                "ok": False,
                "error": (
                    f"LiDAR fallback refused: {self.topics['cmdVel']} has {cmd_publishers} publishers. "
                    "Stop teleop/Nav2 cmd_vel publishers first."
                ),
            }

        clean_path = [dict(point) for point in path]
        clean_route = [dict(pose, stamp=now_iso()) for pose in route]
        clean_repeat_path = [dict(point) for point in (repeat_path or [])]
        final_yaw = float(clean_route[-1].get("yaw", 0.0))
        with self._fallback_nav_lock:
            self._fallback_nav_generation += 1
            generation = self._fallback_nav_generation
            self._fallback_nav_active = True
        self.state.update_runtime(
            {
                "goal": clean_route[0],
                "route": clean_route,
                "routeIndex": 0,
                "navStatus": "fallback_starting",
                "navMessage": f"LiDAR fallback accepted: {len(clean_path)} path points.",
                "fallbackActive": True,
                "fallbackPathIndex": 0,
                "fallbackPathLength": len(clean_path),
                "fallbackDisplayPath": sample_navigation_path(clean_path),
                "fallbackSpeedScale": 0.0,
                "fallbackRecoveryPhase": "none",
                "fallbackRecoveryAttempts": 0,
                "routeRepeatEnabled": repeat_config["enabled"],
                "routeRepeatCycle": repeat_cycle,
                "routeRepeatPauseSeconds": repeat_config["pauseSeconds"],
                "routeRepeatResumeAt": None,
                "lastCommandAt": now_iso(),
            }
        )
        self._fallback_nav_thread = threading.Thread(
            target=self._fallback_navigation_loop,
            args=(
                generation, clean_path, clean_route, final_yaw, settings, fallback_setup,
                repeat_config, clean_repeat_path, repeat_cycle,
            ),
            daemon=True,
        )
        self._fallback_nav_thread.start()
        return {
            "ok": True,
            "transport": "lidar_fallback",
            "pathLength": len(clean_path),
            "routeLength": len(clean_route),
            "repeat": repeat_config,
        }

    def _queue_lidar_fallback_until_fresh(
        self,
        path: list[Dict[str, Any]],
        route: list[Dict[str, float]],
        settings: Dict[str, Any],
        fallback_setup: Dict[str, Any],
        repeat: Optional[Dict[str, Any]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        repeat_cycle: int = 1,
    ) -> Dict[str, Any]:
        clean_path = [dict(point) for point in path]
        clean_route = [dict(pose, stamp=now_iso()) for pose in route]
        clean_repeat_path = [dict(point) for point in (repeat_path or [])]
        repeat_config = normalized_route_repeat(repeat, len(clean_route))
        with self._fallback_nav_lock:
            self._fallback_nav_generation += 1
            generation = self._fallback_nav_generation
            self._fallback_nav_active = False
        self.state.update_runtime(
            {
                "goal": clean_route[0],
                "route": clean_route,
                "routeIndex": 0,
                "navStatus": "fallback_waiting_sensors",
                "navMessage": (
                    f"Waiting up to {FALLBACK_SENSOR_PENDING_WAIT:.0f}s for fresh "
                    f"{self.topics['scan']} and {self.topics['odom']}; the goal is queued."
                ),
                "fallbackActive": False,
                "fallbackPathIndex": 0,
                "fallbackPathLength": len(clean_path),
                "fallbackSpeedScale": 0.0,
                "fallbackRecoveryPhase": "none",
                "fallbackRecoveryAttempts": 0,
                "routeRepeatEnabled": repeat_config["enabled"],
                "routeRepeatCycle": repeat_cycle,
                "routeRepeatPauseSeconds": repeat_config["pauseSeconds"],
                "routeRepeatResumeAt": None,
                "lastCommandAt": now_iso(),
            }
        )
        self._fallback_nav_thread = threading.Thread(
            target=self._pending_lidar_fallback_loop,
            args=(
                generation, clean_path, clean_route, settings, fallback_setup,
                repeat_config, clean_repeat_path, repeat_cycle,
            ),
            daemon=True,
        )
        self._fallback_nav_thread.start()
        return {
            "ok": True,
            "transport": "lidar_fallback_waiting",
            "message": "Goal queued; LiDAR fallback will start automatically when scan/odom recover.",
            "pathLength": len(clean_path),
            "routeLength": len(clean_route),
        }

    def _pending_lidar_fallback_loop(
        self,
        generation: int,
        path: list[Dict[str, Any]],
        route: list[Dict[str, float]],
        settings: Dict[str, Any],
        fallback_setup: Dict[str, Any],
        repeat: Optional[Dict[str, Any]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        repeat_cycle: int = 1,
    ) -> None:
        deadline = time.monotonic() + FALLBACK_SENSOR_PENDING_WAIT
        while (
            not self._shutdown_event.is_set()
            and self._fallback_request_current(generation)
        ):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                ages = self._fallback_sensor_ages()
                self.state.update_runtime(
                    {
                        "goal": None,
                        "route": [],
                        "routeIndex": 0,
                        "navStatus": "fallback_sensor_timeout",
                        "navMessage": (
                            "Queued LiDAR fallback timed out: "
                            f"scan {self._sensor_age_text(ages['scan'])}, "
                            f"odom {self._sensor_age_text(ages['odom'])}, "
                            f"pose {self._sensor_age_text(ages['pose'])}."
                        ),
                        "fallbackActive": False,
                        "fallbackSpeedScale": 0.0,
                        "fallbackRecoveryPhase": None,
                        "fallbackRecoveryAttempts": 0,
                    }
                )
                return
            ages = self._wait_for_fresh_fallback_sensors(
                settings, min(0.5, remaining)
            )
            if (
                ages["scan"] > settings["scanTimeout"]
                or ages["odom"] > settings["odomTimeout"]
                or ages["pose"] > settings["odomTimeout"]
            ):
                continue
            if not self._fallback_request_current(generation):
                return
            result = self._start_lidar_fallback(
                path, route, queue_if_stale=False, repeat=repeat,
                repeat_path=repeat_path, repeat_cycle=repeat_cycle,
            )
            if result.get("ok"):
                return
            self.state.update_runtime(
                {
                    "goal": None,
                    "route": [],
                    "routeIndex": 0,
                    "navStatus": "fallback_error",
                    "navMessage": str(result.get("error") or "LiDAR fallback failed."),
                    "fallbackActive": False,
                    "fallbackSpeedScale": 0.0,
                    "fallbackRecoveryPhase": None,
                    "fallbackRecoveryAttempts": 0,
                }
            )
            return

    def _fallback_navigation_loop(
        self,
        generation: int,
        path: list[Dict[str, Any]],
        route: list[Dict[str, float]],
        final_yaw: float,
        settings: Dict[str, Any],
        fallback_setup: Dict[str, Any],
        repeat: Optional[Dict[str, Any]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        repeat_cycle: int = 1,
    ) -> None:
        footprint = effective_footprint_from_setup(fallback_setup)
        path_index = 0
        route_index = 0
        route_path_ends = fallback_route_path_ends(path, len(route))
        if route_path_ends:
            route_index = min(route_path_ends)
        last_runtime_update = 0.0
        last_run_log = 0.0
        recovery: Dict[str, Any] = {
            "phase": "none",
            "started": 0.0,
            "attempts": 0,
            "turn": 0,
        }
        last_safe_safety: Optional[Dict[str, Any]] = None
        recovery_started_at: Optional[float] = None
        last_replan_at = -math.inf
        dynamic_replan_clearance_x, dynamic_replan_clearance_y = (
            planning_clearance_extents_from_setup(fallback_setup)
        )
        period = 0.1
        try:
            while not self._shutdown_event.is_set() and self._fallback_generation_current(generation):
                cycle_started = time.monotonic()
                with self._scan_lock:
                    scan = {
                        "points": list(self._latest_scan.get("points") or []),
                        "received": float(self._latest_scan.get("received") or 0.0),
                    }
                scan_age = (
                    cycle_started - scan["received"] if scan["received"] > 0 else math.inf
                )
                odom_age = (
                    cycle_started - self._last_odom_monotonic
                    if self._last_odom_monotonic > 0
                    else math.inf
                )
                pose_age = (
                    cycle_started - self._last_pose_monotonic
                    if self._last_pose_monotonic > 0
                    else math.inf
                )
                if odom_age > settings["odomTimeout"] or pose_age > settings["odomTimeout"]:
                    recovery = {
                        "phase": "none",
                        "started": 0.0,
                        "attempts": 0,
                        "turn": 0,
                    }
                    self._publish_cmd_vel(0.0, 0.0)
                    if not self._fallback_generation_current(generation):
                        return
                    if cycle_started - last_runtime_update >= 0.2:
                        missing = [
                            name
                            for name, age, limit in (
                                ("odom", odom_age, settings["odomTimeout"]),
                                ("pose", pose_age, settings["odomTimeout"]),
                            )
                            if age > limit
                        ]
                        self.state.update_runtime(
                            {
                                "navStatus": "fallback_sensor_stop",
                                "navMessage": f"LiDAR fallback stopped: stale {', '.join(missing)}.",
                                "fallbackSpeedScale": 0.0,
                                "fallbackRecoveryPhase": None,
                                "fallbackRecoveryAttempts": recovery["attempts"],
                                "lidarConnection": "lost",
                                "lidarConnectionMessage": "위치 데이터 끊김 · 안전 정지",
                                "lidarConnectionChangedAt": now_iso(),
                                "scanAgeMs": None if not math.isfinite(scan_age) else round(scan_age * 1000),
                                "odomAgeMs": (
                                    None
                                    if not math.isfinite(max(odom_age, pose_age))
                                    else round(max(odom_age, pose_age) * 1000)
                                ),
                                "lastCommandAt": now_iso(),
                            }
                        )
                        last_runtime_update = cycle_started
                    self._shutdown_event.wait(max(0.0, period - (time.monotonic() - cycle_started)))
                    continue

                runtime = self.state.snapshot()["runtime"]
                pose = runtime.get("pose") or {}
                if not all(key in pose for key in ("x", "y", "yaw")):
                    recovery = {
                        "phase": "none",
                        "started": 0.0,
                        "attempts": 0,
                        "turn": 0,
                    }
                    self._publish_cmd_vel(0.0, 0.0)
                    if not self._fallback_generation_current(generation):
                        return
                    self.state.update_runtime(
                        {
                            "navStatus": "fallback_sensor_stop",
                            "navMessage": "LiDAR fallback stopped: current pose is missing.",
                            "fallbackSpeedScale": 0.0,
                        }
                    )
                    self._shutdown_event.wait(period)
                    continue

                command = compute_fallback_command(
                    pose,
                    path,
                    path_index,
                    final_yaw,
                    settings,
                    path_end_index=route_path_ends.get(route_index),
                )
                if command.get("error"):
                    raise RuntimeError(str(command["error"]))
                path_index = int(command.get("pathIndex", path_index))
                if command.get("reached"):
                    self._publish_cmd_vel(0.0, 0.0, repeats=12, interval=0.04)
                    with self._fallback_nav_lock:
                        is_current = self._fallback_nav_generation == generation
                        if is_current:
                            self._fallback_nav_active = False
                    if not is_current:
                        return
                    repeat_config = normalized_route_repeat(repeat, len(route))
                    if repeat_config["enabled"] and repeat_path:
                        pause_seconds = float(repeat_config["pauseSeconds"])
                        resume_at = time.strftime(
                            "%Y-%m-%dT%H:%M:%S",
                            time.localtime(time.time() + pause_seconds),
                        )
                        self.state.log_run_event(
                            "lidar_repeat_paused",
                            cycle=repeat_cycle,
                            pauseSeconds=pause_seconds,
                            returnPathPointCount=len(repeat_path),
                        )
                        self.state.update_runtime(
                            {
                                "goal": None,
                                "route": route,
                                "routeIndex": len(route) - 1,
                                "navStatus": "route_pause",
                                "navMessage": (
                                    f"LiDAR A* cycle {repeat_cycle} complete; "
                                    f"resuming the return path in {pause_seconds:.1f}s."
                                ),
                                "fallbackActive": False,
                                "fallbackPathIndex": len(path) - 1,
                                "fallbackSpeedScale": 0.0,
                                "fallbackRecoveryPhase": None,
                                "fallbackRecoveryAttempts": 0,
                                "routeRepeatEnabled": True,
                                "routeRepeatCycle": repeat_cycle,
                                "routeRepeatPauseSeconds": pause_seconds,
                                "routeRepeatResumeAt": resume_at,
                                "lastCommandAt": now_iso(),
                            }
                        )
                        threading.Thread(
                            target=self._resume_lidar_fallback_repeat,
                            args=(generation, repeat_path, route, repeat_config, repeat_cycle),
                            daemon=True,
                        ).start()
                        return
                    self.state.update_runtime(
                        {
                            "goal": None,
                            "route": [],
                            "routeIndex": 0,
                            "navStatus": "succeeded",
                            "navMessage": "LiDAR fallback route complete.",
                            "fallbackActive": False,
                            "fallbackPathIndex": len(path) - 1,
                            "fallbackSpeedScale": 0.0,
                            "fallbackRecoveryPhase": None,
                            "fallbackRecoveryAttempts": 0,
                            "lastCommandAt": now_iso(),
                        }
                    )
                    return

                if scan_age > settings["scanTimeout"]:
                    recovery = {
                        "phase": "none",
                        "started": 0.0,
                        "attempts": 0,
                        "turn": 0,
                    }
                    if lidar_coast_allowed(
                        scan_age, odom_age, pose_age, settings, last_safe_safety
                    ):
                        linear, angular = lidar_coast_command(command, settings)
                        requested_linear = float(command.get("linear") or 0.0)
                        speed_scale = (
                            abs(linear) / abs(requested_linear)
                            if abs(requested_linear) > 1e-4
                            else 1.0
                        )
                        if not self._fallback_generation_current(generation):
                            return
                        self._publish_cmd_vel(linear, angular)
                        checkpoint_skipped_indices: list[int] = []
                        while route_index < len(route) - 1:
                            route_target = route[route_index]
                            route_distance = math.hypot(
                                float(route_target["x"]) - float(pose["x"]),
                                float(route_target["y"]) - float(pose["y"]),
                            )
                            if route_distance > max(
                                settings["lookahead"], settings["goalTolerance"] * 1.5
                            ):
                                break
                            next_route_index = fallback_next_route_index(
                                route_index, len(route), route_path_ends
                            )
                            if next_route_index is None:
                                break
                            if next_route_index > route_index + 1:
                                skipped_indices = list(
                                    range(route_index + 1, next_route_index)
                                )
                                checkpoint_skipped_indices.extend(skipped_indices)
                                self.state.log_run_event(
                                    "fallback_waypoints_skipped",
                                    skippedRouteIndices=skipped_indices,
                                    nextRouteIndex=next_route_index,
                                    reason="no_astar_path",
                                )
                            route_index = next_route_index
                        if cycle_started - last_run_log >= 0.5:
                            self.state.log_run_event(
                                "fallback_lidar_coast",
                                pose=pose,
                                pathIndex=path_index,
                                pathLength=len(path),
                                routeIndex=route_index,
                                command={"linear": linear, "angular": angular},
                                scanAgeMs=round(scan_age * 1000),
                                odomAgeMs=round(max(odom_age, pose_age) * 1000),
                                lastSafety=last_safe_safety,
                            )
                            last_run_log = cycle_started
                        if cycle_started - last_runtime_update >= 0.2:
                            clearance = last_safe_safety.get("minClearance")
                            connection_changed = runtime.get("lidarConnection") != "degraded"
                            self.state.update_runtime(
                                {
                                    "goal": route[route_index],
                                    "routeIndex": route_index,
                                    "_routeCheckpointSkippedIndices": checkpoint_skipped_indices,
                                    "navStatus": "fallback_lidar_coast",
                                    "navMessage": (
                                        "LiDAR scan 끊김 · 마지막 안전 scan 기준 저속 주행 "
                                        f"({scan_age - settings['scanTimeout']:.1f}/"
                                        f"{FALLBACK_LIDAR_COAST_SECONDS:.1f}s)."
                                    ),
                                    "fallbackActive": True,
                                    "fallbackPathIndex": path_index,
                                    "fallbackSpeedScale": round(speed_scale, 3),
                                    "lidarMinClearance": (
                                        None if clearance is None else round(float(clearance), 3)
                                    ),
                                    "fallbackRecoveryPhase": "none",
                                    "fallbackRecoveryAttempts": 0,
                                    "lidarConnection": "degraded",
                                    "lidarConnectionMessage": "LiDAR 연결 끊김 · 마지막 안전 scan 기준 저속 주행",
                                    "lidarConnectionChangedAt": (
                                        now_iso() if connection_changed else runtime.get("lidarConnectionChangedAt")
                                    ),
                                    "scanAgeMs": round(scan_age * 1000),
                                    "odomAgeMs": round(odom_age * 1000),
                                    "lastCommandAt": now_iso(),
                                }
                            )
                            last_runtime_update = cycle_started
                        self._shutdown_event.wait(
                            max(0.0, period - (time.monotonic() - cycle_started))
                        )
                        continue

                    self._publish_cmd_vel(0.0, 0.0)
                    if not self._fallback_generation_current(generation):
                        return
                    if cycle_started - last_runtime_update >= 0.2:
                        self.state.update_runtime(
                            {
                                "navStatus": "fallback_sensor_stop",
                                "navMessage": "LiDAR fallback stopped: stale scan.",
                                "fallbackSpeedScale": 0.0,
                                "fallbackRecoveryPhase": None,
                                "fallbackRecoveryAttempts": 0,
                                "lidarConnection": "lost",
                                "lidarConnectionMessage": "LiDAR 연결 끊김 · 안전 정지",
                                "lidarConnectionChangedAt": now_iso(),
                                "scanAgeMs": None if not math.isfinite(scan_age) else round(scan_age * 1000),
                                "odomAgeMs": round(max(odom_age, pose_age) * 1000),
                                "lastCommandAt": now_iso(),
                            }
                        )
                        last_runtime_update = cycle_started
                    self._shutdown_event.wait(max(0.0, period - (time.monotonic() - cycle_started)))
                    continue

                safety = evaluate_lidar_safety(
                    scan["points"],
                    float(command.get("linear", 0.0)),
                    float(command.get("angular", 0.0)),
                    footprint,
                    settings["hardMargin"],
                    settings["softDistance"],
                    settings["collisionHorizon"],
                )
                dynamic_obstacles = runtime.get("dynamicLidarObstacles") or []
                dynamic_path_blocked = fallback_path_hits_dynamic_obstacle(
                    path,
                    path_index,
                    pose,
                    dynamic_obstacles,
                    clearance_x=dynamic_replan_clearance_x,
                    clearance_y=dynamic_replan_clearance_y,
                )
                blocked = bool(safety["recoveryRequired"]) or dynamic_path_blocked
                if not blocked and not bool(safety["collision"]):
                    last_safe_safety = dict(safety)
                speed_scale = 1.0
                if blocked:
                    if recovery_started_at is None:
                        recovery_started_at = cycle_started
                    recovery_duration = cycle_started - recovery_started_at
                    if dynamic_path_blocked:
                        replan_reason = "dynamic_obstacle"
                        replan_due = (
                            cycle_started - last_replan_at >= FALLBACK_DYNAMIC_REPLAN_COOLDOWN
                        )
                    else:
                        recovery = next_lidar_recovery_command(
                            scan["points"], footprint, recovery, cycle_started
                        )
                        replan_reason = "recovery"
                        replan_due = fallback_replan_due(
                            recovery_started_at,
                            int(recovery.get("attempts", 0)),
                            cycle_started,
                            last_replan_at,
                        )
                    if replan_due:
                        self._publish_cmd_vel(0.0, 0.0)
                        replanned_path, replan_error = self._replan_lidar_fallback(
                            pose, route, route_index
                        )
                        last_replan_at = cycle_started
                        if replanned_path:
                            path = replanned_path
                            path_index = 0
                            route_path_ends = fallback_route_path_ends(path, len(route))
                            replanned_indices = sorted(
                                index for index in route_path_ends if index >= route_index
                            )
                            skipped_route_indices: list[int] = []
                            if replanned_indices and replanned_indices[0] > route_index:
                                next_route_index = replanned_indices[0]
                                skipped_route_indices = list(
                                    range(route_index, next_route_index)
                                )
                                route_index = next_route_index
                                self.state.log_run_event(
                                    "fallback_waypoints_skipped",
                                    skippedRouteIndices=skipped_route_indices,
                                    nextRouteIndex=route_index,
                                    reason="replan_no_astar_path",
                                )
                            recovery = {
                                "phase": "none",
                                "started": 0.0,
                                "attempts": 0,
                                "turn": 0,
                            }
                            recovery_started_at = None
                            self.state.log_run_event(
                                "fallback_replan_success",
                                pose=pose,
                                routeIndex=route_index,
                                pathLength=len(path),
                                recoveryDurationMs=round(recovery_duration * 1000),
                                reason=replan_reason,
                                dynamicObstacleCount=len(dynamic_obstacles),
                            )
                            self.state.update_runtime(
                                {
                                    "goal": route[route_index],
                                    "routeIndex": route_index,
                                    "_routeCheckpointSkippedIndices": skipped_route_indices,
                                    "navStatus": "fallback_replanned",
                                    "navMessage": (
                                        "LiDAR obstacle reached the active path; "
                                        "A* detour path replaced."
                                        if replan_reason == "dynamic_obstacle"
                                        else "LiDAR recovery persisted; A* detour path replaced."
                                    ) + (
                                        " Unreachable waypoint(s) skipped: "
                                        + ", ".join(
                                            str(index + 1) for index in skipped_route_indices
                                        )
                                        if skipped_route_indices
                                        else ""
                                    ),
                                    "fallbackActive": True,
                                    "fallbackPathIndex": 0,
                                    "fallbackPathLength": len(path),
                                    "fallbackDisplayPath": sample_navigation_path(path),
                                    "fallbackSpeedScale": 0.0,
                                    "fallbackRecoveryPhase": "replanned",
                                    "fallbackRecoveryAttempts": 0,
                                    "lastCommandAt": now_iso(),
                                }
                            )
                        else:
                            self.state.log_run_event(
                                "fallback_replan_failed",
                                pose=pose,
                                routeIndex=route_index,
                                recoveryDurationMs=round(recovery_duration * 1000),
                                reason=replan_reason,
                                dynamicObstacleCount=len(dynamic_obstacles),
                                error=replan_error,
                            )
                            self.state.update_runtime(
                                {
                                    "goal": route[route_index],
                                    "routeIndex": route_index,
                                    "navStatus": "fallback_replan_failed",
                                    "navMessage": (
                                        f"LiDAR obstacle reached the active path; A* replan failed: {replan_error}"
                                        if replan_reason == "dynamic_obstacle"
                                        else f"LiDAR recovery persisted; A* replan failed: {replan_error}"
                                    ),
                                    "fallbackActive": True,
                                    "fallbackSpeedScale": 0.0,
                                    "fallbackRecoveryPhase": "replan_failed",
                                    "fallbackRecoveryAttempts": int(recovery.get("attempts", 0)),
                                    "lastCommandAt": now_iso(),
                                }
                            )
                        self._shutdown_event.wait(
                            max(0.0, period - (time.monotonic() - cycle_started))
                        )
                        continue
                    linear = float(recovery["linear"])
                    angular = float(recovery["angular"])
                    nav_status = f"fallback_recovery_{recovery['phase']}"
                else:
                    recovery_started_at = None
                    recovery = {
                        "phase": "none",
                        "started": 0.0,
                        "attempts": 0,
                        "turn": 0,
                    }
                    requested_linear = float(command.get("linear", 0.0))
                    requested_angular = float(command.get("angular", 0.0))
                    if bool(safety["slow"]) and abs(requested_linear) > 1e-4:
                        linear = math.copysign(
                            min(settings["minLinear"], abs(requested_linear)),
                            requested_linear,
                        )
                        speed_scale = abs(linear) / abs(requested_linear)
                        angular = requested_angular * speed_scale
                    else:
                        linear = requested_linear
                        angular = requested_angular
                    nav_status = (
                        "fallback_slow"
                        if bool(command.get("slow")) or bool(safety["slow"])
                        else "fallback_moving"
                    )
                if not self._fallback_generation_current(generation):
                    return
                self._publish_cmd_vel(linear, angular)

                if cycle_started - last_run_log >= 0.5:
                    clearance = safety.get("minClearance")
                    self.state.log_run_event(
                        "fallback_tick",
                        pose=pose,
                        pathIndex=path_index,
                        pathLength=len(path),
                        routeIndex=route_index,
                        command={"linear": linear, "angular": angular},
                        plannedCommand={
                            "linear": command.get("linear"),
                            "angular": command.get("angular"),
                        },
                        safety={
                            "blocked": blocked,
                            "speedScale": speed_scale,
                            "minClearance": clearance,
                            "motionClearance": safety.get("motionClearance"),
                            "slowZone": bool(safety["slow"]),
                            "recoveryRequired": bool(safety["recoveryRequired"]),
                            "predictedCollision": bool(safety["collision"]),
                        },
                        recovery={
                            "phase": recovery.get("phase"),
                            "attempts": recovery.get("attempts"),
                            "turn": recovery.get("turn"),
                        },
                        scanAgeMs=round(scan_age * 1000),
                        odomAgeMs=round(max(odom_age, pose_age) * 1000),
                    )
                    last_run_log = cycle_started

                checkpoint_skipped_indices = []
                while route_index < len(route) - 1:
                    route_target = route[route_index]
                    route_distance = math.hypot(
                        float(route_target["x"]) - float(pose["x"]),
                        float(route_target["y"]) - float(pose["y"]),
                    )
                    if route_distance > max(settings["lookahead"], settings["goalTolerance"] * 1.5):
                        break
                    next_route_index = fallback_next_route_index(
                        route_index, len(route), route_path_ends
                    )
                    if next_route_index is None:
                        break
                    if next_route_index > route_index + 1:
                        skipped_indices = list(
                            range(route_index + 1, next_route_index)
                        )
                        checkpoint_skipped_indices.extend(skipped_indices)
                        self.state.log_run_event(
                            "fallback_waypoints_skipped",
                            skippedRouteIndices=skipped_indices,
                            nextRouteIndex=next_route_index,
                            reason="no_astar_path",
                        )
                    route_index = next_route_index

                if cycle_started - last_runtime_update >= 0.2:
                    if not self._fallback_generation_current(generation):
                        return
                    clearance = safety.get("minClearance")
                    if blocked:
                        phase = str(recovery["phase"])
                        if phase == "turn":
                            direction = "left" if int(recovery["turn"]) > 0 else "right"
                            message = f"LiDAR recovery: turning {direction} to rejoin the path."
                        elif phase == "forward":
                            message = "LiDAR recovery: forward path is clear; advancing briefly."
                        elif phase == "reverse":
                            message = "LiDAR recovery: rear is clear; reversing briefly."
                        elif phase == "trapped":
                            message = "LiDAR recovery: no safe turn or reverse motion; stopped."
                        else:
                            message = "LiDAR recovery: stopping before escape motion."
                    else:
                        message = (
                            f"LiDAR fallback {path_index + 1}/{len(path)}"
                            f" · v={linear:.2f}, w={angular:.2f}"
                            f" · scale={speed_scale:.2f}"
                        )
                    self.state.update_runtime(
                        {
                            "goal": route[route_index],
                            "routeIndex": route_index,
                            "_routeCheckpointSkippedIndices": checkpoint_skipped_indices,
                            "navStatus": nav_status,
                            "navMessage": message,
                            "fallbackActive": True,
                            "fallbackPathIndex": path_index,
                            "fallbackSpeedScale": round(speed_scale, 3),
                            "lidarMinClearance": None if clearance is None else round(float(clearance), 3),
                            "fallbackRecoveryPhase": recovery["phase"],
                            "fallbackRecoveryAttempts": recovery["attempts"],
                            "scanAgeMs": round(scan_age * 1000),
                            "odomAgeMs": round(odom_age * 1000),
                            "lastCommandAt": now_iso(),
                        }
                    )
                    last_runtime_update = cycle_started
                self._shutdown_event.wait(max(0.0, period - (time.monotonic() - cycle_started)))
        except Exception as exc:
            try:
                self._publish_cmd_vel(0.0, 0.0, repeats=12, interval=0.04)
            except Exception:
                pass
            with self._fallback_nav_lock:
                is_current = self._fallback_nav_generation == generation
                if is_current:
                    self._fallback_nav_active = False
            if not is_current:
                return
            self.state.update_runtime(
                {
                    "navStatus": "fallback_error",
                    "navMessage": f"LiDAR fallback failed: {exc}",
                    "fallbackActive": False,
                    "fallbackSpeedScale": 0.0,
                    "fallbackRecoveryPhase": None,
                    "fallbackRecoveryAttempts": 0,
                }
            )

    def _resume_lidar_fallback_repeat(
        self,
        generation: int,
        repeat_path: list[Dict[str, Any]],
        route: list[Dict[str, float]],
        repeat: Dict[str, Any],
        completed_cycle: int,
    ) -> None:
        pause_seconds = float(repeat.get("pauseSeconds", 0.0))
        if self._shutdown_event.wait(max(0.0, pause_seconds)):
            return
        if not self._fallback_request_current(generation):
            return
        next_cycle = completed_cycle + 1
        self.state.log_run_event(
            "lidar_repeat_resumed",
            cycle=next_cycle,
            returnPathPointCount=len(repeat_path),
        )
        result = self._start_lidar_fallback(
            repeat_path,
            route,
            queue_if_stale=True,
            repeat=repeat,
            repeat_path=repeat_path,
            repeat_cycle=next_cycle,
        )
        if result.get("ok"):
            return
        self.state.update_runtime(
            {
                "goal": None,
                "route": [],
                "routeIndex": 0,
                "navStatus": "fallback_error",
                "navMessage": f"LiDAR repeat failed: {result.get('error') or 'unknown reason'}",
                "fallbackActive": False,
                "routeRepeatEnabled": False,
                "routeRepeatResumeAt": None,
            }
        )

    def _cancel_route_sequence(self, update_runtime: bool = True) -> int:
        with self._route_sequence_lock:
            self._route_sequence_generation += 1
            generation = self._route_sequence_generation
            self._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
            self._route_transport = ""
        if update_runtime:
            self.state.update_runtime(
                {
                    "routeRepeatEnabled": False,
                    "routeRepeatCycle": 0,
                    "routeRepeatPauseSeconds": 0.0,
                    "routeRepeatResumeAt": None,
                }
            )
        return generation

    def _begin_route_sequence(
        self, repeat: Optional[Dict[str, Any]], route_length: int
    ) -> Tuple[int, Dict[str, Any]]:
        repeat_config = normalized_route_repeat(repeat, route_length)
        with self._route_sequence_lock:
            self._route_sequence_generation += 1
            generation = self._route_sequence_generation
            self._route_repeat = repeat_config
            self._route_transport = ""
        return generation, repeat_config

    def _route_sequence_is_current(self, generation: int) -> bool:
        with self._route_sequence_lock:
            return self._route_sequence_generation == generation

    def _set_route_transport(self, generation: int, transport: str) -> bool:
        with self._route_sequence_lock:
            if self._route_sequence_generation != generation:
                return False
            self._route_transport = transport
            return True

    def _retire_route_sequence(self, generation: int) -> bool:
        with self._route_sequence_lock:
            if self._route_sequence_generation != generation:
                return False
            self._route_sequence_generation += 1
            self._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
            self._route_transport = ""
        return True

    def _route_sequence_failed(
        self,
        generation: int,
        nav_status: str,
        message: str,
        clear_goal: bool = True,
    ) -> None:
        if not self._retire_route_sequence(generation):
            return
        self.state.log_run_event(
            "route_sequence_failed",
            navStatus=nav_status,
            message=message,
        )
        self.state.update_runtime(
            {
                "navStatus": nav_status,
                "navMessage": message,
                "goal": None if clear_goal else self.state.snapshot()["runtime"].get("goal"),
                "route": [],
                "routeIndex": 0,
                "routeRepeatEnabled": False,
                "routeRepeatResumeAt": None,
            }
        )

    def _dispatch_route_pose(self, generation: int, route_index: int) -> None:
        if not self._route_sequence_is_current(generation):
            return
        snapshot = self.state.snapshot()
        route = snapshot["runtime"].get("route") or []
        if not 0 <= route_index < len(route):
            self._route_sequence_failed(
                generation,
                "route_error",
                f"Route waypoint index {route_index} is invalid.",
            )
            return
        pose = route[route_index]
        try:
            goal_msg = self.NavigateToPose.Goal()
            goal_msg.pose = self._pose_stamped(
                float(pose["x"]),
                float(pose["y"]),
                float(pose.get("yaw", 0.0)),
            )
            future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback, g=generation, i=route_index: (
                    self._on_route_pose_feedback(feedback, g, i)
                ),
            )
            future.add_done_callback(
                lambda done, g=generation, i=route_index: (
                    self._on_route_pose_goal_response(done, g, i)
                )
            )
            self.state.log_run_event(
                "route_waypoint_dispatched",
                routeIndex=route_index,
                routeLength=len(route),
                goal=pose,
            )
        except Exception as exc:
            self._route_sequence_failed(
                generation,
                "route_error",
                f"Could not send route waypoint {route_index + 1}: {exc}",
            )

    def _on_route_pose_goal_response(
        self, future: Any, generation: int, route_index: int
    ) -> None:
        try:
            handle = future.result()
            if not self._route_sequence_is_current(generation):
                if getattr(handle, "accepted", False):
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                return
            if not handle.accepted:
                self._route_sequence_failed(
                    generation,
                    "rejected",
                    f"Nav2 rejected route waypoint {route_index + 1}.",
                )
                return
            self.goal_handle = handle
            route = self.state.snapshot()["runtime"].get("route") or []
            self.state.update_runtime(
                {
                    "routeIndex": route_index,
                    "goal": route[route_index] if route_index < len(route) else None,
                    "navStatus": "accepted",
                    "navMessage": f"Route waypoint {route_index + 1}/{len(route)} accepted.",
                }
            )
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done, g=generation, i=route_index: (
                    self._on_route_pose_result(done, g, i)
                )
            )
        except Exception as exc:
            self.goal_handle = None
            self._route_sequence_failed(
                generation,
                "route_error",
                f"Route waypoint {route_index + 1} response failed: {exc}",
            )

    def _on_route_pose_feedback(
        self, feedback_msg: Any, generation: int, route_index: int
    ) -> None:
        if not self._route_sequence_is_current(generation):
            return
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, "distance_remaining", None)
        route = self.state.snapshot()["runtime"].get("route") or []
        suffix = "" if remaining is None else f" · {float(remaining):.2f} m remaining"
        self.state.update_runtime(
            {
                "routeIndex": route_index,
                "goal": route[route_index] if route_index < len(route) else None,
                "navStatus": "moving",
                "navMessage": f"Route waypoint {route_index + 1}/{len(route)}{suffix}.",
            }
        )

    def _on_route_pose_result(
        self, future: Any, generation: int, route_index: int
    ) -> None:
        try:
            response = future.result()
            if not self._route_sequence_is_current(generation):
                return
            self.goal_handle = None
            status = int(response.status)
            if status != 4:
                if status == 5:
                    nav_status = "canceled"
                    message = f"Route canceled at waypoint {route_index + 1}."
                else:
                    nav_status = "aborted" if status == 6 else "route_error"
                    message = f"Nav2 waypoint {route_index + 1} failed with status {status}."
                self._route_sequence_failed(generation, nav_status, message)
                return

            snapshot = self.state.snapshot()
            route = snapshot["runtime"].get("route") or []
            self.state.log_run_event(
                "route_waypoint_succeeded",
                routeIndex=route_index,
                routeLength=len(route),
            )
            if route_index < len(route) - 1:
                next_index = route_index + 1
                next_goal = route[next_index]
                self.state.update_runtime(
                    {
                        "goal": next_goal,
                        "routeIndex": next_index,
                        "navStatus": "sending_goal",
                        "navMessage": f"Sending route waypoint {next_index + 1}/{len(route)}.",
                    }
                )
                threading.Thread(
                    target=self._dispatch_route_pose,
                    args=(generation, next_index),
                    daemon=True,
                ).start()
                return

            if self._schedule_route_repeat(generation):
                return
            if not self._retire_route_sequence(generation):
                return
            self.state.update_runtime(
                {
                    "goal": None,
                    "route": [],
                    "routeIndex": 0,
                    "routeRepeatEnabled": False,
                    "routeRepeatResumeAt": None,
                    "navStatus": "succeeded",
                    "navMessage": "Route complete.",
                }
            )
        except Exception as exc:
            self.goal_handle = None
            self._route_sequence_failed(generation, "route_error", str(exc))

    def _schedule_route_repeat(self, generation: int) -> bool:
        with self._route_sequence_lock:
            if self._route_sequence_generation != generation:
                return False
            repeat_config = dict(self._route_repeat)
            transport = self._route_transport
        if not repeat_config.get("enabled"):
            return False
        pause_seconds = float(repeat_config["pauseSeconds"])
        runtime = self.state.snapshot()["runtime"]
        route = runtime.get("route") or []
        if not route:
            self._route_sequence_failed(generation, "route_error", "Repeat route is missing.")
            return True
        cycle = int(runtime.get("routeRepeatCycle") or 1)
        resume_at = time.strftime(
            "%Y-%m-%dT%H:%M:%S",
            time.localtime(time.time() + pause_seconds),
        )
        self.state.log_run_event(
            "route_repeat_paused",
            cycle=cycle,
            pauseSeconds=pause_seconds,
            transport=transport,
        )
        self.state.update_runtime(
            {
                "goal": None,
                "routeIndex": len(route) - 1,
                "routeRepeatResumeAt": resume_at,
                "navStatus": "route_pause",
                "navMessage": (
                    f"Repeat cycle {cycle} complete. "
                    f"Resuming in {pause_seconds:g} seconds."
                ),
            }
        )
        thread = threading.Thread(
            target=self._resume_route_repeat,
            args=(generation, pause_seconds),
            daemon=True,
        )
        with self._route_sequence_lock:
            if self._route_sequence_generation != generation:
                return True
            self._route_pause_thread = thread
        thread.start()
        return True

    def _resume_route_repeat(self, generation: int, pause_seconds: float) -> None:
        if self._shutdown_event.wait(pause_seconds):
            return
        with self._route_sequence_lock:
            if self._route_sequence_generation != generation:
                return
            transport = self._route_transport
        runtime = self.state.snapshot()["runtime"]
        route = runtime.get("route") or []
        if not route:
            self._route_sequence_failed(generation, "route_error", "Repeat route disappeared.")
            return
        cycle = int(runtime.get("routeRepeatCycle") or 1) + 1
        self.state.log_run_event(
            "route_repeat_resumed",
            cycle=cycle,
            transport=transport,
        )
        self.state.update_runtime(
            {
                "goal": route[0],
                "routeIndex": 0,
                "routeRepeatCycle": cycle,
                "routeRepeatResumeAt": None,
                "navStatus": "sending_route",
                "navMessage": f"Starting repeat cycle {cycle}: 1/{len(route)}.",
            }
        )
        if transport == "navigate_to_pose_sequence":
            self._dispatch_route_pose(generation, 0)
            return
        if transport == "navigate_through_poses":
            self._dispatch_route_action(generation)
            return
        self._route_sequence_failed(
            generation,
            "route_error",
            f"Repeat transport is unavailable: {transport or 'none'}.",
        )

    def _dispatch_route_action(self, generation: int) -> None:
        if not self._route_sequence_is_current(generation):
            return
        route = self.state.snapshot()["runtime"].get("route") or []
        try:
            goal_msg = self.NavigateThroughPoses.Goal()
            goal_msg.poses = [
                self._pose_stamped(
                    float(pose["x"]),
                    float(pose["y"]),
                    float(pose.get("yaw", 0.0)),
                )
                for pose in route
            ]
            future = self.route_action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback, g=generation: (
                    self._on_route_feedback(feedback, g)
                ),
            )
            future.add_done_callback(
                lambda done, g=generation: self._on_route_goal_response(done, g)
            )
        except Exception as exc:
            self._route_sequence_failed(
                generation,
                "route_error",
                f"Could not send NavigateThroughPoses route: {exc}",
            )

    def _prepare_autonomous_navigation(self) -> None:
        self._cancel_route_sequence()
        self._clear_manual_watchdog()
        self._stop_lidar_fallback(publish_stop=True)
        self._cancel_navigation_handles()
        try:
            self._publish_cmd_vel(0.0, 0.0)
        except Exception:
            pass

    def send_goal(
        self,
        x: float,
        y: float,
        yaw: float,
        clear_route: bool = True,
        path: Optional[list[Dict[str, Any]]] = None,
        force_fallback: bool = False,
    ) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.send_goal(
                x,
                y,
                yaw,
                clear_route=clear_route,
                path=path,
                force_fallback=force_fallback,
            )
        self.state.log_run_event(
            "goal_requested",
            goal={"x": x, "y": y, "yaw": yaw},
            pathPointCount=len(path or []),
            clearRoute=clear_route,
            forceFallback=force_fallback,
        )
        if clear_route:
            self._prepare_autonomous_navigation()
        with self._route_sequence_lock:
            goal_generation = self._route_sequence_generation

        patch = {
            "goal": {"x": x, "y": y, "yaw": yaw, "stamp": now_iso()},
            "navStatus": "sending_goal",
            "navMessage": "Sending goal.",
            "lastCommandAt": now_iso(),
        }
        if clear_route:
            patch.update({"route": [], "routeIndex": 0})
        self.state.update_runtime(patch)

        if force_fallback:
            self._retire_route_sequence(goal_generation)
            fallback_result = self._start_lidar_fallback(
                path or [], [{"x": x, "y": y, "yaw": yaw}], queue_if_stale=True
            )
            if not fallback_result.get("ok"):
                self.state.update_runtime(
                    {
                        "navStatus": "fallback_unavailable",
                        "navMessage": (
                            "Forced LiDAR A* tracking could not start: "
                            f"{fallback_result.get('error') or 'unknown reason'}"
                        ),
                    }
                )
            return {**fallback_result, "forced": True}

        pose = self._pose_stamped(x, y, yaw)

        if not self._action_available(0.25):
            self._retire_route_sequence(goal_generation)
            fallback_result = self._start_lidar_fallback(
                path or [], [{"x": x, "y": y, "yaw": yaw}], queue_if_stale=True
            )
            if fallback_result.get("ok"):
                return fallback_result
            goal_subscribers = self._endpoint_count(self.topics["goalTopic"], "subscriptions")
            if goal_subscribers <= 0:
                self.state.update_runtime(
                    {
                        "navStatus": "goal_unavailable",
                        "navMessage": (
                            f"No Nav2 action server and LiDAR fallback was unavailable: "
                            f"{fallback_result.get('error') or 'unknown reason'}"
                        ),
                    }
                )
                return {
                    "ok": False,
                    "error": (
                        "Nav2 action server unavailable; "
                        f"LiDAR fallback failed: {fallback_result.get('error') or 'unknown reason'}"
                    ),
                    "transport": "none",
                }
            self.goal_pub.publish(pose)
            self.state.update_runtime(
                {
                    "navStatus": "goal_topic_published",
                    "navMessage": f"Action unavailable; published {self.topics['goalTopic']}.",
                }
            )
            return {"ok": True, "transport": "topic"}

        goal_msg = self.NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback, g=goal_generation: self._on_feedback(feedback, g),
        )
        future.add_done_callback(
            lambda done, g=goal_generation: self._on_goal_response(done, g)
        )
        return {"ok": True, "transport": "action"}

    def send_route(
        self,
        poses: list[Dict[str, float]],
        path: Optional[list[Dict[str, Any]]] = None,
        repeat_path: Optional[list[Dict[str, Any]]] = None,
        force_fallback: bool = False,
        repeat: Optional[Dict[str, Any]] = None,
        resume: bool = False,
    ) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.send_route(
                poses,
                path=path,
                repeat_path=repeat_path,
                force_fallback=force_fallback,
                repeat=repeat,
                resume=resume,
            )
        if not poses:
            return {"ok": False, "error": "Route is empty."}
        repeat_config = normalized_route_repeat(repeat, len(poses))
        self.state.log_run_event(
            "route_requested",
            poses=poses,
            pathPointCount=len(path or []),
            repeatPathPointCount=len(repeat_path or []),
            pathRouteIndices=sorted(
                {
                    int(point["routeIndex"])
                    for point in (path or [])
                    if isinstance(point.get("routeIndex"), int)
                }
            ),
            forceFallback=force_fallback,
            repeat=repeat_config,
        )
        self._prepare_autonomous_navigation()
        generation, repeat_config = self._begin_route_sequence(repeat_config, len(poses))
        route = [dict(pose, stamp=now_iso()) for pose in poses]
        self.state.begin_route_checkpoint(route, force_fallback, resume=resume)
        self.state.update_runtime(
            {
                "goal": route[0],
                "route": route,
                "routeIndex": 0,
                "navStatus": "sending_route",
                "navMessage": f"Sending route with {len(route)} poses.",
                "lastCommandAt": now_iso(),
                "routeRepeatEnabled": repeat_config["enabled"],
                "routeRepeatCycle": 1,
                "routeRepeatPauseSeconds": repeat_config["pauseSeconds"],
                "routeRepeatStart": repeat_config["sourceStart"],
                "routeRepeatEnd": repeat_config["sourceEnd"],
                "routeRepeatResumeAt": None,
            }
        )

        if force_fallback:
            self._retire_route_sequence(generation)
            fallback_result = self._start_lidar_fallback(
                path or [],
                route,
                queue_if_stale=True,
                repeat=repeat_config,
                repeat_path=repeat_path,
            )
            if not fallback_result.get("ok"):
                self.state.interrupt_route_checkpoint("fallback_unavailable")
                self.state.update_runtime(
                    {
                        "navStatus": "fallback_unavailable",
                        "navMessage": (
                            "Forced LiDAR A* route tracking could not start: "
                            f"{fallback_result.get('error') or 'unknown reason'}"
                        ),
                    }
                )
            return {**fallback_result, "forced": True}

        if self._action_available(0.25):
            self._set_route_transport(generation, "navigate_to_pose_sequence")
            self._dispatch_route_pose(generation, 0)
            self.state.update_runtime(
                {
                    "navStatus": "sending_goal",
                    "navMessage": f"Sequential route started: 1/{len(route)}.",
                }
            )
            return {
                "ok": True,
                "transport": "navigate_to_pose_sequence",
                "routeLength": len(route),
                "repeat": repeat_config,
            }

        if self._route_action_available(0.25):
            self._set_route_transport(generation, "navigate_through_poses")
            self._dispatch_route_action(generation)
            return {
                "ok": True,
                "transport": "navigate_through_poses",
                "routeLength": len(route),
                "repeat": repeat_config,
            }

        self._retire_route_sequence(generation)
        fallback_result = self._start_lidar_fallback(
            path or [],
            route,
            queue_if_stale=True,
            repeat=repeat_config,
            repeat_path=repeat_path,
        )
        if fallback_result.get("ok"):
            return fallback_result
        message = (
            f"Neither {self.topics['goalAction']} nor {self.topics['routeAction']} has a usable action server. "
            f"LiDAR fallback failed: {fallback_result.get('error') or 'unknown reason'}"
        )
        self.state.update_runtime(
            {
                "navStatus": "route_unavailable",
                "navMessage": message,
                "goal": None,
                "route": [],
                "routeIndex": 0,
            }
        )
        self.state.interrupt_route_checkpoint("route_unavailable")
        return {"ok": False, "error": message, "transport": "none"}

    def _on_route_goal_response(
        self, future: Any, generation: Optional[int] = None
    ) -> None:
        try:
            handle = future.result()
            if generation is not None and not self._route_sequence_is_current(generation):
                if getattr(handle, "accepted", False):
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                return
            self.route_goal_handle = handle
            if not handle.accepted:
                self.route_goal_handle = None
                if generation is not None:
                    self._route_sequence_failed(
                        generation,
                        "rejected",
                        "Nav2 rejected the route.",
                    )
                    return
                self.state.update_runtime(
                    {
                        "navStatus": "rejected",
                        "navMessage": "Nav2 rejected the route.",
                        "goal": None,
                        "route": [],
                        "routeIndex": 0,
                    }
                )
                return
            self.state.update_runtime(
                {"navStatus": "accepted", "navMessage": "Route accepted by Nav2."}
            )
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done, g=generation: self._on_route_result(done, g)
            )
        except Exception as exc:
            self.route_goal_handle = None
            if generation is not None:
                self._route_sequence_failed(generation, "route_error", str(exc))
                return
            self.state.update_runtime(
                {
                    "navStatus": "error",
                    "navMessage": str(exc),
                    "goal": None,
                    "route": [],
                    "routeIndex": 0,
                }
            )

    def _on_route_feedback(
        self, feedback_msg: Any, generation: Optional[int] = None
    ) -> None:
        if generation is not None and not self._route_sequence_is_current(generation):
            return
        feedback = feedback_msg.feedback
        snapshot = self.state.snapshot()
        route = snapshot["runtime"].get("route") or []
        remaining_poses = getattr(feedback, "number_of_poses_remaining", None)
        distance = getattr(feedback, "distance_remaining", None)
        patch: Dict[str, Any] = {"navStatus": "moving"}
        parts = []
        if remaining_poses is not None and route:
            remaining_count = max(0, int(remaining_poses))
            route_index = min(len(route) - 1, max(0, len(route) - remaining_count))
            patch["routeIndex"] = route_index
            patch["goal"] = route[route_index]
            parts.append(f"poses remaining: {remaining_count}")
        if distance is not None:
            parts.append(f"distance: {float(distance):.2f} m")
        patch["navMessage"] = "Route moving" + (f" ({', '.join(parts)})." if parts else ".")
        self.state.update_runtime(patch)

    def _on_route_result(
        self, future: Any, generation: Optional[int] = None
    ) -> None:
        clear_handle = generation is None
        try:
            if generation is not None and not self._route_sequence_is_current(generation):
                return
            clear_handle = True
            response = future.result()
            status = int(response.status)
            result = getattr(response, "result", None)
            error_code = int(getattr(result, "error_code", 0) or 0)
            error_message = str(getattr(result, "error_msg", "") or "").strip()
            if status == 4 and error_code == 0:
                if generation is not None and self._schedule_route_repeat(generation):
                    return
                nav_status = "succeeded"
                message = "Route complete."
            elif status == 5:
                nav_status = "canceled"
                message = "Route canceled."
            else:
                nav_status = "aborted"
                detail = error_message or f"status {status}, error {error_code}"
                message = f"Nav2 route failed: {detail}"
            if generation is not None and nav_status != "succeeded":
                self._route_sequence_failed(
                    generation,
                    nav_status,
                    message,
                    clear_goal=nav_status == "canceled",
                )
                return
            if generation is not None and not self._retire_route_sequence(generation):
                return
            self.state.update_runtime(
                {
                    "navStatus": nav_status,
                    "navMessage": message,
                    "route": [],
                    "routeIndex": 0,
                    "routeRepeatEnabled": False,
                    "routeRepeatResumeAt": None,
                    "goal": None if nav_status in ("succeeded", "canceled") else self.state.snapshot()["runtime"].get("goal"),
                }
            )
        except Exception as exc:
            if generation is not None:
                self._route_sequence_failed(generation, "route_error", str(exc))
                return
            self.state.update_runtime({"navStatus": "error", "navMessage": str(exc)})
        finally:
            if clear_handle:
                self.route_goal_handle = None

    def _on_goal_response(
        self, future: Any, generation: Optional[int] = None
    ) -> None:
        try:
            handle = future.result()
            if generation is not None and not self._route_sequence_is_current(generation):
                if getattr(handle, "accepted", False):
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                return
            self.goal_handle = handle
            if not handle.accepted:
                self.goal_handle = None
                if generation is not None:
                    self._retire_route_sequence(generation)
                self.state.update_runtime(
                    {
                        "navStatus": "rejected",
                        "navMessage": "Nav2 rejected the goal.",
                        "goal": None,
                        "route": [],
                        "routeIndex": 0,
                    }
                )
                return
            self.state.update_runtime({"navStatus": "accepted", "navMessage": "Goal accepted."})
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done, g=generation: self._on_result(done, g)
            )
        except Exception as exc:
            if generation is None or self._retire_route_sequence(generation):
                self.state.update_runtime({"navStatus": "error", "navMessage": str(exc)})

    def _on_feedback(
        self, feedback_msg: Any, generation: Optional[int] = None
    ) -> None:
        if generation is not None and not self._route_sequence_is_current(generation):
            return
        feedback = feedback_msg.feedback
        remaining = getattr(feedback, "distance_remaining", None)
        if remaining is None:
            self.state.update_runtime({"navStatus": "moving", "navMessage": "Moving."})
        else:
            self.state.update_runtime(
                {
                    "navStatus": "moving",
                    "navMessage": f"Distance remaining: {float(remaining):.2f} m",
                }
            )

    def _on_result(
        self, future: Any, generation: Optional[int] = None
    ) -> None:
        try:
            result = future.result()
            if generation is not None and not self._route_sequence_is_current(generation):
                return
            self.goal_handle = None
            status = int(result.status)
            if status == 4:
                snapshot = self.state.snapshot()
                route = snapshot["runtime"].get("route") or []
                route_index = int(snapshot["runtime"].get("routeIndex") or 0)
                if route and route_index < len(route) - 1:
                    next_index = route_index + 1
                    next_goal = route[next_index]
                    self.state.update_runtime(
                        {
                            "routeIndex": next_index,
                            "navStatus": "sending_goal",
                            "navMessage": f"Sending route waypoint {next_index + 1}/{len(route)}.",
                        }
                    )
                    self.send_goal(
                        float(next_goal["x"]),
                        float(next_goal["y"]),
                        float(next_goal.get("yaw", 0.0)),
                        clear_route=False,
                    )
                    return
                nav_status = "succeeded"
                message = "Route complete." if route else "Goal reached."
                route_patch = {"goal": None, "route": [], "routeIndex": 0}
            elif status == 5:
                nav_status = "canceled"
                message = "Goal canceled."
                route_patch = {"goal": None, "route": [], "routeIndex": 0}
            else:
                nav_status = "aborted" if status == 6 else "finished"
                message = f"Nav2 goal failed with status: {status}"
                route_patch = {"route": [], "routeIndex": 0}
            if generation is not None and not self._retire_route_sequence(generation):
                return
            self.state.update_runtime({"navStatus": nav_status, "navMessage": message, **route_patch})
        except Exception as exc:
            if generation is not None and not self._route_sequence_is_current(generation):
                return
            self.goal_handle = None
            if generation is not None:
                self._retire_route_sequence(generation)
            self.state.update_runtime({"navStatus": "error", "navMessage": str(exc)})

    def _cancel_navigation_handles(self) -> None:
        for attribute in ("goal_handle", "route_goal_handle"):
            handle = getattr(self, attribute, None)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
                setattr(self, attribute, None)

    def cancel_goal(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.cancel_goal()
        self.state.log_run_event("cancel_requested")
        self._cancel_route_sequence()
        self._stop_lidar_fallback(publish_stop=True)
        self._cancel_navigation_handles()
        self.state.update_runtime(
            {
                "navStatus": "cancel_requested",
                "navMessage": "Cancel requested.",
                "goal": None,
                "route": [],
                "routeIndex": 0,
            }
        )
        return {"ok": True}

    def stop_robot(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.stop_robot()
        self.state.log_run_event(
            "stop_requested",
            topic=self.topics["cmdVel"],
            messageType=self.cmd_vel_msg_type,
        )
        self._clear_manual_watchdog()
        self._publish_cmd_vel(0.0, 0.0, repeats=12, interval=0.04)
        self.cancel_goal()
        self.state.update_runtime(
            {
                "navStatus": "stopped",
                "navMessage": f"Stop command published to {self.topics['cmdVel']} ({self._cmd_vel_type_label()}).",
                "cmdVelMessageType": self.cmd_vel_msg_type,
            }
        )
        return {"ok": True, "topic": self.topics["cmdVel"], "messageType": self.cmd_vel_msg_type}

    def manual_drive(self, linear: float, angular: float) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.manual_drive(linear, angular)
        linear = max(-0.22, min(0.22, finite_float(linear, "linear")))
        angular = max(-2.84, min(2.84, finite_float(angular, "angular")))
        command = (round(linear, 3), round(angular, 3))
        if command != self._last_logged_manual_command:
            self._last_logged_manual_command = command
            self.state.log_run_event(
                "manual_command",
                linear=linear,
                angular=angular,
                topic=self.topics["cmdVel"],
                messageType=self.cmd_vel_msg_type,
            )
        self._cancel_route_sequence()
        self._stop_lidar_fallback(publish_stop=True)
        self._cancel_navigation_handles()
        active_command = abs(linear) > 0 or abs(angular) > 0
        if active_command:
            self._arm_manual_watchdog(linear, angular)
        else:
            self._clear_manual_watchdog()
        stop_repeats = 1 if active_command else 12
        self._publish_cmd_vel(linear, angular, repeats=stop_repeats, interval=0.04)
        message_action = "Manual target" if active_command else "Manual stop"
        clock_status = self._cmd_vel_clock_status()
        self.state.update_runtime(
            {
                "goal": None,
                "route": [],
                "routeIndex": 0,
                "navStatus": "manual" if active_command else "manual_stop",
                "navMessage": f"{message_action} {self.topics['cmdVel']} ({self._cmd_vel_type_label()}): v={linear:.2f}, w={angular:.2f}",
                "cmdVelMessageType": self.cmd_vel_msg_type,
                "lastManualCommandAt": now_iso(),
                "lastCommandAt": now_iso(),
                **clock_status,
            }
        )
        return {
            "ok": True,
            "linear": linear,
            "angular": angular,
            "topic": self.topics["cmdVel"],
            "messageType": self.cmd_vel_msg_type,
            "clock": clock_status,
        }

    def connection_status(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.connection_status()
        self._ensure_cmd_vel_publisher_type()
        snapshot = self.state.snapshot()
        connection = base_connection_info(
            self._action_available(0.05),
            self._route_action_available(0.05),
            snapshot["setup"].get("network", {}),
        )
        self.state.update_runtime({"connection": connection})
        snapshot = self.state.snapshot()
        return {
            "ok": True,
            "mode": "ros2",
            "runtime": snapshot["runtime"],
            "connection": connection,
            "topics": snapshot["setup"]["topics"],
            "network": public_network(snapshot["setup"].get("network", {})),
        }

    def robot_check(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.robot_check()
        self._ensure_cmd_vel_publisher_type()
        self.state.update_runtime({"cmdVelMessageType": self.cmd_vel_msg_type})

        snapshot = self.state.snapshot()
        setup = snapshot["setup"]
        runtime = snapshot["runtime"]
        topics = setup["topics"]
        network = setup.get("network", {})
        topics_and_types = self.node.get_topic_names_and_types()
        topic_names = sorted(name for name, _types in topics_and_types)
        topic_type_map = {name: list(types) for name, types in topics_and_types}
        topic_set = set(topic_names)
        raw_nodes = self.node.get_node_names()
        nodes = sorted(set(raw_nodes))
        dashboard_node_count = sum(1 for name in raw_nodes if name == "turtlebot_web_dashboard")
        camera_node_keywords = ("camera", "realsense", "usb_cam", "v4l2", "picamera", "raspicam", "image_proc")
        camera_nodes = sorted(
            node for node in nodes if any(keyword in node.lower() for keyword in camera_node_keywords)
        )
        actions = self._action_names()
        action_available = self._action_available(0.2)
        route_action_available = self._route_action_available(0.2)
        connection = base_connection_info(action_available, route_action_available, network)
        candidate = self._score_turtlebot_candidate(topic_names, dict(topics_and_types), nodes, actions)
        recommended_topics = candidate.get("recommendedTopics", {}) if candidate.get("score", 0) >= 50 else {}
        context_domain = str(connection.get("rosDomainId") or "")
        env_domain = os.environ.get("ROS_DOMAIN_ID", "")
        expected_domain = str(network.get("rosDomainId") or "")
        env_localhost = os.environ.get("ROS_LOCALHOST_ONLY", "")
        expected_server_ip = str(network.get("serverIp") or "")

        checks: list[Dict[str, Any]] = []

        def add_check(id_: str, label: str, ok: bool, detail: str, level: Optional[str] = None) -> None:
            checks.append(
                {
                    "id": id_,
                    "label": label,
                    "ok": ok,
                    "level": level or ("ok" if ok else "fail"),
                    "detail": detail,
                }
            )

        add_check("ros_bridge", "ROS2 브릿지", True, "server.py가 ROS2 rclpy 모드로 실행 중입니다.")
        add_check(
            "dashboard_node",
            "Dashboard ROS node",
            dashboard_node_count <= 1,
            f"turtlebot_web_dashboard count {dashboard_node_count}",
            "warn" if dashboard_node_count > 1 else None,
        )
        add_check(
            "domain",
            "ROS_DOMAIN_ID",
            bool(context_domain) and (not expected_domain or context_domain == expected_domain),
            f"브릿지 Context {context_domain or '-'} / 설정값 {expected_domain or '-'} / 서버 환경 {env_domain or '-'}",
        )
        add_check(
            "localhost",
            "ROS_LOCALHOST_ONLY",
            env_localhost in ("", "0", "false", "False"),
            f"환경값 {env_localhost or '-'}; 로봇과 통신하려면 0이어야 합니다.",
        )
        if expected_server_ip:
            add_check(
                "server_ip",
                "서버 IP",
                expected_server_ip in connection.get("serverIps", []),
                f"설정값 {expected_server_ip}; 감지 IP {', '.join(connection.get('serverIps', [])) or '-'}",
            )
        else:
            add_check("server_ip", "서버 IP", False, "설정된 서버 IP가 없습니다.", "warn")
        add_check(
            "ssh_config",
            "Robot SSH",
            bool(network.get("robotSshHost") and network.get("robotSshUser")),
            f"{network.get('robotSshUser') or '-'}@{network.get('robotSshHost') or '-'}",
            "warn" if not (network.get("robotSshHost") and network.get("robotSshUser")) else None,
        )
        scan_topic = topics.get("scan", "/scan")
        odom_topic = topics.get("odom", "/odom")
        pose_topic = topics.get("pose", "/amcl_pose")
        cmd_topic = topics.get("cmdVel", "/cmd_vel")
        camera_topic = topics.get("camera", "")
        compressed_camera_topic = topics.get("compressedCamera", "")
        camera_enabled = bool(runtime.get("cameraEnabled", True))
        action_name = topics.get("goalAction", "/navigate_to_pose")
        route_action_name = topics.get("routeAction", "/navigate_through_poses")
        initial_pose_topic = topics.get("initialPose", "/initialpose")
        namespace_mismatches = []
        for key, configured in (
            ("scan", scan_topic),
            ("odom", odom_topic),
            ("cmdVel", cmd_topic),
            ("pose", pose_topic),
            ("goalAction", action_name),
            ("routeAction", route_action_name),
        ):
            recommended = recommended_topics.get(key)
            if recommended and configured and recommended != configured:
                namespace_mismatches.append(f"{key}: {configured} -> {recommended}")
        if namespace_mismatches:
            add_check(
                "topic_namespace",
                "Topic namespace",
                False,
                "; ".join(namespace_mismatches),
            )

        scan_publishers = self._endpoint_count(scan_topic, "publishers")
        scan_subscribers = self._endpoint_count(scan_topic, "subscriptions")
        odom_publishers = self._endpoint_count(odom_topic, "publishers")
        odom_subscribers = self._endpoint_count(odom_topic, "subscriptions")
        pose_publishers = self._endpoint_count(pose_topic, "publishers")
        pose_subscribers = self._endpoint_count(pose_topic, "subscriptions")
        initial_pose_subscribers = self._endpoint_count(initial_pose_topic, "subscriptions")
        cmd_publishers = self._endpoint_count(cmd_topic, "publishers")
        cmd_subscribers = self._endpoint_count(cmd_topic, "subscriptions")
        cmd_publisher_types = self._endpoint_types(cmd_topic, "publishers")
        cmd_subscriber_types = self._endpoint_types(cmd_topic, "subscriptions")
        cmd_message_type = getattr(self, "cmd_vel_msg_type", self._detect_cmd_vel_message_type(cmd_topic))
        cmd_foreign_publisher_types = [
            msg_type for msg_type in cmd_publisher_types if cmd_subscriber_types and msg_type not in cmd_subscriber_types
        ]
        cmd_type_matches = not cmd_subscriber_types or cmd_message_type in cmd_subscriber_types
        cmd_clock_status = self._cmd_vel_clock_status()
        cmd_clock_ready = (
            cmd_message_type != TWIST_STAMPED_TYPE
            or cmd_clock_status.get("cmdVelStampSource") == "robot_odom"
        )
        camera_publishers = self._endpoint_count(camera_topic, "publishers") if camera_topic else 0
        compressed_camera_publishers = (
            self._endpoint_count(compressed_camera_topic, "publishers") if compressed_camera_topic else 0
        )
        camera_topic_candidates = [
            name
            for name, types in topic_type_map.items()
            if any(msg_type in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage") for msg_type in types)
            and self._endpoint_count(name, "publishers") > 0
        ]
        camera_bringup_ok = (not camera_enabled) or camera_publishers > 0 or compressed_camera_publishers > 0
        scan_last_seen = bool(runtime.get("lastScanAt"))
        nav2_expected_nodes = (
            "map_server",
            "amcl",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "behavior_server",
            "collision_monitor",
        )
        nav2_present_nodes = [node for node in nav2_expected_nodes if node in nodes]

        add_check(
            "scan",
            "LiDAR scan",
            scan_topic in topic_set and scan_publishers > 0,
            f"{scan_topic}; exists {scan_topic in topic_set}; publishers {scan_publishers}; subscribers {scan_subscribers}; last {runtime.get('lastScanAt') or '-'}",
            "warn" if scan_topic in topic_set and scan_publishers > 0 and not scan_last_seen else None,
        )
        add_check(
            "odom",
            "Odometry",
            odom_topic in topic_set and odom_publishers > 0,
            f"{odom_topic}; exists {odom_topic in topic_set}; publishers {odom_publishers}; subscribers {odom_subscribers}; last {runtime.get('lastOdomAt') or '-'}",
        )
        add_check(
            "cmd_vel",
            "수동운전 cmd_vel",
            cmd_subscribers > 0,
            f"{cmd_topic}; exists {cmd_topic in topic_set}; publishers {cmd_publishers}; subscribers {cmd_subscribers}. subscribers 0이면 로봇 드라이버가 명령을 못 받습니다.",
        )
        add_check(
            "cmd_vel_type",
            "cmd_vel message type",
            cmd_type_matches,
            f"dashboard {cmd_message_type}; publisher types {', '.join(cmd_publisher_types) or '-'}; subscriber types {', '.join(cmd_subscriber_types) or '-'}",
            "warn" if cmd_type_matches and cmd_foreign_publisher_types else None,
        )
        add_check(
            "cmd_vel_clock",
            "cmd_vel timestamp clock",
            cmd_clock_ready,
            (
                f"source {cmd_clock_status.get('cmdVelStampSource') or '-'}; "
                f"odom clock age {cmd_clock_status.get('cmdVelClockAgeMs')}; "
                f"robot-server skew {cmd_clock_status.get('cmdVelClockSkewMs')} ms"
            ),
            "warn" if not cmd_clock_ready else None,
        )
        fallback_settings = normalized_fallback_settings(setup)
        now_monotonic = time.monotonic()
        with self._scan_lock:
            fallback_scan_received = float(self._latest_scan.get("received") or 0.0)
        fallback_scan_age = (
            now_monotonic - fallback_scan_received if fallback_scan_received > 0 else math.inf
        )
        fallback_odom_age = (
            now_monotonic - self._last_odom_monotonic
            if self._last_odom_monotonic > 0
            else math.inf
        )
        fallback_pose_age = (
            now_monotonic - self._last_pose_monotonic
            if self._last_pose_monotonic > 0
            else math.inf
        )
        fallback_ready = (
            fallback_settings["enabled"]
            and scan_publishers > 0
            and odom_publishers > 0
            and cmd_subscribers > 0
            and cmd_publishers <= 1
            and fallback_scan_age <= fallback_settings["scanTimeout"]
            and fallback_odom_age <= fallback_settings["odomTimeout"]
            and fallback_pose_age <= fallback_settings["odomTimeout"]
        )
        add_check(
            "lidar_fallback",
            "LiDAR 비상주행",
            fallback_ready,
            (
                f"enabled {fallback_settings['enabled']}; scan age "
                f"{fallback_scan_age * 1000:.0f} ms; odom age {fallback_odom_age * 1000:.0f} ms; "
                f"pose age {fallback_pose_age * 1000:.0f} ms; cmd_vel publishers {cmd_publishers}, "
                f"subscribers {cmd_subscribers}"
            ),
            None if fallback_ready else "warn",
        )
        duplicate_sources_ok = scan_publishers <= 1 and odom_publishers <= 1
        add_check(
            "duplicate_base_sources",
            "중복 base bringup",
            duplicate_sources_ok,
            f"{scan_topic} publishers {scan_publishers}; {odom_topic} publishers {odom_publishers}",
            "warn" if not duplicate_sources_ok else None,
        )
        add_check(
            "tf",
            "TF",
            "/tf" in topic_set and "/tf_static" in topic_set,
            f"/tf {'있음' if '/tf' in topic_set else '없음'}, /tf_static {'있음' if '/tf_static' in topic_set else '없음'}",
        )
        add_check(
            "amcl",
            "AMCL 위치",
            pose_topic in topic_set and pose_publishers > 0 and bool(runtime.get("lastPoseAt") or runtime.get("pose", {}).get("stamp")),
            f"{pose_topic}; exists {pose_topic in topic_set}; publishers {pose_publishers}; subscribers {pose_subscribers}; last {runtime.get('lastPoseAt') or runtime.get('pose', {}).get('stamp') or '-'}",
            "warn" if pose_topic not in topic_set or pose_publishers == 0 else None,
        )
        add_check(
            "initialpose_receiver",
            "Initialpose receiver",
            initial_pose_subscribers > 0,
            f"{initial_pose_topic}; subscribers {initial_pose_subscribers}. 0이면 AMCL/Nav2가 초기 위치를 받지 않습니다.",
            "warn" if initial_pose_subscribers <= 0 else None,
        )
        add_check(
            "nav2_nodes",
            "Nav2 lifecycle nodes",
            len(nav2_present_nodes) >= 4,
            (
                f"present {', '.join(nav2_present_nodes) or '-'}; expected {', '.join(nav2_expected_nodes)}. "
                "A* direct driving remains available without Nav2."
            ),
            "warn",
        )
        add_check(
            "nav2",
            "Nav2 NavigateToPose",
            action_available,
            (
                f"{action_name}; action server {'available' if action_available else 'not available'}; "
                f"actions {', '.join(actions) or '-'}. A* direct mode does not require it."
            ),
            "warn" if not action_available else None,
        )
        add_check(
            "nav2_route",
            "Nav2 NavigateThroughPoses",
            route_action_available,
            (
                f"{route_action_name}; action server {'available' if route_action_available else 'not available'}; "
                f"actions {', '.join(actions) or '-'}. A* direct mode does not require it."
            ),
            "warn" if not route_action_available else None,
        )
        add_check(
            "camera_bringup",
            "Camera bringup",
            camera_bringup_ok,
            "dashboard camera off"
            if not camera_enabled
            else (
                f"nodes {', '.join(camera_nodes) or '-'}; "
                f"raw {camera_topic or '-'} publishers {camera_publishers}; "
                f"compressed {compressed_camera_topic or '-'} publishers {compressed_camera_publishers}"
            ),
        )
        add_check(
            "camera",
            "카메라",
            (not camera_enabled) or camera_publishers > 0 or compressed_camera_publishers > 0,
            "dashboard camera off"
            if not camera_enabled
            else f"raw publishers {camera_publishers}, compressed publishers {compressed_camera_publishers}; last {runtime.get('lastCameraAt') or '-'}",
            None if not camera_enabled else "warn" if camera_publishers == 0 and compressed_camera_publishers == 0 else None,
        )

        advice = []
        failed = {check["id"] for check in checks if check["level"] == "fail"}
        warned = {check["id"] for check in checks if check["level"] == "warn"}
        if "domain" in failed:
            advice.append(
                f"서버와 로봇 모두 export ROS_DOMAIN_ID={expected_domain or '1'} 로 맞춘 뒤 실행하세요."
            )
        if "localhost" in failed:
            advice.append("서버와 로봇 모두 export ROS_LOCALHOST_ONLY=0 으로 설정하세요.")
        if "server_ip" in failed:
            advice.append(f"서버 PC가 설정 IP {expected_server_ip or '-'} 네트워크 인터페이스로 실행 중인지 확인하세요.")
        if "topic_namespace" in failed:
            advice.append(f"ROS graph recommended topics: {json.dumps(recommended_topics, ensure_ascii=False)}")
        if "cmd_vel" in failed:
            if cmd_topic in topic_set and cmd_publishers > 0 and cmd_subscribers == 0:
                advice.append("cmd_vel 토픽은 있고 publisher도 있지만 subscriber가 없습니다. 즉 명령을 보내는 쪽은 보이지만 robot driver/diff_drive_controller/OpenCR 쪽이 이 토픽을 듣지 않는 상태입니다.")
            advice.append(f"수동운전이 안 되면 robot.launch.py와 OpenCR/모터 드라이버, {cmd_topic} 구독자를 먼저 확인하세요.")
        if "cmd_vel_type" in failed:
            advice.append(f"{cmd_topic} message type mismatch: dashboard {cmd_message_type}, robot subscribers {', '.join(cmd_subscriber_types) or '-'}.")
        if "cmd_vel_type" in warned:
            advice.append(f"{cmd_topic} has multiple publisher message types. Stop old server.py/teleop processes and run one dashboard instance.")
        if "cmd_vel_clock" in warned:
            advice.append(
                f"{cmd_topic} is TwistStamped but a fresh robot clock was not learned from {odom_topic}. "
                "Wait for odom reception and keep NTP enabled on both machines before manual driving."
            )
        if "duplicate_base_sources" in warned:
            advice.append(
                "scan/odom publisher가 여러 개입니다. 중복 robot.launch.py를 종료하고 기본 bringup 한 세트만 남기세요."
            )
        if "lidar_fallback" in warned and fallback_settings["enabled"]:
            advice.append(
                "LiDAR 비상주행은 최신 scan/odom/pose, cmd_vel subscriber, 단일 cmd_vel publisher가 "
                "모두 확인될 때만 시작합니다."
            )
        if "dashboard_node" in warned:
            advice.append("Multiple turtlebot_web_dashboard ROS nodes are visible. Stop old server.py processes and keep one dashboard server running.")
        if "camera" in warned and camera_enabled and camera_publishers == 0 and compressed_camera_publishers == 0:
            if camera_topic_candidates:
                advice.append(
                    f"Configured camera topics have no publishers. Detected camera publishers: {', '.join(camera_topic_candidates)}. Apply one of these topics in setup."
                )
            else:
                advice.append(
                    f"Camera topics have no publishers. The dashboard is subscribed to {camera_topic} / {compressed_camera_topic}, but no camera driver node is publishing frames."
                )
        if "camera_bringup" in failed:
            advice.append(
                f"Start a camera driver node and verify it publishes {camera_topic} or {compressed_camera_topic}. Visible camera nodes: {', '.join(camera_nodes) or '-'}."
            )
        if "nav2" in failed:
            advice.append("목표 이동은 turtlebot3_navigation2와 /navigate_to_pose action server가 필요합니다. action list에 이름만 보여도 action servers 0이면 Nav2가 안 떠 있는 상태입니다.")
        if "nav2_route" in failed:
            advice.append(
                f"경유지 이동은 Jazzy Nav2의 {route_action_name} action server가 필요합니다. "
                "bt_navigator가 active인지 확인하세요."
            )
        if "nav2_nodes" in failed:
            advice.append("현재 graph에는 기본 bringup 노드만 보입니다. map_server/amcl/controller_server/planner_server/bt_navigator 등 Nav2 lifecycle 노드를 올려야 목표 이동이 됩니다.")
        if "initialpose_receiver" in warned:
            advice.append(f"{initial_pose_topic} subscribers가 0입니다. 지금 초기 위치 지정은 AMCL이 아니라 대시보드 odom 표시 보정에만 적용됩니다.")
        if "amcl" in failed or "amcl" in warned:
            advice.append("초기 위치 적용 후 /amcl_pose가 나오는지 확인하세요.")
        if "scan" in failed:
            advice.append(f"LiDAR bringup과 {scan_topic} 발행 상태를 확인하세요.")
        if "scan" in warned:
            advice.append(f"{scan_topic} publisher는 있지만 대시보드가 최근 scan을 받지 못했습니다. v2026-07-09.11부터 sensor_data QoS를 사용하므로 서버를 재시작해 확인하세요.")

        overall_ok = all(check["level"] != "fail" for check in checks)
        summary = "로봇 체크 통과" if overall_ok else "로봇 체크에서 문제가 발견됐습니다."
        self.state.update_runtime({"connection": connection})
        return {
            "ok": overall_ok,
            "mode": "ros2",
            "summary": summary,
            "checks": checks,
            "advice": advice,
            "runtime": runtime,
            "connection": connection,
            "network": public_network(network),
            "topics": topics,
            "recommendedTopics": recommended_topics,
            "graphTopics": topic_names,
            "nodes": nodes,
            "cameraNodes": camera_nodes,
            "actions": actions,
        }

    def diagnostics_report(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.diagnostics_report()
        check = self.robot_check()
        snapshot = self.state.snapshot()
        setup = snapshot.get("setup", {})
        command_outputs = collect_diagnostic_commands(
            setup.get("topics", {}),
            setup.get("robotProfiles", {}),
        )
        robot_ssh_output = self.robot_ssh_check(detailed=True)
        return {
            "ok": True,
            "generatedAt": now_iso(),
            "report": format_diagnostics_report(
                snapshot,
                check,
                graph_topics=check.get("graphTopics", []),
                nodes=check.get("nodes", []),
                actions=check.get("actions", []),
                command_outputs=command_outputs,
                robot_ssh_output=robot_ssh_output,
            ),
            "commands": command_outputs,
            "robotSsh": robot_ssh_output,
        }

    def robot_bringup(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.robot_bringup()
        result = run_robot_bringup_from_state(self.state)
        time.sleep(2.0)
        result["check"] = self.robot_check()
        return result

    def robot_ssh_check(self, detailed: bool = False) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.robot_ssh_check(detailed=detailed)
        return run_robot_ssh_check_from_state(self.state, detailed=detailed)

    def manual_drive_check(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.manual_drive_check()
        setup = self.state.get_setup()
        network = setup.get("network", {})
        topic = str(self.topics.get("cmdVel") or "/cmd_vel")
        self._ensure_cmd_vel_publisher_type()
        message_type = self.cmd_vel_msg_type
        script = build_cmd_vel_delivery_check_script(network, topic, message_type)
        result_holder: Dict[str, Any] = {}

        def run_remote_echo() -> None:
            result_holder["result"] = run_ssh_script(network, script, timeout=18.0)

        self._clear_manual_watchdog()
        self._stop_lidar_fallback(publish_stop=True)
        self._cancel_navigation_handles()
        thread = threading.Thread(target=run_remote_echo, daemon=True)
        thread.start()
        deadline = time.monotonic() + 12.0
        publish_error = ""
        while thread.is_alive() and time.monotonic() < deadline:
            try:
                self._publish_cmd_vel(0.0, 0.0)
            except Exception as exc:
                publish_error = str(exc)
                break
            time.sleep(0.2)
        thread.join(timeout=7.0)
        result = result_holder.get("result")
        if result is None:
            result = {
                "ok": False,
                "command": "ssh cmd_vel delivery check",
                "returncode": None,
                "stdout": "",
                "stderr": "cmd_vel delivery check did not finish within the timeout.",
            }
        stdout = str(result.get("stdout") or "")
        delivered = bool(result.get("ok")) and "DASHBOARD_CMD_VEL_ECHO_RC=0" in stdout
        clock_status = self._cmd_vel_clock_status()
        checked_at = now_iso()
        self.state.update_runtime(
            {
                "lastCmdVelDeliveryCheckAt": checked_at,
                "lastCmdVelDeliveryCheckOk": delivered,
                "navStatus": "cmd_vel_check_ok" if delivered else "cmd_vel_check_failed",
                "navMessage": (
                    f"Robot received a zero cmd_vel on {topic}."
                    if delivered
                    else f"Robot did not confirm the dashboard zero cmd_vel on {topic}."
                ),
                **clock_status,
            }
        )
        return {
            "ok": delivered,
            "checkedAt": checked_at,
            "topic": topic,
            "messageType": message_type,
            "clock": clock_status,
            "command": result.get("command"),
            "returncode": result.get("returncode"),
            "stdout": stdout,
            "stderr": "\n".join(part for part in (publish_error, str(result.get("stderr") or "")) if part),
        }

    def shutdown(self) -> None:
        self._clear_manual_watchdog()
        self._cancel_route_sequence(update_runtime=False)
        self._stop_lidar_fallback(publish_stop=False)
        self._shutdown_event.set()
        if self._is_fallback():
            self._fallback.shutdown()
            return
        try:
            self._publish_cmd_vel(0.0, 0.0, repeats=12, interval=0.04)
        except Exception:
            pass
        self._cancel_navigation_handles()
        if self._manual_watchdog_thread and self._manual_watchdog_thread.is_alive():
            self._manual_watchdog_thread.join(timeout=1.5)
        if self._fallback_nav_thread and self._fallback_nav_thread.is_alive():
            self._fallback_nav_thread.join(timeout=1.5)
        shutdown_ros_executor(self.ros_executor, self.node)
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.5)
        with self._publisher_lock:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            except Exception:
                pass
        try:
            if self.ros_context is not None:
                self.ros_context.try_shutdown()
        except Exception:
            pass
        self.ros_executor = None

    def _cmd_vel_type_label(self) -> str:
        return getattr(self, "cmd_vel_msg_type", "geometry_msgs/msg/Twist").rsplit("/", 1)[-1]

    def _publish_cmd_vel(self, linear: float, angular: float, repeats: int = 1, interval: float = 0.0) -> None:
        with self._publisher_lock:
            self._ensure_cmd_vel_publisher_type()
            repeats = max(1, int(repeats))
            for index in range(repeats):
                self.cmd_vel_pub.publish(self._make_cmd_vel_message(float(linear), float(angular)))
                if interval > 0 and index < repeats - 1:
                    time.sleep(interval)

    def _start_manual_watchdog(self) -> None:
        if self._manual_watchdog_thread and self._manual_watchdog_thread.is_alive():
            return
        self._manual_watchdog_thread = threading.Thread(target=self._manual_watchdog_loop, daemon=True)
        self._manual_watchdog_thread.start()

    def _arm_manual_watchdog(self, linear: float, angular: float, timeout: float = 1.2) -> None:
        with self._manual_lock:
            self._manual_target_linear = float(linear)
            self._manual_target_angular = float(angular)
            self._manual_active_until = time.monotonic() + timeout

    def _clear_manual_watchdog(self) -> None:
        with self._manual_lock:
            self._manual_active_until = 0.0
            self._manual_target_linear = 0.0
            self._manual_target_angular = 0.0

    def _manual_watchdog_loop(self) -> None:
        while not self._shutdown_event.wait(0.1):
            publish_target = False
            expired = False
            linear = 0.0
            angular = 0.0
            with self._manual_lock:
                deadline = self._manual_active_until
                now = time.monotonic()
                active = deadline > 0 and now <= deadline
                expired = deadline > 0 and now > deadline
                if active:
                    linear = self._manual_target_linear
                    angular = self._manual_target_angular
                    publish_target = abs(linear) > 0 or abs(angular) > 0
                if expired:
                    self._manual_active_until = 0.0
                    self._manual_target_linear = 0.0
                    self._manual_target_angular = 0.0
            if self._is_fallback() or self.node is None:
                continue
            try:
                if publish_target:
                    self._publish_cmd_vel(linear, angular)
                elif expired:
                    self._publish_cmd_vel(0.0, 0.0, repeats=12, interval=0.04)
                    self.state.update_runtime(
                        {
                            "navStatus": "manual_timeout",
                            "navMessage": f"Manual watchdog stop on {self.topics['cmdVel']} ({self._cmd_vel_type_label()}).",
                            "cmdVelMessageType": self.cmd_vel_msg_type,
                            "lastCommandAt": now_iso(),
                        }
                    )
            except Exception as exc:
                self._clear_manual_watchdog()
                self.state.update_runtime(
                    {"navStatus": "manual_error", "navMessage": f"cmd_vel publish failed: {exc}"}
                )

    def _make_cmd_vel_message(self, linear: float, angular: float) -> Any:
        if getattr(self, "cmd_vel_msg_type", "") == TWIST_STAMPED_TYPE:
            msg = self.TwistStamped()
            try:
                msg.header.stamp, _source = self._cmd_vel_stamp()
            except Exception:
                pass
            msg.header.frame_id = self.topics.get("baseFrame") or "base_link"
            msg.twist.linear.x = linear
            msg.twist.angular.z = angular
            return msg
        msg = self.Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    def _ensure_cmd_vel_publisher_type(self) -> None:
        with self._publisher_lock:
            if self.node is None:
                raise RuntimeError("ROS2 node is not available.")
            detected = self._detect_cmd_vel_message_type(self.topics["cmdVel"])
            if detected == getattr(self, "cmd_vel_msg_type", None):
                return
            try:
                self.node.destroy_publisher(self.cmd_vel_pub)
            except Exception:
                pass
            self.cmd_vel_msg_type = detected
            msg_cls = self.TwistStamped if detected == TWIST_STAMPED_TYPE else self.Twist
            self.cmd_vel_pub = self.node.create_publisher(msg_cls, self.topics["cmdVel"], 10)
            self.state.update_runtime(
                {
                    "cmdVelMessageType": self.cmd_vel_msg_type,
                    "navMessage": f"cmd_vel publisher switched to {self.topics['cmdVel']} ({self._cmd_vel_type_label()}).",
                    **self._cmd_vel_clock_status(),
                }
            )

    def _detect_cmd_vel_message_type(self, topic: str) -> str:
        subscription_types = set(self._endpoint_types(topic, "subscriptions"))
        if TWIST_STAMPED_TYPE in subscription_types:
            return TWIST_STAMPED_TYPE
        if TWIST_TYPE in subscription_types:
            return TWIST_TYPE

        try:
            topic_types = set(dict(self.node.get_topic_names_and_types()).get(topic, []))
        except Exception:
            topic_types = set()
        if TWIST_STAMPED_TYPE in topic_types:
            return TWIST_STAMPED_TYPE
        if TWIST_TYPE in topic_types:
            return TWIST_TYPE
        return TWIST_STAMPED_TYPE

    def _endpoint_types(self, topic: str, kind: str) -> list[str]:
        if not topic:
            return []
        try:
            if kind == "publishers":
                infos = self.node.get_publishers_info_by_topic(topic)
            elif kind == "subscriptions":
                infos = self.node.get_subscriptions_info_by_topic(topic)
            else:
                return []
            return sorted({getattr(info, "topic_type", "") for info in infos if getattr(info, "topic_type", "")})
        except Exception:
            return []

    def _endpoint_count(self, topic: str, kind: str) -> int:
        if not topic:
            return 0
        try:
            if kind == "publishers":
                return len(self.node.get_publishers_info_by_topic(topic))
            if kind == "subscriptions":
                return len(self.node.get_subscriptions_info_by_topic(topic))
        except Exception:
            return 0
        return 0

    def discover_robots(self) -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.discover_robots()
        setup = self.state.get_setup()
        network = setup.get("network", {})
        topics_and_types = self.node.get_topic_names_and_types()
        nodes = sorted(set(self.node.get_node_names()))
        topic_names = sorted(name for name, _types in topics_and_types)
        topic_types = {name: types for name, types in topics_and_types}
        actions = self._action_names()
        candidate = self._score_turtlebot_candidate(topic_names, topic_types, nodes, actions)
        candidates = [
            item
            for item in getattr(self, "_last_turtlebot_candidates", [candidate])
            if item.get("score", 0) > 0
        ]
        result = {
            "ok": True,
            "mode": "ros2",
            "connection": base_connection_info(
                self._action_available(0.05), self._route_action_available(0.05), network
            ),
            "topics": [{"name": name, "types": topic_types.get(name, [])} for name in topic_names],
            "nodes": nodes,
            "actions": actions,
            "candidates": candidates,
            "message": "Discovery complete." if candidates else "No TurtleBot-like ROS graph found.",
            "network": public_network(network),
            "networkDiscovery": discover_same_subnet_ssh_hosts(network),
        }
        self.state.update_runtime({"connection": result["connection"]})
        return result

    def detect_active_robot_topics(self, active_robot: str = "") -> Dict[str, Any]:
        if self._is_fallback():
            return self._fallback.detect_active_robot_topics(active_robot)
        setup = self.state.get_setup()
        profiles = setup.get("robotProfiles") or {}
        selected_robot = str(active_robot or setup.get("activeRobot") or "tb3_2")
        profile = profiles.get(selected_robot) if isinstance(profiles, dict) else None
        namespace = normalized_namespace(
            profile.get("namespace") if isinstance(profile, dict) else "/"
        )
        topics_and_types = self.node.get_topic_names_and_types()
        topic_names = sorted(name for name, _types in topics_and_types)
        actions = self._action_names()
        candidate = self._score_turtlebot_candidate(
            topic_names,
            dict(topics_and_types),
            sorted(set(self.node.get_node_names())),
            actions,
        )
        candidates = getattr(self, "_last_turtlebot_candidates", [candidate])
        matching = [
            item
            for item in candidates
            if normalized_namespace(item.get("namespace")) == namespace
        ]
        if not matching:
            return {
                "ok": False,
                "mode": "ros2",
                "activeRobot": selected_robot,
                "namespace": namespace,
                "topics": {},
                "message": "No ROS graph topics were found for the active robot namespace.",
            }
        selected_candidate = max(matching, key=lambda item: int(item.get("score") or 0))
        detected = detected_candidate_topics(selected_candidate, topic_names, actions)
        return {
            "ok": bool(detected),
            "mode": "ros2",
            "activeRobot": selected_robot,
            "namespace": namespace,
            "topics": detected,
            "matchedTopics": selected_candidate.get("matchedTopics", []),
            "message": (
                "Detected active robot topics."
                if detected
                else "No configured TurtleBot topics were found for the active robot namespace."
            ),
        }

    def _action_names(self) -> list[str]:
        if hasattr(self.node, "get_action_names_and_types"):
            try:
                return sorted(name for name, _types in self.node.get_action_names_and_types())
            except Exception:
                pass
        actions = set()
        for name, _types in self.node.get_topic_names_and_types():
            marker = "/_action/"
            if marker in name:
                actions.add(name.split(marker, 1)[0])
        if self._action_available(0.01):
            actions.add(self.topics["goalAction"])
        if self._route_action_available(0.01):
            actions.add(self.topics["routeAction"])
        return sorted(actions)

    def _score_turtlebot_candidate(
        self,
        topic_names: list[str],
        topic_types: Dict[str, list[str]],
        nodes: list[str],
        actions: list[str],
    ) -> Dict[str, Any]:
        _ = topic_types
        topic_set = set(topic_names)
        node_text = " ".join(nodes).lower()
        suffixes = (
            "/scan",
            "/odom",
            "/cmd_vel",
            "/amcl_pose",
            "/goal_pose",
            "/initialpose",
            "/navigate_to_pose",
            "/navigate_through_poses",
            "/camera/camera/image_raw",
            "/camera/camera/image_raw/compressed",
            "/camera/color/image_raw",
            "/camera/color/image_raw/compressed",
            "/camera/image_raw",
            "/camera/image_raw/compressed",
            "/camera/camera/color/image_raw",
            "/camera/camera/color/image_raw/compressed",
        )
        namespaces = {""}
        for name in topic_names:
            for suffix in suffixes:
                namespace = namespace_from_topic(name, suffix)
                if namespace is not None:
                    namespaces.add(namespace)
        for action_name in actions:
            namespace = namespace_from_topic(action_name, "/navigate_to_pose")
            if namespace is None:
                namespace = namespace_from_topic(action_name, "/navigate_through_poses")
            if namespace is not None:
                namespaces.add(namespace)

        weights = {
            "scan": 20,
            "odom": 15,
            "tf": 15,
            "cmdVel": 10,
            "pose": 10,
            "camera": 5,
            "nav2": 15,
            "turtlebotNode": 10,
        }
        best: Optional[Dict[str, Any]] = None
        candidates: list[Dict[str, Any]] = []

        def first_existing(candidates: list[str]) -> str:
            for candidate in candidates:
                if candidate in topic_set and self._endpoint_count(candidate, "publishers") > 0:
                    return candidate
            for candidate in candidates:
                if candidate in topic_set:
                    return candidate
            return candidates[0]

        for namespace in sorted(namespaces):
            scan_topic = namespace_topic(namespace, "/scan")
            odom_topic = namespace_topic(namespace, "/odom")
            cmd_topic = namespace_topic(namespace, "/cmd_vel")
            pose_topic = namespace_topic(namespace, "/amcl_pose")
            initial_topic = namespace_topic(namespace, "/initialpose")
            goal_topic = namespace_topic(namespace, "/goal_pose")
            action_name = namespace_topic(namespace, "/navigate_to_pose")
            route_action_name = namespace_topic(namespace, "/navigate_through_poses")
            raw_camera = first_existing(
                [
                    namespace_topic(namespace, "/camera/camera/image_raw"),
                    namespace_topic(namespace, "/camera/color/image_raw"),
                    namespace_topic(namespace, "/camera/image_raw"),
                    namespace_topic(namespace, "/camera/camera/color/image_raw"),
                ]
            )
            compressed_camera = first_existing(
                [
                    namespace_topic(namespace, "/camera/camera/image_raw/compressed"),
                    namespace_topic(namespace, "/camera/color/image_raw/compressed"),
                    namespace_topic(namespace, "/camera/image_raw/compressed"),
                    namespace_topic(namespace, "/camera/camera/color/image_raw/compressed"),
                ]
            )
            namespace_text = namespace.strip("/").lower()
            namespace_node = bool(namespace_text and namespace_text in node_text)
            checks = {
                "scan": scan_topic in topic_set and self._endpoint_count(scan_topic, "publishers") > 0,
                "odom": odom_topic in topic_set and self._endpoint_count(odom_topic, "publishers") > 0,
                "tf": "/tf" in topic_set and "/tf_static" in topic_set,
                "cmdVel": cmd_topic in topic_set and self._endpoint_count(cmd_topic, "subscriptions") > 0,
                "pose": pose_topic in topic_set and self._endpoint_count(pose_topic, "publishers") > 0,
                "camera": (
                    raw_camera in topic_set and self._endpoint_count(raw_camera, "publishers") > 0
                )
                or (
                    compressed_camera in topic_set
                    and self._endpoint_count(compressed_camera, "publishers") > 0
                ),
                "nav2": (
                    action_name in actions or route_action_name in actions
                )
                and (
                    bool(namespace)
                    or self._action_available(0.01)
                    or self._route_action_available(0.01)
                ),
                "turtlebotNode": namespace_node
                or "turtlebot" in node_text
                or "robot_state_publisher" in node_text,
            }
            score = sum(weights[key] for key, ok in checks.items() if ok)
            matched_topics = [
                topic
                for topic in (
                    scan_topic,
                    odom_topic,
                    cmd_topic,
                    pose_topic,
                    raw_camera,
                    compressed_camera,
                    initial_topic,
                    goal_topic,
                    "/tf",
                    "/tf_static",
                )
                if topic in topic_set
            ]
            candidate = {
                "name": f"TurtleBot candidate {namespace or '/'}",
                "namespace": namespace or "/",
                "score": min(100, score),
                "checks": checks,
                "missing": [key for key, ok in checks.items() if not ok],
                "matchedTopics": matched_topics,
                "matchedNodes": [
                    node
                    for node in nodes
                    if "turtlebot" in node.lower()
                    or "robot" in node.lower()
                    or (namespace_text and namespace_text in node.lower())
                ],
                "recommendedTopics": {
                    "scan": scan_topic,
                    "odom": odom_topic,
                    "cmdVel": cmd_topic,
                    "pose": pose_topic,
                    "camera": raw_camera,
                    "compressedCamera": compressed_camera,
                    "initialPose": initial_topic,
                    "goalTopic": goal_topic,
                    "goalAction": action_name,
                    "routeAction": route_action_name,
                },
            }
            candidates.append(candidate)
            if best is None or candidate["score"] > best["score"]:
                best = candidate
        self._last_turtlebot_candidates = sorted(
            candidates,
            key=lambda item: (item.get("score", 0), item.get("namespace", "")),
            reverse=True,
        )
        return best or {
            "name": "TurtleBot candidate",
            "namespace": "/",
            "score": 0,
            "checks": {},
            "missing": [],
            "matchedTopics": [],
            "matchedNodes": [],
            "recommendedTopics": {},
        }


class RobotPoseMonitor:
    """A small ROS2 worker bound to one profile's ROS_DOMAIN_ID.

    ROS 2 contexts can coexist in one process.  The active profile keeps the
    full control bridge, while these workers subscribe to each other profile's
    AMCL/odom pose so their current bodies can be shown as avoidance obstacles.
    """

    def __init__(self, state: DashboardState, robot_id: str, profile: Dict[str, Any]) -> None:
        self.state = state
        self.robot_id = robot_id
        self.profile = profile
        self.node = None
        self.context = None
        self.executor = None
        self.rclpy = None
        self.thread: Optional[threading.Thread] = None
        self._stopped = threading.Event()
        self._last_amcl = 0.0
        self._start()

    def _start(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node

            connection = self.profile.get("connection") if isinstance(self.profile.get("connection"), dict) else {}
            domain_id = int(connection.get("rosDomainId") or 1)
            self.rclpy = rclpy
            self.context = Context()
            self.context.init(args=None, initialize_logging=False, domain_id=domain_id)
            self.node = Node(f"turtlebot_pose_{self.robot_id}", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            topics = self.profile.get("topics") if isinstance(self.profile.get("topics"), dict) else {}
            self.node.create_subscription(PoseWithCovarianceStamped, topics.get("pose", "/amcl_pose"), self._on_amcl, 10)
            self.thread = threading.Thread(target=self._spin, daemon=True, name=f"pose-{self.robot_id}")
            self.thread.start()
        except Exception as exc:
            self.state.update_robot_pose(
                self.robot_id,
                {"available": False, "error": str(exc), "stamp": now_iso()},
            )

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception as exc:
            if not self._stopped.is_set():
                self.state.update_robot_pose(
                    self.robot_id,
                    {"available": False, "error": str(exc), "stamp": now_iso()},
                )

    def _save_pose(self, pose: Any, source: str) -> None:
        yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        self.state.update_robot_pose(
            self.robot_id,
            {
                "available": True,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": yaw,
                "source": source,
                "stamp": now_iso(),
            },
        )

    def _on_amcl(self, msg: Any) -> None:
        self._last_amcl = time.monotonic()
        self._save_pose(msg.pose.pose, "amcl")

    def shutdown(self) -> None:
        self._stopped.set()
        shutdown_ros_executor(self.executor, self.node)
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass
        try:
            if self.context is not None:
                self.context.try_shutdown()
        except Exception:
            pass
        self.executor = None


class RobotPoseMonitorManager:
    def __init__(self, state: DashboardState) -> None:
        self.state = state
        self.monitors: Dict[str, RobotPoseMonitor] = {}
        self.reload()

    def reload(self) -> None:
        self.shutdown()
        setup = self.state.get_setup()
        active_robot, _ = active_robot_profile(setup)
        profiles = setup.get("robotProfiles") if isinstance(setup.get("robotProfiles"), dict) else {}
        retained_poses: Dict[str, Any] = {}
        for robot_id, profile in profiles.items():
            if robot_id == active_robot or not isinstance(profile, dict):
                continue
            self.monitors[robot_id] = RobotPoseMonitor(self.state, robot_id, profile)
        self.state.replace_robot_poses(retained_poses)

    def shutdown(self) -> None:
        for monitor in self.monitors.values():
            monitor.shutdown()
        self.monitors = {}


@dataclass
class AppContext:
    state: DashboardState
    ros: RosBridge
    pose_monitors: RobotPoseMonitorManager


class DashboardHttpServer(ThreadingHTTPServer):
    allow_reuse_address = False
    daemon_threads = True

    def handle_error(self, request: Any, client_address: Any) -> None:
        error = sys.exc_info()[1]
        if isinstance(error, (BrokenPipeError, ConnectionResetError, ConnectionAbortedError)):
            return
        super().handle_error(request, client_address)


class DashboardHandler(SimpleHTTPRequestHandler):
    context: AppContext

    def translate_path(self, path: str) -> str:
        parsed = urlparse(path)
        request_path = unquote(parsed.path)
        if request_path.startswith("/data/"):
            return str(safe_child_path(DATA_ROOT, request_path[len("/data/") :]))
        if request_path == "/":
            return str(WEB_ROOT / "index.html")
        return str(safe_child_path(WEB_ROOT, request_path.lstrip("/")))

    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, format: str, *args: Any) -> None:
        if self.path.startswith("/api/camera/") or self.path.startswith("/api/events"):
            return
        super().log_message(format, *args)

    def _camera_stream(self) -> None:
        boundary = "turtlebot-camera-frame"
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
            self.send_header("Connection", "close")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            sequence = -1
            last_sent_at = 0.0
            frame_interval = 1.0 / CAMERA_STREAM_MAX_FPS
            while True:
                content, content_type, next_sequence, enabled, stream_mode = (
                    self.context.state.wait_for_camera_frame(sequence, timeout=1.0)
                )
                if not enabled or stream_mode != "compressed":
                    break
                if next_sequence == sequence:
                    continue
                sequence = next_sequence
                delay = frame_interval - (time.monotonic() - last_sent_at)
                if delay > 0:
                    time.sleep(delay)
                last_sent_at = time.monotonic()
                header = (
                    f"--{boundary}\r\n"
                    f"Content-Type: {content_type}\r\n"
                    f"Content-Length: {len(content)}\r\n\r\n"
                ).encode("ascii")
                self.wfile.write(header)
                self.wfile.write(content)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            self.close_connection = True

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self.context.state.refresh_server_ip()
            self._json(self.context.state.public_snapshot())
            return
        if parsed.path == "/api/connection":
            self.context.state.refresh_server_ip()
            self._json(self.context.ros.connection_status())
            return
        if parsed.path == "/api/robot_check":
            self._json(self.context.ros.robot_check())
            return
        if parsed.path == "/api/diagnostics":
            self._json(self.context.ros.diagnostics_report())
            return
        if parsed.path == "/api/discover":
            self._json(self.context.ros.discover_robots())
            return
        if parsed.path == "/api/run_logs":
            self._json(self.context.state.run_log_payload())
            return
        if parsed.path == "/api/run_logs/download":
            payload = self.context.state.run_log_payload()
            content = payload["report"].encode("utf-8")
            filename = f"turtlebot_run_{payload['sessionId']}.md"
            try:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/markdown; charset=utf-8")
                self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            return
        if parsed.path == "/api/events":
            self._events()
            return
        if parsed.path == "/api/camera/stream":
            self._camera_stream()
            return
        if parsed.path == "/api/camera/frame":
            content, content_type = self.context.state.get_camera()
            try:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            return
        super().do_GET()

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        try:
            body = self._read_json()
            if parsed.path == "/api/maps":
                width_cm, height_cm = parse_map_editor_dimensions(body)
                cm_per_pixel = parse_map_editor_cm_per_pixel(body)
                width_pixels = width_cm / cm_per_pixel
                height_pixels = height_cm / cm_per_pixel
                if (
                    abs(width_pixels - round(width_pixels)) > 1e-6
                    or abs(height_pixels - round(height_pixels)) > 1e-6
                ):
                    raise ValueError("map dimensions must divide evenly by cmPerPixel")
                image_data = str(body.get("imageDataUrl") or "")
                if not image_data:
                    raise ValueError("map image data is required")
                name = str(body.get("name") or "").strip()[:80] or "새 맵"
                entry = map_library_entry(
                    f"map-{uuid.uuid4().hex}",
                    name,
                    save_editor_map_data_url(image_data),
                    int(round(width_pixels)),
                    int(round(height_pixels)),
                    resolution=cm_per_pixel / 100.0,
                )
                setup = self.context.state.get_setup()
                library = [
                    dict(item)
                    for item in (setup.get("mapLibrary") or [])
                    if isinstance(item, dict) and str(item.get("id") or "").strip()
                ]
                library.append(entry)
                self.context.state.update_setup({"mapLibrary": library})
                self._json(
                    {
                        "ok": True,
                        "map": entry,
                        "state": self.context.state.public_snapshot(),
                    }
                )
                return
            if parsed.path == "/api/maps/select":
                map_id = str(body.get("id") or "").strip()
                if not map_id:
                    raise ValueError("map id is required")
                setup = self.context.state.get_setup()
                selected = next(
                    (
                        dict(item)
                        for item in (setup.get("mapLibrary") or [])
                        if isinstance(item, dict) and str(item.get("id") or "") == map_id
                    ),
                    None,
                )
                if selected is None:
                    raise ValueError("selected map was not found")
                self.context.ros.cancel_goal()
                self.context.state.update_setup({"map": selected})
                self._json({"ok": True, "state": self.context.state.public_snapshot()})
                return
            if parsed.path == "/api/maps/delete":
                map_id = str(body.get("id") or "").strip()
                if not map_id:
                    raise ValueError("map id is required")
                if map_id == "default-map":
                    raise ValueError("the default map cannot be deleted")
                setup = self.context.state.get_setup()
                library = [
                    dict(item)
                    for item in (setup.get("mapLibrary") or [])
                    if isinstance(item, dict) and str(item.get("id") or "").strip()
                ]
                selected = next((item for item in library if str(item.get("id")) == map_id), None)
                if selected is None:
                    raise ValueError("selected map was not found")
                remaining = [item for item in library if str(item.get("id")) != map_id]
                patch: Dict[str, Any] = {"mapLibrary": remaining}
                if str((setup.get("map") or {}).get("id") or "") == map_id:
                    fallback_map = next(
                        (item for item in remaining if str(item.get("id")) == "default-map"),
                        remaining[0] if remaining else None,
                    )
                    if fallback_map is None:
                        raise ValueError("a replacement map is required")
                    self.context.ros.cancel_goal()
                    patch["map"] = fallback_map
                self.context.state.update_setup(patch)
                if not any(str(item.get("imageUrl")) == str(selected.get("imageUrl")) for item in remaining):
                    delete_editor_map_file(str(selected.get("imageUrl") or ""))
                self._json({"ok": True, "state": self.context.state.public_snapshot()})
                return
            if parsed.path == "/api/setup":
                old_setup = self.context.state.snapshot().get("setup", {})
                old_topics = old_setup.get("topics", {})
                initial_pose_values = None
                initial_pose_changed = False
                if "initialPose" in body:
                    initial_pose = body.get("initialPose")
                    if not isinstance(initial_pose, dict):
                        raise ValueError("initialPose must be an object.")
                    initial_pose_values = parse_pose(initial_pose)
                    normalized_initial_pose = {
                        "x": initial_pose_values[0],
                        "y": initial_pose_values[1],
                        "yaw": initial_pose_values[2],
                    }
                    initial_pose_changed = poses_differ(
                        old_setup.get("initialPose"), normalized_initial_pose
                    )
                    if initial_pose_changed:
                        body["initialPose"] = normalized_initial_pose
                    else:
                        # The setup form always includes initialPose. Do not reset the
                        # live pose when an unrelated setting is being saved.
                        body.pop("initialPose", None)
                network_patch = body.get("network")
                if isinstance(network_patch, dict):
                    network_patch.pop("robotSshPasswordConfigured", None)
                    if not str(network_patch.get("robotSshPassword") or ""):
                        network_patch.pop("robotSshPassword", None)
                profile_patch = body.get("robotProfiles")
                if isinstance(profile_patch, dict):
                    for profile in profile_patch.values():
                        if not isinstance(profile, dict):
                            continue
                        connection = profile.get("connection")
                        if isinstance(connection, dict):
                            connection.pop("robotSshPasswordConfigured", None)
                            if not str(connection.get("robotSshPassword") or ""):
                                connection.pop("robotSshPassword", None)
                map_data = body.pop("mapImageDataUrl", None)
                if map_data:
                    image_url = save_data_url(map_data)
                    map_patch = body.setdefault("map", {})
                    entry = map_library_entry(
                        f"map-{uuid.uuid4().hex}",
                        "업로드 맵",
                        image_url,
                        int(map_patch.get("widthPixels") or 0),
                        int(map_patch.get("heightPixels") or 0),
                    )
                    if entry["widthPixels"] <= 0 or entry["heightPixels"] <= 0:
                        raise ValueError("uploaded map dimensions are missing")
                    # Uploaded maps remain selectable just like maps made in the editor.
                    entry["resolution"] = float(map_patch.get("resolution") or 0.01)
                    entry["originX"] = float(map_patch.get("originX") or 0.0)
                    entry["originY"] = float(map_patch.get("originY") or 0.0)
                    entry["originYaw"] = float(map_patch.get("originYaw") or 0.0)
                    map_patch.update(entry)
                    library = [
                        dict(item)
                        for item in (old_setup.get("mapLibrary") or [])
                        if isinstance(item, dict) and str(item.get("id") or "").strip()
                    ]
                    library.append(entry)
                    body["mapLibrary"] = library
                if initial_pose_changed:
                    # A path planned from the former pose must not continue after
                    # the operator relocates the robot on the dashboard map.
                    self.context.ros.cancel_goal()
                synchronize_active_robot_topics(body, old_setup)
                self.context.state.update_setup(body)
                self.context.pose_monitors.reload()
                reload_result = None
                if any(key in body for key in ("topics", "activeRobot", "robotProfiles", "network")):
                    reload_result = self.context.ros.reload_setup()
                initial_pose_result = None
                if initial_pose_changed and initial_pose_values is not None:
                    initial_pose_result = self.context.ros.set_initial_pose(*initial_pose_values)
                self._json(
                    {
                        "ok": True,
                        "state": self.context.state.public_snapshot(),
                        "rosReload": reload_result,
                        "initialPoseApply": initial_pose_result,
                    }
                )
                return
            if parsed.path == "/api/topics/reset":
                setup = self.context.state.get_setup()
                active_robot = str(body.get("activeRobot") or setup.get("activeRobot") or "tb3_2")
                result = self.context.ros.detect_active_robot_topics(active_robot)
                detected_topics = result.get("topics")
                if not result.get("ok") or not isinstance(detected_topics, dict):
                    raise ValueError(result.get("message") or "No robot topics were detected.")
                updated_topics = deep_merge(setup.get("topics", {}), detected_topics)
                patch = {"activeRobot": active_robot, "topics": updated_topics}
                synchronize_active_robot_topics(patch, setup)
                self.context.state.update_setup(patch)
                self.context.pose_monitors.reload()
                result["rosReload"] = self.context.ros.reload_setup()
                if not result["rosReload"].get("ok"):
                    self.context.state.replace_setup(setup)
                    rollback = self.context.ros.reload_setup()
                    raise ValueError(
                        "Topic settings were restored because ROS bridge reload failed. "
                        f"Reload: {result['rosReload'].get('message')}; "
                        f"rollback: {rollback.get('message')}"
                    )
                result["state"] = self.context.state.public_snapshot()
                self._json(result)
                return
            if parsed.path == "/api/initial_pose":
                x, y, yaw = parse_pose(body)
                self.context.ros.cancel_goal()
                self.context.state.update_setup({"initialPose": {"x": x, "y": y, "yaw": yaw}})
                result = self.context.ros.set_initial_pose(x, y, yaw)
                self._json({**result, "state": self.context.state.public_snapshot()})
                return
            if parsed.path == "/api/goal":
                x, y, yaw = parse_pose(body)
                path = parse_navigation_path(body)
                result = self.context.ros.send_goal(
                    x,
                    y,
                    yaw,
                    path=path,
                    force_fallback=bool(body.get("forceFallback", False)),
                )
                self._json(result)
                return
            if parsed.path == "/api/route":
                poses = parse_route(body)
                path = parse_navigation_path(body)
                repeat_path = parse_navigation_path(body, "repeatPath")
                repeat = parse_route_repeat(body, len(poses))
                poses, path, repeat_path, repeat = select_route_repeat(
                    poses, path, repeat, repeat_path=repeat_path
                )
                result = self.context.ros.send_route(
                    poses,
                    path=path,
                    repeat_path=repeat_path,
                    force_fallback=bool(body.get("forceFallback", False)),
                    repeat=repeat,
                    resume=bool(body.get("resume", False)),
                )
                self._json(result)
                return
            if parsed.path == "/api/cancel":
                self._json(self.context.ros.cancel_goal())
                return
            if parsed.path == "/api/stop":
                self._json(self.context.ros.stop_robot())
                return
            if parsed.path == "/api/robot_bringup":
                self._json(self.context.ros.robot_bringup())
                return
            if parsed.path == "/api/robot_bringup_stop":
                self.context.ros.cancel_goal()
                self._json(run_robot_bringup_stop_from_state(self.context.state))
                return
            if parsed.path == "/api/robot_ssh_check":
                self._json(self.context.ros.robot_ssh_check(detailed=bool(body.get("detailed", False))))
                return
            if parsed.path == "/api/robot_hardware_check":
                self._json(run_robot_hardware_check_from_state(self.context.state))
                return
            if parsed.path == "/api/manual_drive":
                linear = float(body.get("linear", 0.0))
                angular = float(body.get("angular", 0.0))
                self._json(self.context.ros.manual_drive(linear, angular))
                return
            if parsed.path == "/api/manual_drive_check":
                self._json(self.context.ros.manual_drive_check())
                return
            if parsed.path == "/api/camera_control":
                runtime = self.context.ros.set_camera_enabled(bool(body.get("enabled", True)))
                self._json({"ok": True, "runtime": runtime})
                return
            if parsed.path == "/api/camera_stream":
                runtime = self.context.ros.set_camera_stream_mode(body.get("mode"))
                self._json({"ok": True, "runtime": runtime})
                return
            if parsed.path == "/api/run_logs/clear":
                self._json(self.context.state.clear_run_logs())
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return
        except Exception as exc:
            self._json({"ok": False, "error": str(exc)}, status=HTTPStatus.BAD_REQUEST)

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        if length > MAX_JSON_BODY_BYTES:
            raise ValueError(f"request body exceeds {MAX_JSON_BODY_BYTES // (1024 * 1024)} MiB")
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON request body must be an object.")
        return payload

    def _json(self, payload: Dict[str, Any], status: int = HTTPStatus.OK) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        last_payload = ""
        while True:
            try:
                payload = json.dumps(self.context.state.public_snapshot(), ensure_ascii=False)
                if payload != last_payload:
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    last_payload = payload
                time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return


def parse_pose(payload: Dict[str, Any]) -> Tuple[float, float, float]:
    x = finite_float(payload.get("x", 0.0), "x")
    y = finite_float(payload.get("y", 0.0), "y")
    yaw = normalize_yaw(finite_float(payload.get("yaw", 0.0), "yaw"))
    return x, y, yaw


def poses_differ(previous: Any, current: Any, tolerance: float = 1e-6) -> bool:
    if not isinstance(previous, dict) or not isinstance(current, dict):
        return True
    try:
        previous_x, previous_y, previous_yaw = parse_pose(previous)
        current_x, current_y, current_yaw = parse_pose(current)
    except (TypeError, ValueError):
        return True
    yaw_delta = normalize_yaw(current_yaw - previous_yaw)
    return any(
        abs(delta) > tolerance
        for delta in (current_x - previous_x, current_y - previous_y, yaw_delta)
    )


def parse_route(payload: Dict[str, Any]) -> list[Dict[str, Any]]:
    raw_poses = payload.get("poses", [])
    if not isinstance(raw_poses, list) or not raw_poses:
        raise ValueError("Route poses are required.")
    if len(raw_poses) > 20:
        raise ValueError("Route can contain at most 20 poses.")
    poses: list[Dict[str, Any]] = []
    for index, raw_pose in enumerate(raw_poses):
        if not isinstance(raw_pose, dict):
            raise ValueError("Each route pose must be an object.")
        x, y, yaw = parse_pose(raw_pose)
        source_index_value = finite_float(raw_pose.get("sourceIndex", index), "sourceIndex")
        source_index = int(source_index_value)
        if source_index_value != source_index or not 0 <= source_index < 20:
            raise ValueError("sourceIndex must be an integer from 0 to 19.")
        poses.append(
            {
                "x": x,
                "y": y,
                "yaw": yaw,
                "sourceIndex": source_index,
                "final": bool(raw_pose.get("final", False)),
            }
        )
    return poses


def normalized_route_repeat(
    repeat: Optional[Dict[str, Any]], route_length: int
) -> Dict[str, Any]:
    if route_length < 1:
        raise ValueError("Route is empty.")
    if repeat in (None, False):
        repeat = {}
    if not isinstance(repeat, dict):
        raise ValueError("repeat must be an object.")

    enabled = bool(repeat.get("enabled", False))

    def integer_field(name: str, default: int, minimum: int, maximum: int) -> int:
        raw_value = repeat.get(name, default)
        value = finite_float(raw_value, f"repeat.{name}")
        integer = int(value)
        if value != integer or not minimum <= integer <= maximum:
            raise ValueError(
                f"repeat.{name} must be an integer from {minimum} to {maximum}."
            )
        return integer

    start = integer_field("start", 1, 1, route_length)
    end = integer_field("end", route_length, 1, route_length)
    if end < start:
        raise ValueError("repeat.end must be greater than or equal to repeat.start.")
    if enabled and end == start:
        raise ValueError("Route repetition requires at least two numbered points.")

    pause_seconds = finite_float(repeat.get("pauseSeconds", 5.0), "repeat.pauseSeconds")
    if not 0.5 <= pause_seconds <= 3600.0:
        raise ValueError("repeat.pauseSeconds must be from 0.5 to 3600 seconds.")

    source_start = integer_field("sourceStart", start, 1, 20)
    source_end = integer_field("sourceEnd", end, source_start, 20)
    return {
        "enabled": enabled,
        "pauseSeconds": pause_seconds,
        "start": start,
        "end": end,
        "startIndex": start - 1,
        "endIndex": end - 1,
        "sourceStart": source_start,
        "sourceEnd": source_end,
    }


def parse_route_repeat(payload: Dict[str, Any], route_length: int) -> Dict[str, Any]:
    raw_repeat = payload.get("repeat")
    if raw_repeat is None:
        return normalized_route_repeat(None, route_length)
    return normalized_route_repeat(raw_repeat, route_length)


def select_route_repeat(
    poses: list[Dict[str, float]],
    path: list[Dict[str, Any]],
    repeat: Dict[str, Any],
    repeat_path: Optional[list[Dict[str, Any]]] = None,
) -> Any:
    legacy_result = repeat_path is None
    repeat_path = repeat_path or []
    if not repeat.get("enabled"):
        return (poses, path, repeat) if legacy_result else (poses, path, repeat_path, repeat)
    start_index = int(repeat["startIndex"])
    end_index = int(repeat["endIndex"])
    selected_poses = poses[start_index : end_index + 1]
    selected_path = path
    if path and any(isinstance(point.get("routeIndex"), int) for point in path):
        selected_path = []
        for point in path:
            marker = point.get("routeIndex")
            if not isinstance(marker, int) or not start_index <= marker <= end_index:
                continue
            selected_path.append({**point, "routeIndex": marker - start_index})
    execution_repeat = normalized_route_repeat(
        {
            "enabled": True,
            "pauseSeconds": repeat["pauseSeconds"],
            "start": 1,
            "end": len(selected_poses),
            "sourceStart": repeat["sourceStart"],
            "sourceEnd": repeat["sourceEnd"],
        },
        len(selected_poses),
    )
    # repeatPath is precomputed by the browser for the selected range.  If an
    # external client sends the full route, discard it rather than replaying a
    # mismatched path against the shortened route.
    selected_repeat_path = repeat_path if len(selected_poses) == len(poses) else []
    if legacy_result:
        return selected_poses, selected_path, execution_repeat
    return selected_poses, selected_path, selected_repeat_path, execution_repeat


def parse_navigation_path(payload: Dict[str, Any], field: str = "path") -> list[Dict[str, Any]]:
    raw_path = payload.get(field, [])
    if raw_path in (None, ""):
        return []
    if not isinstance(raw_path, list):
        raise ValueError(f"{field} must be an array.")
    if len(raw_path) > 5000:
        raise ValueError(f"{field} can contain at most 5000 points.")
    path: list[Dict[str, Any]] = []
    for raw_point in raw_path:
        if not isinstance(raw_point, dict):
            raise ValueError(f"Each {field} point must be an object.")
        x = finite_float(raw_point.get("x"), f"{field}.x")
        y = finite_float(raw_point.get("y"), f"{field}.y")
        point = {"x": x, "y": y, "slow": bool(raw_point.get("slow", False))}
        raw_route_index = raw_point.get("routeIndex")
        if raw_route_index is not None:
            route_index_value = finite_float(raw_route_index, f"{field}.routeIndex")
            route_index = int(route_index_value)
            if route_index_value != route_index or not 0 <= route_index < 20:
                raise ValueError(f"{field}.routeIndex must be an integer from 0 to 19.")
            point["routeIndex"] = route_index
        if path and math.hypot(path[-1]["x"] - x, path[-1]["y"] - y) < 1e-6:
            path[-1]["slow"] = path[-1]["slow"] or point["slow"]
            if "routeIndex" in point:
                path[-1]["routeIndex"] = max(
                    int(path[-1].get("routeIndex", 0)), point["routeIndex"]
                )
        else:
            path.append(point)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="TurtleBot3 web dashboard")
    parser.add_argument("--host", default=os.environ.get("TURTLEBOT_WEB_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("TURTLEBOT_WEB_PORT", "8080")))
    args = parser.parse_args()

    DATA_ROOT.mkdir(exist_ok=True)
    CONFIG_ROOT.mkdir(exist_ok=True)
    state = DashboardState()
    ros = RosBridge(state)
    pose_monitors = RobotPoseMonitorManager(state)
    DashboardHandler.context = AppContext(state=state, ros=ros, pose_monitors=pose_monitors)
    try:
        server = DashboardHttpServer((args.host, args.port), DashboardHandler)
    except OSError as exc:
        ros.shutdown()
        pose_monitors.shutdown()
        state.run_logs.close()
        raise SystemExit(
            f"Dashboard could not start on {args.host}:{args.port}. "
            "Stop the existing dashboard process before starting a new one. "
            f"({exc})"
        ) from exc
    print(f"TurtleBot3 web dashboard: http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.", flush=True)
    finally:
        try:
            ros.shutdown()
        finally:
            try:
                pose_monitors.shutdown()
                state.run_logs.close()
            finally:
                server.server_close()


if __name__ == "__main__":
    main()
