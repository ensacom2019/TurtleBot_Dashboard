import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import server


class ServerHelpersTest(unittest.TestCase):
    def test_default_map_matches_saved_map_setup(self) -> None:
        setup = server.DEFAULT_STATE["setup"]
        self.assertEqual(setup["map"]["imageUrl"], "/data/Sprite-1.png")
        self.assertEqual((setup["map"]["widthPixels"], setup["map"]["heightPixels"]), (1800, 1800))
        self.assertEqual(setup["initialPose"], {"x": 0.188, "y": 0.224, "yaw": 1.571})
        self.assertEqual(setup["robot"], {"length": 0.25, "width": 0.15, "radius": 0.146})
        self.assertEqual(len(setup["obstacles"]), 2)
        self.assertEqual(
            [(item["width"], item["height"]) for item in setup["obstacles"]],
            [(0.13, 0.13), (0.13, 0.10)],
        )
        self.assertEqual(
            [(item["x"], item["y"]) for item in setup["obstacles"]],
            [(1.027, 0.449), (1.218, 1.342)],
        )
        self.assertEqual(setup["planner"]["hardClearance"], 0.05)
        self.assertEqual(setup["fallbackNavigation"]["softDistance"], 0.10)
        self.assertEqual(setup["map"]["id"], "default-map")
        self.assertEqual(setup["mapLibrary"][0]["name"], "기본 맵")

    def test_default_robot_profile_only_contains_turtlebot_2(self) -> None:
        profiles = server.DEFAULT_STATE["setup"]["robotProfiles"]
        self.assertEqual(list(profiles), ["tb3_2"])

    def test_legacy_turtlebot_1_profile_is_removed_but_discovered_profile_remains(self) -> None:
        legacy_state = server.deep_merge(
            server.DEFAULT_STATE,
            {
                "setup": {
                    "activeRobot": "tb3_1",
                    "robotProfiles": {"tb3_1": {"namespace": "/tb3_1"}},
                }
            },
        )
        normalized_legacy = server.normalize_robot_profiles_state(legacy_state)
        self.assertEqual(list(normalized_legacy["setup"]["robotProfiles"]), ["tb3_2"])
        self.assertEqual(normalized_legacy["setup"]["activeRobot"], "tb3_2")

        discovered_state = server.deep_merge(
            server.DEFAULT_STATE,
            {
                "setup": {
                    "robotProfiles": {
                        "tb3_1": {"namespace": "/tb3_1", "source": "discovered"}
                    }
                }
            },
        )
        normalized_discovered = server.normalize_robot_profiles_state(discovered_state)
        self.assertIn("tb3_1", normalized_discovered["setup"]["robotProfiles"])

    def test_map_editor_dimensions_require_whole_centimeters(self) -> None:
        self.assertEqual(
            server.parse_map_editor_dimensions({"widthCm": 180, "heightCm": 120}),
            (180, 120),
        )
        with self.assertRaises(ValueError):
            server.parse_map_editor_dimensions({"widthCm": 180.5, "heightCm": 120})
        with self.assertRaises(ValueError):
            server.parse_map_editor_dimensions({"widthCm": 9, "heightCm": 120})

    def test_map_editor_entry_uses_one_centimeter_per_pixel(self) -> None:
        entry = server.map_library_entry("map-test", "실험 맵", "/data/map-test.png", 180, 120)
        self.assertEqual(entry["resolution"], 0.01)
        self.assertEqual((entry["widthPixels"], entry["heightPixels"]), (180, 120))

    def test_map_editor_cm_per_pixel_accepts_coarser_scale(self) -> None:
        self.assertEqual(server.parse_map_editor_cm_per_pixel({"cmPerPixel": 10}), 10.0)
        entry = server.map_library_entry(
            "map-test", "축척 맵", "/data/map-test.png", 18, 18, resolution=0.1
        )
        self.assertEqual(entry["resolution"], 0.1)
        self.assertEqual((entry["widthPixels"], entry["heightPixels"]), (18, 18))

    def test_editor_maps_are_saved_under_the_internal_maps_folder(self) -> None:
        self.assertEqual(server.MAP_DATA_ROOT, server.DATA_ROOT / "maps")

    def test_robot_bringup_stop_script_targets_dashboard_sessions(self) -> None:
        script = server.build_robot_bringup_stop_script(
            {"rosDomainId": "1", "rosLocalhostOnly": "0"}, {"cmdVel": "/cmd_vel"}
        )
        self.assertIn("tb3_base tb3_nav2 tb3_camera", script)
        self.assertIn("/cmd_vel", script)

    def test_server_ip_prefers_robot_subnet(self) -> None:
        selected = server.select_server_ip(
            "192.168.20.7",
            ["172.21.160.1", "192.168.20.3", "192.168.35.229"],
            routed_ip="192.168.35.229",
        )
        self.assertEqual(selected, "192.168.20.3")

    def test_server_ip_uses_routed_address_without_subnet_match(self) -> None:
        selected = server.select_server_ip(
            "192.168.20.7",
            ["127.0.0.1", "192.168.35.229", "not-an-ip"],
            routed_ip="192.168.35.229",
        )
        self.assertEqual(selected, "192.168.35.229")

    def test_same_subnet_hosts_stays_within_private_24(self) -> None:
        subnet, hosts = server.same_subnet_hosts("192.168.20.3")
        self.assertEqual(str(subnet), "192.168.20.0/24")
        self.assertEqual(len(hosts), 253)
        self.assertNotIn("192.168.20.3", hosts)
        self.assertIn("192.168.20.7", hosts)

    def test_same_subnet_hosts_rejects_public_or_invalid_addresses(self) -> None:
        self.assertEqual(server.same_subnet_hosts("8.8.8.8"), (None, []))
        self.assertEqual(server.same_subnet_hosts("not-an-ip"), (None, []))

    def test_root_namespace_topics_have_one_leading_slash(self) -> None:
        topics = server.topics_for_namespace("/", "camera_ros")
        self.assertEqual(topics["cmdVel"], "/cmd_vel")
        self.assertEqual(topics["camera"], "/camera/camera/image_raw")
        self.assertEqual(topics["routeAction"], "/navigate_through_poses")

    def test_namespaced_route_action(self) -> None:
        topics = server.topics_for_namespace("/tb3_1")
        self.assertEqual(topics["goalAction"], "/tb3_1/navigate_to_pose")
        self.assertEqual(topics["routeAction"], "/tb3_1/navigate_through_poses")

    def test_finite_float_rejects_non_finite_values(self) -> None:
        for value in (math.nan, math.inf, -math.inf, "NaN"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                server.finite_float(value, "value")

    def test_parse_pose_normalizes_yaw(self) -> None:
        x, y, yaw = server.parse_pose({"x": "1.2", "y": -0.3, "yaw": math.tau + 0.5})
        self.assertEqual((x, y), (1.2, -0.3))
        self.assertAlmostEqual(yaw, 0.5)

    def test_poses_differ_ignores_equivalent_wrapped_yaw(self) -> None:
        previous = {"x": 0.2, "y": 0.3, "yaw": 0.5}
        current = {"x": 0.2, "y": 0.3, "yaw": math.tau + 0.5}
        self.assertFalse(server.poses_differ(previous, current))

    def test_poses_differ_detects_position_change(self) -> None:
        previous = {"x": 0.2, "y": 0.3, "yaw": 0.5}
        current = {"x": 0.21, "y": 0.3, "yaw": 0.5}
        self.assertTrue(server.poses_differ(previous, current))

    def test_parse_navigation_path_preserves_slow_cells_and_deduplicates(self) -> None:
        path = server.parse_navigation_path(
            {
                "path": [
                    {"x": 0.1, "y": 0.2, "slow": False, "routeIndex": 0},
                    {"x": 0.1, "y": 0.2, "slow": True, "routeIndex": 1},
                    {"x": 0.3, "y": 0.4, "slow": False, "routeIndex": 1},
                ]
            }
        )
        self.assertEqual(len(path), 2)
        self.assertTrue(path[0]["slow"])
        self.assertEqual([point["routeIndex"] for point in path], [1, 1])

    def test_laser_scan_points_filters_invalid_ranges(self) -> None:
        points = server.laser_scan_points(
            [float("inf"), 0.05, 0.5, 8.0],
            angle_min=0.0,
            angle_increment=math.pi / 2,
            range_min=0.1,
            range_max=3.5,
        )
        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], -0.5)
        self.assertAlmostEqual(points[0][1], 0.0, places=6)

    def test_lidar_display_points_are_bounded_and_rounded(self) -> None:
        points = [(index / 1000.0, -index / 2000.0) for index in range(720)]
        sampled = server.sample_lidar_points(points, max_points=240)
        self.assertEqual(len(sampled), 240)
        self.assertEqual(sampled[0], [0.0, 0.0])
        self.assertEqual(sampled[1], [0.003, -0.002])

    def test_lidar_safety_marks_predicted_collision_without_early_recovery(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        result = server.evaluate_lidar_safety(
            [(0.22, 0.0)],
            linear=0.06,
            angular=0.0,
            footprint=footprint,
            hard_margin=0.03,
            soft_distance=0.08,
            horizon=1.5,
        )
        self.assertTrue(result["collision"])
        self.assertTrue(result["slow"])
        self.assertFalse(result["recoveryRequired"])

    def test_lidar_safety_starts_recovery_at_hard_clearance(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        result = server.evaluate_lidar_safety(
            [(0.18, 0.0)],
            linear=0.06,
            angular=0.0,
            footprint=footprint,
            hard_margin=0.03,
            soft_distance=0.08,
            horizon=1.5,
        )
        self.assertTrue(result["recoveryRequired"])
        self.assertFalse(result["slow"])

    def test_lidar_coast_requires_recent_clear_scan_and_fresh_pose(self) -> None:
        settings = server.normalized_fallback_settings({"fallbackNavigation": {}})
        last_safety = {
            "recoveryRequired": False,
            "collision": False,
            "minClearance": 0.05,
            "motionClearance": 0.30,
        }
        self.assertTrue(
            server.lidar_coast_allowed(0.8, 0.1, 0.1, settings, last_safety)
        )
        self.assertFalse(
            server.lidar_coast_allowed(1.5, 0.1, 0.1, settings, last_safety)
        )
        self.assertFalse(
            server.lidar_coast_allowed(0.8, 0.7, 0.1, settings, last_safety)
        )
        self.assertFalse(
            server.lidar_coast_allowed(
                0.8, 0.1, 0.1, settings, {**last_safety, "collision": True}
            )
        )

    def test_lidar_coast_command_caps_speed_and_turn(self) -> None:
        settings = server.normalized_fallback_settings({"fallbackNavigation": {}})
        linear, angular = server.lidar_coast_command(
            {"linear": 0.08, "angular": 0.6}, settings
        )
        self.assertEqual(linear, settings["minLinear"])
        self.assertEqual(angular, server.FALLBACK_LIDAR_COAST_MAX_ANGULAR)

    def test_lidar_safety_keeps_speed_in_clear_space(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        result = server.evaluate_lidar_safety(
            [(1.0, 0.0)],
            linear=0.06,
            angular=0.0,
            footprint=footprint,
            hard_margin=0.03,
            soft_distance=0.08,
            horizon=1.5,
        )
        self.assertFalse(result["collision"])
        self.assertEqual(result["scale"], 1.0)

    def test_lidar_safety_does_not_throttle_for_a_side_point_while_driving_forward(self) -> None:
        footprint = {"front": 0.125, "back": 0.125, "left": 0.075, "right": 0.075}
        result = server.evaluate_lidar_safety(
            [(0.0, 0.12)],
            linear=0.08,
            angular=0.0,
            footprint=footprint,
            hard_margin=0.03,
            soft_distance=0.10,
            horizon=1.0,
        )

        self.assertFalse(result["collision"])
        self.assertEqual(result["scale"], 1.0)
        self.assertAlmostEqual(result["minClearance"], 0.045)
        self.assertIsNone(result["motionClearance"])

    def test_lidar_recovery_turns_toward_clearer_side(self) -> None:
        turn = server.choose_lidar_recovery_turn([(0.4, -0.1), (0.9, 0.3)])
        self.assertEqual(turn, 1)

    def test_lidar_recovery_checks_reverse_path(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        self.assertTrue(
            server.lidar_recovery_motion_safe(
                [(0.18, 0.0)], -0.02, 0.0, footprint, horizon=0.8
            )
        )

    def test_lidar_recovery_can_reverse_out_of_existing_overlap(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        points = [(0.14, 0.0)]
        self.assertTrue(
            server.lidar_recovery_motion_safe(
                points, -0.02, 0.0, footprint, horizon=0.8
            )
        )
        self.assertFalse(
            server.lidar_recovery_motion_safe(
                points, 0.025, 0.0, footprint, horizon=0.8
            )
        )

    def test_lidar_recovery_uses_reverse_when_turn_worsens_overlap(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        recovery = server.next_lidar_recovery_command(
            [(0.14, 0.0)],
            footprint,
            {"phase": "stop", "started": 100.0, "attempts": 0, "turn": 1},
            now=100.3,
        )
        self.assertEqual(recovery["phase"], "reverse")
        self.assertLess(recovery["linear"], 0.0)

    def test_lidar_recovery_rechecks_trapped_state(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        recovery = server.next_lidar_recovery_command(
            [(0.14, 0.0)],
            footprint,
            {"phase": "trapped", "started": 100.0, "attempts": 4, "turn": 1},
            now=100.8,
        )
        self.assertEqual(recovery["phase"], "stop")
        self.assertEqual(recovery["attempts"], 0)
        self.assertFalse(
            server.lidar_recovery_motion_safe(
                [(-0.16, 0.0)], -0.02, 0.0, footprint, horizon=0.8
            )
        )
        self.assertFalse(
            server.lidar_recovery_motion_safe(
                [(0.16, 0.0)], 0.025, 0.0, footprint, horizon=0.8
            )
        )

    def test_lidar_recovery_turns_immediately_toward_clearer_side(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        points = [(0.18, 0.0), (0.8, 0.4)]
        recovery = server.next_lidar_recovery_command(
            points,
            footprint,
            {"phase": "none", "started": 0.0, "attempts": 0, "turn": 0},
            now=100.0,
        )
        self.assertEqual(recovery["phase"], "turn")
        self.assertNotEqual(recovery["angular"], 0.0)

    def test_lidar_recovery_turn_can_transition_to_safe_forward_motion(self) -> None:
        footprint = {"front": 0.15, "back": 0.15, "left": 0.10, "right": 0.10}
        recovery = server.next_lidar_recovery_command(
            [(-0.5, 0.0)],
            footprint,
            {"phase": "turn", "started": 100.0, "attempts": 0, "turn": 1},
            now=101.3,
        )
        self.assertEqual(recovery["phase"], "forward")
        self.assertGreater(recovery["linear"], 0.0)
        self.assertEqual(recovery["angular"], 0.0)

    def test_fallback_path_command_slows_on_soft_cell(self) -> None:
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])
        command = server.compute_fallback_command(
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            [
                {"x": 0.0, "y": 0.0, "slow": False},
                {"x": 0.12, "y": 0.0, "slow": True},
                {"x": 0.3, "y": 0.0, "slow": False},
            ],
            0,
            0.0,
            settings,
        )
        self.assertFalse(command["reached"])
        self.assertTrue(command["slow"])
        self.assertAlmostEqual(command["linear"], settings["minLinear"])

    def test_fallback_path_command_finishes_at_goal_heading(self) -> None:
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])
        command = server.compute_fallback_command(
            {"x": 0.3, "y": 0.0, "yaw": 0.0},
            [{"x": 0.3, "y": 0.0, "slow": False}],
            0,
            0.0,
            settings,
        )
        self.assertTrue(command["reached"])

    def test_fallback_inflation_can_be_disabled(self) -> None:
        setup = server.json.loads(server.json.dumps(server.DEFAULT_STATE["setup"]))
        setup["fallbackNavigation"]["softDistance"] = 0.0
        settings = server.normalized_fallback_settings(setup)
        self.assertEqual(settings["softDistance"], 0.0)

    def test_fallback_sensor_timeouts_are_limited_to_slider_range(self) -> None:
        setup = server.json.loads(server.json.dumps(server.DEFAULT_STATE["setup"]))
        setup["fallbackNavigation"]["scanTimeout"] = 0.1
        setup["fallbackNavigation"]["odomTimeout"] = 5.0
        settings = server.normalized_fallback_settings(setup)
        self.assertEqual(settings["scanTimeout"], 0.5)
        self.assertEqual(settings["odomTimeout"], 1.0)

    def test_stamp_conversion_and_monotonic_extrapolation(self) -> None:
        stamp = SimpleNamespace(sec=12, nanosec=345)
        anchor = server.stamp_to_nanoseconds(stamp)
        self.assertEqual(anchor, 12 * server.NANOSECONDS_PER_SECOND + 345)
        self.assertEqual(server.extrapolate_clock_nanoseconds(anchor, 1_000, 1_250), anchor + 250)
        self.assertEqual(server.extrapolate_clock_nanoseconds(anchor, 1_250, 1_000), anchor)

    def test_invalid_stamp_is_rejected(self) -> None:
        self.assertIsNone(server.stamp_to_nanoseconds(SimpleNamespace(sec=0, nanosec=0)))
        self.assertIsNone(
            server.stamp_to_nanoseconds(
                SimpleNamespace(sec=1, nanosec=server.NANOSECONDS_PER_SECOND)
            )
        )

    def test_safe_child_path_blocks_parent_and_absolute_paths(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(server.safe_child_path(root, "app.js"), (root / "app.js").resolve())
            self.assertEqual(server.safe_child_path(root, "../secret"), root / "__invalid_path__")
            self.assertEqual(server.safe_child_path(root, str(root.parent / "secret")), root / "__invalid_path__")

    def test_public_network_hides_password(self) -> None:
        network = server.public_network({"robotIp": "192.168.20.7", "robotSshPassword": "1234"})
        self.assertEqual(network["robotSshPassword"], "")
        self.assertTrue(network["robotSshPasswordConfigured"])

    def test_image_conversion_rejects_short_rows(self) -> None:
        invalid = SimpleNamespace(width=2, height=1, step=3, encoding="bgr8", data=b"\x00" * 3)
        self.assertIsNone(server.image_msg_to_bmp(invalid))

    def test_image_conversion_builds_bmp(self) -> None:
        image = SimpleNamespace(width=1, height=1, step=3, encoding="bgr8", data=b"\x01\x02\x03")
        result = server.image_msg_to_bmp(image)
        self.assertIsNotNone(result)
        self.assertTrue(result.startswith(b"BM"))

    def test_cmd_vel_delivery_script_is_zero_motion_monitor(self) -> None:
        script = server.build_cmd_vel_delivery_check_script(
            server.DEFAULT_STATE["setup"]["network"],
            "/cmd_vel",
            server.TWIST_STAMPED_TYPE,
        )
        self.assertIn("ros2 topic echo /cmd_vel geometry_msgs/msg/TwistStamped --once", script)
        self.assertIn("DASHBOARD_CMD_VEL_ECHO_RC", script)
        self.assertNotIn("linear: {x:", script)

    def test_robot_bringup_verifies_opencr_before_launch(self) -> None:
        script = server.build_robot_bringup_script(
            server.DEFAULT_STATE["setup"]["network"]
        )
        self.assertLess(script.index("/dev/ttyACM1"), script.index("/dev/ttyACM0"))
        self.assertIn("is_opencr_port", script)
        self.assertIn("Arduino|Uno", script)
        self.assertIn("usb_port:=$OPENCR_PORT", script)
        self.assertIn("turtlebot3_node did not become ready", script)
        self.assertIn("Refusing to launch turtlebot3_node", script)

    def test_ssh_diagnostics_checks_both_nav2_actions(self) -> None:
        topics = server.topics_for_namespace("/")
        script = server.build_robot_ssh_diagnostics_script(
            server.DEFAULT_STATE["setup"]["network"], topics
        )
        self.assertIn("ros2 action info /navigate_to_pose", script)
        self.assertIn("ros2 action info /navigate_through_poses", script)

    def test_route_uses_navigate_through_poses_when_single_goal_action_is_missing(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {"route": [], "routeIndex": 0}

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, event_type, **data):
                pass

            def snapshot(self):
                return {"runtime": dict(self.runtime)}

        class RouteGoal:
            def __init__(self) -> None:
                self.poses = []

        class Future:
            def add_done_callback(self, callback) -> None:
                self.callback = callback

        class Client:
            def send_goal_async(self, goal, feedback_callback=None):
                self.goal = goal
                self.feedback_callback = feedback_callback
                return Future()

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge.topics = {
            "routeAction": "/navigate_through_poses",
            "goalAction": "/navigate_to_pose",
        }
        bridge.NavigateThroughPoses = SimpleNamespace(Goal=RouteGoal)
        bridge.route_action_client = Client()
        bridge.route_goal_handle = None
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 0
        bridge._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
        bridge._route_transport = ""
        bridge._route_pause_thread = None
        bridge._shutdown_event = server.threading.Event()
        bridge._action_available = lambda _timeout: False
        bridge._route_action_available = lambda _timeout: True
        bridge._pose_stamped = lambda x, y, yaw: (x, y, yaw)
        bridge._prepare_autonomous_navigation = lambda: None

        result = bridge.send_route(
            [{"x": 0.2, "y": 0.3, "yaw": 0.0}, {"x": 0.7, "y": 0.9, "yaw": 1.0}]
        )

        self.assertTrue(result["ok"])
        self.assertEqual(result["transport"], "navigate_through_poses")
        self.assertEqual(len(bridge.route_action_client.goal.poses), 2)
        self.assertEqual(bridge.state.runtime["goal"]["x"], 0.2)

    def test_route_prefers_working_navigate_to_pose_sequence(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {"route": [], "routeIndex": 0}

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, event_type, **data):
                pass

            def snapshot(self):
                return {"runtime": dict(self.runtime)}

        class PoseGoal:
            def __init__(self) -> None:
                self.pose = None

        class Future:
            def add_done_callback(self, callback) -> None:
                self.callback = callback

        class Client:
            def send_goal_async(self, goal, feedback_callback=None):
                self.goal = goal
                self.feedback_callback = feedback_callback
                return Future()

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge.NavigateToPose = SimpleNamespace(Goal=PoseGoal)
        bridge.action_client = Client()
        bridge.goal_handle = None
        bridge._pose_stamped = lambda x, y, yaw: (x, y, yaw)
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 0
        bridge._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
        bridge._route_transport = ""
        bridge._route_pause_thread = None
        bridge._shutdown_event = server.threading.Event()
        bridge._prepare_autonomous_navigation = lambda: None
        bridge._action_available = lambda _timeout: True
        bridge._route_action_available = lambda _timeout: self.fail(
            "NavigateThroughPoses should not replace a working NavigateToPose sequence"
        )
        result = bridge.send_route(
            [{"x": 0.2, "y": 0.3, "yaw": 0.0}, {"x": 0.7, "y": 0.9, "yaw": 1.0}]
        )

        self.assertTrue(result["ok"])
        self.assertEqual(result["transport"], "navigate_to_pose_sequence")
        self.assertEqual(bridge.action_client.goal.pose, (0.2, 0.3, 0.0))
        self.assertEqual(len(bridge.state.runtime["route"]), 2)

    def test_sequential_route_dispatches_next_waypoint_after_success(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {
                    "route": [
                        {"x": 0.2, "y": 0.3, "yaw": 0.0},
                        {"x": 0.7, "y": 0.9, "yaw": 1.0},
                    ],
                    "routeIndex": 0,
                    "routeRepeatCycle": 1,
                }

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, event_type, **data):
                pass

            def snapshot(self):
                return {"runtime": dict(self.runtime)}

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge.goal_handle = object()
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 7
        bridge._route_repeat = {"enabled": False, "pauseSeconds": 5.0}
        bridge._route_transport = "navigate_to_pose_sequence"
        dispatched = []
        dispatched_event = server.threading.Event()

        def dispatch(generation, route_index):
            dispatched.append((generation, route_index))
            dispatched_event.set()

        bridge._dispatch_route_pose = dispatch
        future = SimpleNamespace(result=lambda: SimpleNamespace(status=4))

        bridge._on_route_pose_result(future, 7, 0)

        self.assertTrue(dispatched_event.wait(1.0))
        self.assertEqual(dispatched, [(7, 1)])
        self.assertEqual(bridge.state.runtime["routeIndex"], 1)
        self.assertEqual(bridge.state.runtime["goal"]["x"], 0.7)

    def test_stale_waypoint_result_cannot_clear_current_goal(self) -> None:
        bridge = object.__new__(server.RosBridge)
        current_handle = object()
        bridge.goal_handle = current_handle
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 8
        bridge.state = SimpleNamespace(
            snapshot=lambda: {"runtime": {"route": [], "routeIndex": 0}},
            update_runtime=lambda _patch: self.fail("stale result changed runtime"),
            log_run_event=lambda *_args, **_kwargs: None,
        )
        future = SimpleNamespace(result=lambda: SimpleNamespace(status=4))

        bridge._on_route_pose_result(future, 7, 0)

        self.assertIs(bridge.goal_handle, current_handle)

    def test_repeat_resume_starts_first_waypoint_and_next_cycle(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {
                    "route": [
                        {"x": 0.2, "y": 0.3, "yaw": 0.0},
                        {"x": 0.7, "y": 0.9, "yaw": 1.0},
                    ],
                    "routeIndex": 1,
                    "routeRepeatCycle": 3,
                }

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, event_type, **data):
                pass

            def snapshot(self):
                return {"runtime": dict(self.runtime)}

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 4
        bridge._route_repeat = {"enabled": True, "pauseSeconds": 0.5}
        bridge._route_transport = "navigate_to_pose_sequence"
        bridge._shutdown_event = server.threading.Event()
        dispatched = []
        bridge._dispatch_route_pose = lambda generation, index: dispatched.append(
            (generation, index)
        )

        bridge._resume_route_repeat(4, 0.0)

        self.assertEqual(dispatched, [(4, 0)])
        self.assertEqual(bridge.state.runtime["routeRepeatCycle"], 4)
        self.assertEqual(bridge.state.runtime["routeIndex"], 0)
        self.assertEqual(bridge.state.runtime["goal"]["x"], 0.2)

    def test_repeat_route_parser_selects_numbered_range(self) -> None:
        poses = [
            {"x": 0.1, "y": 0.1, "yaw": 0.0},
            {"x": 0.2, "y": 0.2, "yaw": 0.0},
            {"x": 0.3, "y": 0.3, "yaw": 1.0},
        ]
        path = [
            {"x": 0.1, "y": 0.1, "slow": False, "routeIndex": 0},
            {"x": 0.2, "y": 0.2, "slow": False, "routeIndex": 1},
            {"x": 0.3, "y": 0.3, "slow": True, "routeIndex": 2},
        ]
        repeat = server.normalized_route_repeat(
            {"enabled": True, "start": 2, "end": 3, "pauseSeconds": 2.5},
            len(poses),
        )

        selected_poses, selected_path, execution = server.select_route_repeat(
            poses, path, repeat
        )

        self.assertEqual([pose["x"] for pose in selected_poses], [0.2, 0.3])
        self.assertEqual([point["routeIndex"] for point in selected_path], [0, 1])
        self.assertEqual(execution["sourceStart"], 2)
        self.assertEqual(execution["sourceEnd"], 3)
        self.assertEqual(execution["pauseSeconds"], 2.5)

    def test_repeat_route_requires_two_points(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least two"):
            server.normalized_route_repeat(
                {"enabled": True, "start": 1, "end": 1, "pauseSeconds": 5},
                1,
            )

    def test_fallback_command_respects_current_route_segment_end(self) -> None:
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])
        command = server.compute_fallback_command(
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            [
                {"x": 0.0, "y": 0.0, "slow": False},
                {"x": 0.2, "y": 0.0, "slow": False},
                {"x": 0.02, "y": 0.01, "slow": False},
                {"x": 0.5, "y": 0.5, "slow": False},
            ],
            0,
            0.0,
            settings,
            path_end_index=1,
        )
        self.assertLessEqual(command["pathIndex"], 1)

    def test_fallback_route_does_not_finish_before_a_waypoint(self) -> None:
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])
        command = server.compute_fallback_command(
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            [
                {"x": 0.0, "y": 0.0, "slow": False},
                {"x": 0.3, "y": 0.0, "slow": False},
                {"x": 0.0, "y": 0.01, "slow": False},
            ],
            0,
            0.0,
            settings,
            path_end_index=1,
        )
        self.assertFalse(command["reached"])
        self.assertLessEqual(command["pathIndex"], 1)

    def test_forced_lidar_goal_bypasses_nav2_action_discovery(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {}
                self.events = []

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, event_type, **data):
                self.events.append((event_type, data))

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge._is_fallback = lambda: False
        bridge._prepare_autonomous_navigation = lambda: None
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 1
        bridge._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
        bridge._route_transport = ""
        bridge._action_available = lambda _timeout: self.fail(
            "Nav2 discovery must be skipped in forced LiDAR mode"
        )
        captured = {}

        def start_fallback(path, route, queue_if_stale=False):
            captured.update(path=path, route=route, queue=queue_if_stale)
            return {"ok": True, "transport": "lidar_fallback_waiting"}

        bridge._start_lidar_fallback = start_fallback
        result = bridge.send_goal(
            0.8,
            0.4,
            0.2,
            path=[{"x": 0.8, "y": 0.4, "slow": False}],
            force_fallback=True,
        )

        self.assertTrue(result["ok"])
        self.assertTrue(result["forced"])
        self.assertEqual(result["transport"], "lidar_fallback_waiting")
        self.assertTrue(captured["queue"])
        self.assertEqual(captured["route"][0]["x"], 0.8)
        self.assertTrue(bridge.state.events[0][1]["forceFallback"])

    def test_forced_lidar_route_passes_precomputed_repeat_path(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {}

            def update_runtime(self, patch):
                self.runtime.update(patch)

            def log_run_event(self, _event_type, **_data):
                pass

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge._is_fallback = lambda: False
        bridge._prepare_autonomous_navigation = lambda: None
        bridge._route_sequence_lock = server.threading.RLock()
        bridge._route_sequence_generation = 0
        bridge._route_repeat = {"enabled": False, "pauseSeconds": 0.0}
        bridge._route_transport = ""
        bridge._action_available = lambda _timeout: self.fail("Nav2 must be skipped")
        bridge._route_action_available = lambda _timeout: self.fail("Nav2 must be skipped")
        captured = {}

        def start_fallback(path, route, **kwargs):
            captured.update(path=path, route=route, **kwargs)
            return {"ok": True, "transport": "lidar_fallback"}

        bridge._start_lidar_fallback = start_fallback
        repeat_path = [
            {"x": 0.8, "y": 0.2, "slow": False, "routeIndex": 0},
            {"x": 0.2, "y": 0.2, "slow": False, "routeIndex": 1},
        ]
        result = bridge.send_route(
            [{"x": 0.2, "y": 0.2, "yaw": 0.0}, {"x": 0.8, "y": 0.2, "yaw": 0.0}],
            path=[{"x": 0.2, "y": 0.2, "slow": False, "routeIndex": 0}],
            repeat_path=repeat_path,
            force_fallback=True,
            repeat={"enabled": True, "start": 1, "end": 2, "pauseSeconds": 1.0},
        )

        self.assertTrue(result["ok"])
        self.assertTrue(result["forced"])
        self.assertEqual(captured["repeat_path"], repeat_path)
        self.assertTrue(captured["repeat"]["enabled"])

    def test_lidar_fallback_refuses_stale_scan(self) -> None:
        bridge = object.__new__(server.RosBridge)
        bridge.state = SimpleNamespace(
            get_setup=lambda: server.DEFAULT_STATE["setup"],
            update_runtime=lambda _patch: None,
        )
        bridge.topics = {"scan": "/scan", "odom": "/odom", "cmdVel": "/cmd_vel"}
        bridge._scan_lock = server.threading.Lock()
        bridge._latest_scan = {"points": [], "received": 0.0, "frame": "base_scan"}
        bridge._last_odom_monotonic = server.time.monotonic()
        bridge._last_pose_monotonic = server.time.monotonic()
        bridge._fallback_sensor_startup_wait = 0.0

        result = bridge._start_lidar_fallback(
            [{"x": 0.0, "y": 0.0, "slow": False}],
            [{"x": 0.0, "y": 0.0, "yaw": 0.0}],
        )

        self.assertFalse(result["ok"])
        self.assertIn("/scan age missing", result["error"])

    def test_lidar_fallback_waits_for_fresh_sensor_samples(self) -> None:
        bridge = object.__new__(server.RosBridge)
        bridge._scan_lock = server.threading.Lock()
        bridge._latest_scan = {"points": [], "received": 0.0, "frame": "base_scan"}
        bridge._last_odom_monotonic = 0.0
        bridge._last_pose_monotonic = 0.0
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])

        def publish_samples() -> None:
            server.time.sleep(0.05)
            received = server.time.monotonic()
            with bridge._scan_lock:
                bridge._latest_scan = {
                    "points": [(1.0, 0.0)],
                    "received": received,
                    "frame": "base_scan",
                }
            bridge._last_odom_monotonic = received
            bridge._last_pose_monotonic = received

        thread = server.threading.Thread(target=publish_samples)
        thread.start()
        ages = bridge._wait_for_fresh_fallback_sensors(settings, timeout=0.4)
        thread.join(timeout=1.0)

        self.assertLessEqual(ages["scan"], settings["scanTimeout"])
        self.assertLessEqual(ages["odom"], settings["odomTimeout"])
        self.assertLessEqual(ages["pose"], settings["odomTimeout"])

    def test_lidar_fallback_queues_goal_until_sensors_recover(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {}

            def get_setup(self):
                return server.DEFAULT_STATE["setup"]

            def update_runtime(self, patch):
                self.runtime.update(patch)

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge.topics = {"scan": "/scan", "odom": "/odom", "cmdVel": "/cmd_vel"}
        bridge._scan_lock = server.threading.Lock()
        bridge._latest_scan = {"points": [], "received": 0.0, "frame": "base_scan"}
        bridge._last_odom_monotonic = 0.0
        bridge._last_pose_monotonic = 0.0
        bridge._fallback_sensor_startup_wait = 0.0
        bridge._fallback_nav_lock = server.threading.RLock()
        bridge._fallback_nav_generation = 0
        bridge._fallback_nav_active = False
        bridge._fallback_nav_thread = None
        bridge._shutdown_event = server.threading.Event()
        bridge._endpoint_count = lambda _topic, _kind: 1
        bridge._fallback_navigation_loop = lambda *_args: None

        def publish_samples() -> None:
            server.time.sleep(0.05)
            received = server.time.monotonic()
            with bridge._scan_lock:
                bridge._latest_scan = {
                    "points": [(1.0, 0.0)],
                    "received": received,
                    "frame": "base_scan",
                }
            bridge._last_odom_monotonic = received
            bridge._last_pose_monotonic = received

        sensor_thread = server.threading.Thread(target=publish_samples)
        sensor_thread.start()
        result = bridge._start_lidar_fallback(
            [
                {"x": 0.0, "y": 0.0, "slow": False},
                {"x": 0.3, "y": 0.0, "slow": False},
            ],
            [{"x": 0.3, "y": 0.0, "yaw": 0.0}],
            queue_if_stale=True,
        )
        pending_thread = bridge._fallback_nav_thread
        pending_thread.join(timeout=1.0)
        sensor_thread.join(timeout=1.0)

        self.assertTrue(result["ok"])
        self.assertEqual(result["transport"], "lidar_fallback_waiting")
        self.assertEqual(bridge.state.runtime["navStatus"], "fallback_starting")
        self.assertTrue(bridge.state.runtime["fallbackActive"])

    def test_lidar_fallback_accepts_fresh_single_publisher_path(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {}

            def get_setup(self):
                return server.DEFAULT_STATE["setup"]

            def update_runtime(self, patch):
                self.runtime.update(patch)

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge.topics = {"scan": "/scan", "odom": "/odom", "cmdVel": "/cmd_vel"}
        bridge._scan_lock = server.threading.Lock()
        bridge._latest_scan = {
            "points": [(1.0, 0.0)],
            "received": server.time.monotonic(),
            "frame": "base_scan",
        }
        bridge._last_odom_monotonic = server.time.monotonic()
        bridge._last_pose_monotonic = server.time.monotonic()
        bridge._fallback_nav_lock = server.threading.RLock()
        bridge._fallback_nav_generation = 0
        bridge._fallback_nav_active = False
        bridge._fallback_nav_thread = None
        bridge._endpoint_count = lambda _topic, _kind: 1
        bridge._fallback_navigation_loop = lambda *_args: None

        result = bridge._start_lidar_fallback(
            [
                {"x": 0.0, "y": 0.0, "slow": False},
                {"x": 0.3, "y": 0.0, "slow": True},
            ],
            [{"x": 0.3, "y": 0.0, "yaw": 0.0}],
        )
        bridge._fallback_nav_thread.join(timeout=1.0)

        self.assertTrue(result["ok"])
        self.assertEqual(result["transport"], "lidar_fallback")
        self.assertTrue(bridge.state.runtime["fallbackActive"])

    def test_lidar_fallback_loop_stops_at_completed_goal(self) -> None:
        class FakeState:
            def __init__(self) -> None:
                self.runtime = {"pose": {"x": 0.3, "y": 0.0, "yaw": 0.0}}

            def snapshot(self):
                return {"runtime": dict(self.runtime)}

            def update_runtime(self, patch):
                self.runtime.update(patch)

        bridge = object.__new__(server.RosBridge)
        bridge.state = FakeState()
        bridge._scan_lock = server.threading.Lock()
        bridge._latest_scan = {
            "points": [(1.0, 0.0)],
            "received": server.time.monotonic(),
        }
        bridge._last_odom_monotonic = server.time.monotonic()
        bridge._last_pose_monotonic = server.time.monotonic()
        bridge._fallback_nav_lock = server.threading.RLock()
        bridge._fallback_nav_generation = 1
        bridge._fallback_nav_active = True
        bridge._shutdown_event = server.threading.Event()
        published = []
        bridge._publish_cmd_vel = lambda linear, angular, repeats=1, interval=0.0: published.append(
            (linear, angular, repeats)
        )
        settings = server.normalized_fallback_settings(server.DEFAULT_STATE["setup"])

        bridge._fallback_navigation_loop(
            1,
            [{"x": 0.3, "y": 0.0, "slow": False}],
            [{"x": 0.3, "y": 0.0, "yaw": 0.0}],
            0.0,
            settings,
            server.DEFAULT_STATE["setup"],
        )

        self.assertEqual(bridge.state.runtime["navStatus"], "succeeded")
        self.assertFalse(bridge.state.runtime["fallbackActive"])
        self.assertEqual(published[-1], (0.0, 0.0, 12))

    def test_run_log_sanitizer_redacts_secrets_and_non_finite_values(self) -> None:
        sanitized = server.sanitize_run_log_value(
            {
                "robotSshPassword": "1234",
                "nested": {"apiSecret": "hidden", "distance": math.inf},
            }
        )
        self.assertEqual(sanitized["robotSshPassword"], "***")
        self.assertEqual(sanitized["nested"]["apiSecret"], "***")
        self.assertEqual(sanitized["nested"]["distance"], "inf")

    def test_run_log_store_exports_and_starts_new_session(self) -> None:
        original_root = server.RUN_LOG_ROOT
        try:
            with tempfile.TemporaryDirectory() as directory:
                server.RUN_LOG_ROOT = Path(directory)
                store = server.RunLogStore()
                first_session = store.session_id
                store.record("manual_command", linear=0.08, password="1234")
                report = store.report(
                    {
                        "setup": {
                            "network": {"robotSshPassword": "1234"},
                            "topics": {"cmdVel": "/cmd_vel"},
                        },
                        "runtime": {"navStatus": "manual"},
                    }
                )
                self.assertIn("manual_command", report)
                self.assertNotIn("1234", report)
                self.assertEqual(store.summary()["eventCount"], 1)
                cleared = store.clear()
                self.assertNotEqual(cleared["sessionId"], first_session)
                self.assertEqual(cleared["eventCount"], 1)
                store.close()
        finally:
            server.RUN_LOG_ROOT = original_root


if __name__ == "__main__":
    unittest.main()
