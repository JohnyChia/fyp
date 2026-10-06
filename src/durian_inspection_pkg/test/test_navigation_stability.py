import importlib.util
import math
from pathlib import Path
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))

from localization_stability import (
    LocalizationGuardPolicy,
    PoseSample,
    StationaryStabilityGate,
    evaluate_pose_continuity,
)


def policy():
    return LocalizationGuardPolicy.from_mapping({
        "enabled": True,
        "source": "gazebo_world_odometry",
        "global_frame": "map",
        "odom_frame": "odom",
        "max_translation_step_m": 0.15,
        "max_yaw_step_rad": 0.20,
        "max_linear_speed_mps": 0.60,
        "max_angular_speed_radps": 2.0,
        "stationary_linear_speed_mps": 0.03,
        "stationary_angular_speed_radps": 0.05,
        "stationary_translation_drift_m": 0.08,
        "stationary_yaw_drift_rad": 0.12,
        "settling_interval_sec": 1.0,
        "settling_timeout_sec": 8.0,
        "required_consecutive_good_samples": 3,
        "tf_unavailable_timeout_sec": 1.0,
    })


def test_pose_jump_guard_accepts_normal_and_plausible_motion():
    p = policy()
    start = PoseSample(1.0, 0.0, 0.0, 0.0)
    normal = evaluate_pose_continuity(start, PoseSample(1.1, 0.01, 0.0, 0.01), 0.0, 0.0, p)
    assert normal.accepted
    moving = evaluate_pose_continuity(start, PoseSample(2.0, 0.5, 0.0, 0.5), 0.5, 0.5, p)
    assert moving.accepted


def test_batched_simulator_yaw_uses_pose_source_elapsed_time():
    """The observed failed run moved 0.491 rad over a batched sim interval."""
    result = evaluate_pose_continuity(
        PoseSample(100.0, 8.0, -0.4, -1.5),
        PoseSample(100.2, 8.009, -0.4, -1.009),
        0.1,
        0.3,
        policy(),
        previous_was_stationary=False,
    )
    assert result.accepted
    assert result.elapsed_sec == pytest.approx(0.2)
    assert result.allowed_yaw_rad == pytest.approx(0.6)


def test_same_yaw_change_without_source_time_is_rejected():
    result = evaluate_pose_continuity(
        PoseSample(100.0, 8.0, -0.4, -1.5),
        PoseSample(100.0, 8.009, -0.4, -1.009),
        0.1,
        0.3,
        policy(),
        previous_was_stationary=False,
    )
    assert not result.accepted
    assert result.reason == "LOCALIZATION_POSE_JUMP"


def test_stationary_multimeter_jump_is_rejected():
    result = evaluate_pose_continuity(
        PoseSample(1.0, 9.11, -1.49, 0.0),
        PoseSample(1.1, 3.13, -0.60, 0.0),
        0.0, 0.0, policy(),
    )
    assert not result.accepted
    assert result.reason == "LOCALIZATION_POSE_JUMP"
    assert result.translation_delta_m > 6.0


def test_arrival_sample_uses_moving_interval_before_stationary_limits():
    result = evaluate_pose_continuity(
        PoseSample(1.0, 0.0, 0.0, 0.0),
        PoseSample(2.0, 0.5, 0.0, 0.5),
        0.0, 0.0, policy(), previous_was_stationary=False,
    )
    assert result.accepted


def test_stationary_gate_requires_time_and_consecutive_samples():
    gate = StationaryStabilityGate(policy())
    assert not gate.observe(PoseSample(0.0, 1.0, 2.0, 0.0), 0.0, 0.0)
    assert not gate.observe(PoseSample(0.5, 1.01, 2.0, 0.01), 0.0, 0.0)
    assert gate.observe(PoseSample(1.1, 1.01, 2.0, 0.01), 0.0, 0.0)


def _generator_module():
    path = WORKSPACE / "experiment/tools/generate_navigation_map.py"
    spec = importlib.util.spec_from_file_location("generate_navigation_map", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_navigation_map_is_generic_and_contains_all_six_trunks():
    source = yaml.safe_load(
        (WORKSPACE / "experiment/controlled_world_spec.yaml").read_text()
    )
    pixels, metadata = _generator_module().build_occupancy(source)
    assert metadata["tree_count"] == 6
    min_x, min_y, _ = metadata["origin"]
    max_y = min_y + (metadata["height"] - 1) * metadata["resolution"]
    for tree in source["trees"]:
        x, y = map(float, tree["pose"][:2])
        column = round((x - min_x) / metadata["resolution"])
        row = round((max_y - y) / metadata["resolution"])
        assert pixels[row][column] == 0


def test_navigation_map_supports_different_tree_count():
    source = {
        "geometry": {"trunk_radius_m": 0.3},
        "navigation_map": {
            "resolution_m": 0.1,
            "bounds_margin_m": 2.0,
            "static_obstacle_padding_m": 0.0,
            "include_points": [[0.0, 0.0]],
        },
        "trees": [
            {"pose": [1.0, 2.0, 0, 0, 0, 0]},
            {"pose": [-3.0, 4.0, 0, 0, 0, 0]},
            {"pose": [6.0, -2.0, 0, 0, 0, 0]},
        ],
    }
    _, metadata = _generator_module().build_occupancy(source)
    assert metadata["tree_count"] == 3


def test_single_global_tf_authority_and_static_global_costmap():
    launch = (PACKAGE / "launch/navigation_launch.py").read_text()
    assert "simulation_map_to_odom" in launch
    assert "'publish_tf': False" in launch
    assert "'map_frame_id': 'rtabmap_map'" in launch
    assert launch.count("'map', 'odom'") == 1
    urdf = (PACKAGE / "urdf/robot.urdf").read_text()
    assert "<odometry_source>1</odometry_source>" in urdf
    assert "('map', '/rtabmap/grid_map')" in launch
    nav = yaml.safe_load((PACKAGE / "config/nav2_params.yaml").read_text())
    global_params = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    local_params = nav["local_costmap"]["local_costmap"]["ros__parameters"]
    assert global_params["global_frame"] == "map"
    assert global_params["rolling_window"] is False
    assert "static_layer" in global_params["plugins"]
    assert "obstacle_layer" in global_params["plugins"]
    assert "obstacle_layer" in local_params["plugins"]
    assert nav["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] is False


def test_navigation_runtime_has_no_disease_gt_leakage_or_fixed_route():
    runtime = "\n".join(
        (PACKAGE / name).read_text()
        for name in ("inspection_server.py", "localization_stability.py")
    )
    assert "dominant_class" not in runtime
    assert "expected_class_counts" not in runtime
    assert "fixed_route" not in runtime


def test_capture_tf_uses_image_timestamp_with_bounded_wait():
    vision = (PACKAGE / "vision_node.py").read_text()
    assert 'stamp = rclpy.time.Time.from_msg(rgb_msg.header.stamp)' in vision
    assert "lookup_transform(target, source, stamp, timeout)" in vision
    assert 'declare_parameter("tf_lookup_timeout_sec", 0.3)' in vision


def test_navigation_failure_reason_is_published_and_persisted():
    server = (PACKAGE / "inspection_server.py").read_text()
    assert "def _finish_failed_mission(self, reason):" in server
    assert "processing_error=?" in server
    assert "status='ABORTED_MISSION'" in server
    assert "status IN ('CAPTURED', 'PROCESSING')" in server
    assert '"FAILED", terminal=True, success=False, reason=reason' in server
    for reason in ("LOCALIZATION_POSE_JUMP", "LOCALIZATION_TF_UNAVAILABLE"):
        assert reason in server


def test_localization_continuity_uses_tf_source_timestamp():
    server = (PACKAGE / "inspection_server.py").read_text()
    assert "source_timestamp = float(stamp.sec)" in server
    assert "sample = PoseSample(source_timestamp" in server
    assert "sample = PoseSample(time.monotonic()" not in server


def test_gui_shutdown_joins_spin_before_destroying_node():
    gui = (PACKAGE / "gui.py").read_text()
    shutdown = gui.split("    def shutdown(self):", 1)[1].split("\ndef main", 1)[0]
    assert shutdown.index("rclpy.shutdown()") < shutdown.index("ros_spin_thread.join")
    assert shutdown.index("ros_spin_thread.join") < shutdown.index("destroy_node()")


def test_return_home_uses_captured_authoritative_map_pose():
    server = (PACKAGE / "inspection_server.py").read_text()
    assert "self.home_x, self.home_y, self.home_yaw = rx, ry, map_yaw" in server
    assert 'pose.header.frame_id = "map"' in server
    assert "self.home_x, self.home_y, self.home_yaw," in server


def test_view_policy_metadata_matches_runtime_and_controlled_source():
    source = yaml.safe_load(
        (WORKSPACE / "experiment/controlled_world_spec.yaml").read_text()
    )["inspection_policy"]
    ground_truth = yaml.safe_load(
        (WORKSPACE / "experiment/ground_truth/durian_farm_scale_1x_gt.yaml").read_text()
    )["inspection_policy"]
    runtime = yaml.safe_load(
        (PACKAGE / "config/leaf_detector_deployment.yaml").read_text()
    )["inspection_policy"]
    for key in ("version", "radius_m", "angular_offsets_deg", "capture_duration_sec"):
        assert ground_truth[key] == source[key] == runtime[key]
