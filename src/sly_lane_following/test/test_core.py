"""Semantic checks on constructed scenes; no tuning/validation bag labels used."""

from copy import deepcopy

import cv2
import numpy as np
import pytest

from sly_lane_following.core import LaneDetector, curve_x


WIDTH, HEIGHT = 768, 480


@pytest.fixture
def camera():
    return {
        "width": WIDTH, "height": HEIGHT,
        "K": [450.0, 0.0, 384.0, 0.0, 450.0, 240.0, 0.0, 0.0, 1.0],
        "D": [0.0] * 5, "distortion_model": "plumb_bob",
    }


def boundary(identity, near, far=None, y_min=270, y_max=475, score=1.0):
    """A visible painted boundary with position specified at both ends."""
    far = near if far is None else far
    slope = (near - far) / ((y_max - y_min) / HEIGHT)
    return {
        "id": identity, "coef": [0.0, slope, far - slope * y_min / HEIGHT],
        "height": HEIGHT, "y_min": float(y_min), "y_max": float(y_max),
        "support": 30, "residual_px": 0.0, "score": score,
    }


def paint_frame(shift=0, neighbor_brightness=240, only_one=False):
    image = np.full((HEIGHT, WIDTH, 3), 35, np.uint8)
    cv2.line(image, (320 + shift, 270), (190 + shift, 475), (145,) * 3, 7)
    if not only_one:
        cv2.line(image, (430 + shift, 270), (550 + shift, 475), (145,) * 3, 7)
        if neighbor_brightness:
            cv2.line(image, (550 + shift, 270), (715 + shift, 475), (neighbor_brightness,) * 3, 7)
    return image


def assert_pixels_only(result, stamp):
    assert result["stamp_ns"] == stamp
    assert result["metric_valid"] is False
    assert result["metric_reason"] == "NOT_USED_IMAGE_SERVO"
    assert result["coordinate_space"] == "undistorted_image_pixels"
    assert result["debug_image"].shape == (HEIGHT, WIDTH, 3)


def test_multi_boundary_selection_stays_in_current_adjacent_corridor():
    detector = LaneDetector()
    candidates = [boundary(1, 130, 240, score=1.0),
                  boundary(2, 290, 340, score=0.75),
                  boundary(3, 545, 435, score=0.75),
                  boundary(4, 720, 550, score=1.0)]
    chosen, _, _, rejected = detector.select(candidates, WIDTH, HEIGHT, 384)
    assert chosen is not None
    assert chosen["ids"] == [2, 3]
    assert any(r["reason"] == "SPANS_ANOTHER_BOUNDARY" for r in rejected)
    assert all(290 <= x <= 545 for x, _ in chosen["center"])


def test_end_to_end_dimmer_ego_paint_beats_brighter_adjacent_lane(camera):
    detector = LaneDetector()
    result = detector.process(paint_frame(), camera, 1_000_000_000)
    assert_pixels_only(result, 1_000_000_000)
    assert result["vision_valid"], result["reason"]
    assert len(result["candidates"]) >= 3
    left, right = np.asarray(result["left"]), np.asarray(result["right"])
    # Check the actual visible boundaries, rather than relying on assigned IDs.
    expected_left = 320 + (190 - 320) * (450 - 270) / (475 - 270)
    expected_right = 430 + (550 - 430) * (450 - 270) / (475 - 270)
    assert abs(np.interp(450, left[:, 1], left[:, 0]) - expected_left) < 10
    assert abs(np.interp(450, right[:, 1], right[:, 0]) - expected_right) < 10
    assert min(p[1] for p in result["center"]) >= max(min(p[1] for p in result["left"]), min(p[1] for p in result["right"]))


def test_missing_boundary_does_not_invent_centerline(camera):
    result = LaneDetector().process(paint_frame(only_one=True), camera, 1_000_000_000)
    assert_pixels_only(result, 1_000_000_000)
    assert result["vision_valid"] is False
    assert result["center"] == []
    assert result["selected"] == []


def test_crossing_boundaries_are_not_a_valid_corridor():
    detector = LaneDetector()
    candidates = [boundary(1, 200, 460), boundary(2, 560, 300)]
    chosen, _, _, _ = detector.select(candidates, WIDTH, HEIGHT, 384)
    assert chosen is None


@pytest.mark.parametrize("use_history", [False, True])
def test_two_unresolved_corridors_remain_ambiguous_even_with_history(use_history):
    detector = LaneDetector()
    # Distinct disconnected near/far hypotheses cannot establish one continuous
    # corridor. Their comparable support is not permission to guess a route.
    candidates = [boundary(1, 200, y_min=300, y_max=400),
                  boundary(2, 568, y_min=300, y_max=400),
                  boundary(3, 218, y_min=410, y_max=475),
                  boundary(4, 550, y_min=410, y_max=475)]
    if use_history:
        detector.previous_pair = (1, 2)
    chosen, reason, pairs, _ = detector.select(candidates, WIDTH, HEIGHT, 384)
    assert len(pairs) == 2
    assert chosen is None
    assert reason == "AMBIGUOUS_CORRIDOR"


def test_two_edges_of_one_paint_stripe_merge_but_separate_lines_do_not():
    detector = LaneDetector()
    one_stripe = np.zeros((HEIGHT, WIDTH), np.uint8)
    for x in (200, 208):
        cv2.line(one_stripe, (x, 270), (x, 475), 255, 3)
    merged = detector.candidates(one_stripe)
    assert len(merged) == 1
    assert 198 < float(curve_x(merged[0], 400)) < 211
    separate = np.zeros_like(one_stripe)
    for x in (200, 230):
        cv2.line(separate, (x, 270), (x, 475), 255, 3)
    assert len(detector.candidates(separate)) == 2


def test_adjacent_lane_brightening_does_not_change_tracked_identity(camera):
    detector = LaneDetector()
    results = [detector.process(paint_frame(neighbor_brightness=b), camera, 1_000_000_000 + i * 33_333_333)
               for i, b in enumerate((150, 180, 220, 255))]
    assert all(r["vision_valid"] for r in results)
    assert all(r["selected"] == results[0]["selected"] for r in results)
    centers = [np.asarray(r["center"]) for r in results]
    assert all(np.max(np.abs(c - centers[0])) < 3 for c in centers[1:])


def test_reacquisition_requires_uninterrupted_evidence():
    detector = LaneDetector({"switch_frames": 4})
    detector.previous_pair = (1, 2)
    new_pair = [boundary(3, 220, 330), boundary(4, 550, 440)]
    assert detector.select(new_pair, WIDTH, HEIGHT, 384)[0] is None
    assert detector.select([], WIDTH, HEIGHT, 384)[0] is None
    # A detection before the interruption must not count towards the new run.
    for _ in range(3):
        assert detector.select(new_pair, WIDTH, HEIGHT, 384)[0] is None
    assert detector.select(new_pair, WIDTH, HEIGHT, 384)[0] is not None


@pytest.mark.parametrize("next_stamp", [5_000_000_000, 11_000_000_000])
def test_clock_rewind_and_large_gap_reset_old_corridor(camera, next_stamp):
    detector = LaneDetector()
    first = detector.process(paint_frame(neighbor_brightness=0), camera, 10_000_000_000)
    assert first["vision_valid"]
    # This observation has moved far enough that old IDs/selection cannot be
    # reused. A new playback segment must initialize from current evidence.
    changed = detector.process(paint_frame(shift=70, neighbor_brightness=0), camera, next_stamp)
    assert_pixels_only(changed, next_stamp)
    assert changed["vision_valid"], changed["reason"]
    assert np.mean(np.asarray(changed["center"])[:, 0]) > np.mean(np.asarray(first["center"])[:, 0]) + 50


@pytest.mark.parametrize("corruption", ["dimensions", "zero_focal", "nonfinite", "bottom_row", "model", "distortion"])
def test_bad_camera_info_never_produces_valid_geometry(camera, corruption):
    bad = deepcopy(camera)
    if corruption == "dimensions": bad["width"] = 1280
    elif corruption == "zero_focal": bad["K"][0] = 0.0
    elif corruption == "nonfinite": bad["K"][4] = float("nan")
    elif corruption == "bottom_row": bad["K"][8] = 2.0
    elif corruption == "model": bad["distortion_model"] = "equidistant"
    elif corruption == "distortion": bad["D"] = [0.0, 0.0, 0.0]
    result = LaneDetector().process(paint_frame(), bad, 1_000_000_000)
    assert_pixels_only(result, 1_000_000_000)
    assert result["vision_valid"] is False
    assert result["selected"] == [] and result["center"] == []


def test_texture_without_painted_boundaries_does_not_create_path(camera):
    # Bright isolated road speckle must not become a long supported paint line.
    rng = np.random.default_rng(13)
    gray = np.clip(rng.normal(45, 7, (HEIGHT, WIDTH)), 0, 255).astype(np.uint8)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    result = LaneDetector().process(image, camera, 1_000_000_000)
    assert result["vision_valid"] is False
    assert result["center"] == []
