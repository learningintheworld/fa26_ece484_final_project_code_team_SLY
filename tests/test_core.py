"""Behavior/invariant tests; no simulator, ROS installation, or GPU required."""
from dataclasses import replace
import math
import unittest

from sly_autonomy.control import ControlConfig, pure_pursuit
from sly_autonomy.safety import Inputs, SafetyConfig, SafetySupervisor


def fresh(now, **changes):
    return replace(Inputs(path_stamp=now, odom_stamp=now, perception_stamp=now,
                          path_valid=True, perception_valid=True, speed_mps=0.0,
                          obstacle_present=False), **changes)


class PurePursuitTests(unittest.TestCase):
    def setUp(self):
        self.config = ControlConfig()

    def test_straight_path_has_zero_steering_and_speed_cap(self):
        result = pure_pursuit([(0, 0), (1, 0), (10, 0)], self.config)
        self.assertEqual(result.steering_rad, 0.0)
        self.assertEqual(result.target, (2.0, 0.0))
        self.assertEqual(result.speed_mps, 1.0)

    def test_circle_geometry_and_mirror_symmetry(self):
        # Circle of radius 4 centred at (0, 4), with a target at (4, 4).
        config = replace(self.config, lookahead_m=10.0, max_steering_rad=1.0)
        left = pure_pursuit([(0, 0), (4, 4)], config)
        right = pure_pursuit([(0, 0), (4, -4)], config)
        self.assertAlmostEqual(math.tan(left.steering_rad) / config.wheelbase_m, 1 / 4)
        self.assertAlmostEqual(left.steering_rad, -right.steering_rad)
        self.assertAlmostEqual(left.speed_mps, right.speed_mps)

    def test_steering_saturation_and_curvature_speed_limit(self):
        result = pure_pursuit([(0, 0), (0.2, 1.0)], self.config)
        self.assertEqual(result.steering_rad, self.config.max_steering_rad)
        self.assertLess(result.speed_mps, self.config.cruise_speed_mps)
        self.assertLessEqual(result.speed_mps ** 2 * abs(result.curvature_per_m),
                             self.config.max_lateral_accel_mps2 + 1e-12)

    def test_duplicates_do_not_change_target(self):
        regular = pure_pursuit([(0, 0), (4, 0)], self.config)
        duplicate = pure_pursuit([(0, 0), (0, 0), (4, 0)], self.config)
        self.assertEqual(regular, duplicate)

    def test_no_usable_path_never_generates_command(self):
        paths = [[], [(1, 0)], [(2, 0), (2, 0)], [(0, 0), (0.01, 0)],
                 [(-2, 0), (-1, 0)], [(0, 0), (float("nan"), 0)],
                 [(0, 0), (float("inf"), 0)], [(0, 0), (0, 2)]]
        for path in paths:
            with self.subTest(path=path), self.assertRaises(ValueError):
                pure_pursuit(path, self.config)

    def test_bad_parameters_fail_at_startup(self):
        for field in vars(self.config):
            for value in (0, -1, float("nan"), float("inf")):
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    replace(self.config, **{field: value})


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.config = SafetyConfig(sign_dwell_s=1.0, sign_clear_rearm_s=0.5)
        self.safety = SafetySupervisor(self.config)

    def enable(self, now=1.0):
        self.assertTrue(self.safety.request_enable(now, fresh(now))[0])

    def tick(self, now, **changes):
        return self.safety.tick(now, fresh(now, **changes))

    def test_startup_stays_disabled_until_explicit_enable(self):
        self.assertFalse(self.tick(1.0).allow_motion)
        self.assertFalse(self.safety.request_enable(1.0, Inputs())[0])
        self.enable()
        self.assertTrue(self.tick(1.0).allow_motion)

    def test_each_stale_sensor_faults_and_does_not_auto_resume(self):
        for field in ("path_stamp", "odom_stamp", "perception_stamp"):
            with self.subTest(field=field):
                self.safety = SafetySupervisor(self.config)
                self.enable()
                decision = self.tick(2.0, **{field: 1.0})
                self.assertEqual(decision.state, "FAULT_STOP")
                self.assertFalse(decision.allow_motion)
                self.assertFalse(self.tick(2.1).allow_motion)
                self.enable(2.2)
                self.assertTrue(self.tick(2.2).allow_motion)

    def test_invalid_missing_and_future_inputs_are_rejected(self):
        cases = [{"path_valid": False}, {"perception_valid": False},
                 {"speed_mps": float("nan")}, {"path_stamp": None},
                 {"odom_stamp": 3.0}, {"perception_stamp": float("nan")}]
        for changes in cases:
            with self.subTest(changes=changes):
                self.safety = SafetySupervisor(self.config)
                self.enable()
                self.assertFalse(self.tick(1.0, **changes).allow_motion)
                self.assertFalse(self.safety.enabled)

    def test_sign_dwell_begins_only_at_measured_standstill(self):
        self.enable()
        self.assertEqual(self.tick(1.0, speed_mps=1.0,
                                   stop_sign_stop_required=True).state, "SIGN_STOP")
        self.assertEqual(self.tick(4.0, speed_mps=0.4).state, "SIGN_STOP")
        self.assertEqual(self.tick(4.1).state, "SIGN_DWELL")
        self.assertFalse(self.tick(4.9).allow_motion)
        self.assertTrue(self.tick(5.2).allow_motion)

    def test_motion_during_dwell_restarts_timer(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True)
        self.tick(1.7, speed_mps=0.2)
        self.tick(1.8)
        self.assertFalse(self.tick(2.1).allow_motion)
        self.assertTrue(self.tick(2.9).allow_motion)

    def test_same_sign_does_not_repeat_but_can_rearm_after_clear(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True)
        self.assertTrue(self.tick(2.1, stop_sign_stop_required=True).allow_motion)
        self.assertTrue(self.tick(3.0, stop_sign_stop_required=True).allow_motion)
        self.tick(3.1)
        self.tick(3.7)
        self.assertFalse(self.tick(3.8, stop_sign_stop_required=True).allow_motion)

    def test_brief_sign_dropout_does_not_rearm(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True)
        self.tick(2.1, stop_sign_stop_required=True)
        self.tick(2.2)
        self.assertTrue(self.tick(2.3, stop_sign_stop_required=True).allow_motion)

    def test_obstacle_has_priority_and_holds_after_sign_dwell(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True, obstacle_present=True)
        for now in (1.2, 2.1, 5.0):
            decision = self.tick(now, obstacle_present=True)
            self.assertEqual(decision.state, "OBSTACLE_STOP")
            self.assertFalse(decision.allow_motion)
        self.assertTrue(self.tick(5.1).allow_motion)

    def test_manual_release_requires_explicit_enable_and_preserves_stop(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True)
        self.safety.set_manual_takeover(True)
        self.assertEqual(self.tick(1.1).state, "MANUAL")
        self.assertFalse(self.safety.request_enable(1.1, fresh(1.1))[0])
        self.safety.set_manual_takeover(False)
        self.assertEqual(self.tick(2.0).state, "DISABLED")
        self.enable(2.1)
        self.assertFalse(self.tick(2.1).allow_motion)
        self.assertTrue(self.tick(3.2).allow_motion)

    def test_clock_rewind_latches_fault(self):
        self.enable(3.0)
        self.tick(3.1)
        self.assertEqual(self.tick(1.0).state, "FAULT_STOP")
        self.assertFalse(self.tick(1.1).allow_motion)

    def test_disable_during_stop_does_not_erase_required_dwell(self):
        self.enable()
        self.tick(1.0, stop_sign_stop_required=True)
        self.safety.disable()
        self.enable(5.0)
        self.assertFalse(self.tick(5.0).allow_motion)
        self.assertTrue(self.tick(6.1).allow_motion)


if __name__ == "__main__":
    unittest.main()
