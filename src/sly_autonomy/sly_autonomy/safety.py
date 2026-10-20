"""Deterministic simulation stop supervisor; no actuator or ROS dependencies.

Zero target speed is a simulator request, not a verified physical brake.
Faults and manual takeover latch disabled until an explicit enable succeeds.
"""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class SafetyConfig:
    input_timeout_s: float = 0.5
    future_stamp_tolerance_s: float = 0.05
    standstill_speed_mps: float = 0.05
    sign_dwell_s: float = 3.0
    sign_clear_rearm_s: float = 2.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class Inputs:
    path_stamp: Optional[float] = None
    odom_stamp: Optional[float] = None
    perception_stamp: Optional[float] = None
    path_valid: bool = False
    perception_valid: bool = False
    speed_mps: float = float("nan")
    obstacle_present: bool = True
    stop_sign_stop_required: bool = False


@dataclass(frozen=True)
class Decision:
    state: str
    allow_motion: bool
    reason: str


class SafetySupervisor:
    def __init__(self, config: SafetyConfig):
        self.config = config
        self.enabled = False
        self.manual_takeover = False
        self.fault_reason = ""
        self.sign_armed = True
        self.sign_pending = False
        self._dwell_start = None
        self._clear_start = None
        self._last_tick = None

    def invalid_reason(self, now: float, inputs: Inputs) -> str:
        if not math.isfinite(now):
            return "invalid clock"
        for name in ("path", "odom", "perception"):
            stamp = getattr(inputs, f"{name}_stamp")
            if stamp is None or not math.isfinite(stamp):
                return f"missing or invalid {name} timestamp"
            if now - stamp > self.config.input_timeout_s:
                return f"stale {name}"
            if stamp - now > self.config.future_stamp_tolerance_s:
                return f"future {name} timestamp"
        if not inputs.path_valid:
            return "invalid local path"
        if not inputs.perception_valid:
            return "invalid perception"
        if not math.isfinite(inputs.speed_mps):
            return "invalid measured speed"
        return ""

    def request_enable(self, now: float, inputs: Inputs) -> tuple[bool, str]:
        if self.manual_takeover:
            return False, "manual takeover remains active"
        reason = self.invalid_reason(now, inputs)
        if reason:
            return False, reason
        self.enabled = True
        self.fault_reason = ""
        self._last_tick = now
        return True, "enabled for simulation; stop conditions still have priority"

    def disable(self):
        self.enabled = False
        self.fault_reason = ""
        self._dwell_start = None
        self._clear_start = None

    def set_manual_takeover(self, active: bool):
        self.manual_takeover = active
        if active:
            self.disable()

    def _fault(self, reason: str) -> Decision:
        self.enabled = False
        self.fault_reason = reason
        self._dwell_start = None
        self._clear_start = None
        return Decision("FAULT_STOP", False, reason)

    def tick(self, now: float, inputs: Inputs) -> Decision:
        if self.manual_takeover:
            return Decision("MANUAL", False, "explicit enable required after release")
        if not self.enabled:
            state = "FAULT_STOP" if self.fault_reason else "DISABLED"
            return Decision(state, False, self.fault_reason or "explicit enable required")
        reason = self.invalid_reason(now, inputs)
        if reason:
            return self._fault(reason)
        if self._last_tick is not None and now < self._last_tick:
            return self._fault("clock moved backwards")
        self._last_tick = now

        if inputs.stop_sign_stop_required:
            self._clear_start = None
            if self.sign_armed:
                self.sign_pending = True
                self.sign_armed = False
        elif not self.sign_pending:
            if self._clear_start is None:
                self._clear_start = now
            if now - self._clear_start >= self.config.sign_clear_rearm_s:
                self.sign_armed = True

        sign_state = "SIGN_STOP"
        if self.sign_pending:
            if abs(inputs.speed_mps) <= self.config.standstill_speed_mps:
                if self._dwell_start is None:
                    self._dwell_start = now
                sign_state = "SIGN_DWELL"
                if now - self._dwell_start >= self.config.sign_dwell_s:
                    self.sign_pending = False
                    self._dwell_start = None
            else:
                # Dwell must be continuous standstill, never elapsed travel time.
                self._dwell_start = None

        if inputs.obstacle_present:
            return Decision("OBSTACLE_STOP", False, "path occupied; hold until clear")
        if self.sign_pending:
            return Decision(sign_state, False, "stop sign; measured standstill then dwell")
        return Decision("CRUISE", True, "fresh valid inputs and no active stop condition")
