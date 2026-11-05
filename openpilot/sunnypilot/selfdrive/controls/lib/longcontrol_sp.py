"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState

STOP_JERK = 0.25
TAPER_MAX_DECEL = 1.0
STOP_LEAD_TIME = 0.4
STOP_STALL_TIME = 1.5
STOP_RELEASE_DEBOUNCE = 0.2
MAX_JERK = 2.0


def taper_decel(v_ego: float) -> float:
  return float(np.sqrt(2.0 * STOP_JERK * max(v_ego, 0.0)))


class StopSmoother:
  def __init__(self, stop_accel: float):
    self.stop_accel = stop_accel
    self.last_accel = 0.0
    self.holding = False
    self.stall_time = 0.0

  def reset(self, accel: float = 0.0) -> None:
    self.last_accel = accel
    self.holding = False
    self.stall_time = 0.0

  def update(self, v_ego: float, a_ego: float, standstill: bool, accel_limits: tuple[float, float]) -> float:
    self.holding = self.holding or standstill

    if self.stop_accel >= 0.0:
      target_accel = min(self.last_accel, 0.0)
    elif self.holding:
      target_accel = self.stop_accel
    else:
      self.stall_time += DT_CTRL
      stall_decel = STOP_JERK * (self.stall_time - STOP_STALL_TIME)
      v_lead = v_ego + min(a_ego, 0.0) * STOP_LEAD_TIME
      target_accel = max(self.stop_accel, -max(taper_decel(v_lead), stall_decel))

    max_change = MAX_JERK * DT_CTRL
    accel = self.last_accel + np.clip(target_accel - self.last_accel, -max_change, max_change)
    self.last_accel = float(np.clip(accel, *accel_limits))
    return self.last_accel


class LongControlSP(LongControl):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.stop_smoother = StopSmoother(CP.stopAccel)
    self.release_time = 0.0
    self.was_tapering = False

  def debounce_should_stop(self, CS, should_stop: bool) -> bool:
    if self.stop_smoother.holding and CS.standstill and not should_stop:
      self.release_time += DT_CTRL
      return self.release_time < STOP_RELEASE_DEBOUNCE
    self.release_time = 0.0
    return should_stop

  def update(self, active, CS, a_target, should_stop, accel_limits):
    prev_output = self.last_output_accel

    tapering = False
    if -TAPER_MAX_DECEL < a_target < 0.0:
      tapered = max(a_target, -taper_decel(CS.vEgo))
      tapering = tapered != a_target
      a_target = tapered

    should_stop = self.debounce_should_stop(CS, should_stop)
    output_accel = super().update(active, CS, a_target, should_stop, accel_limits)

    if self.long_control_state == LongCtrlState.stopping:
      output_accel = self.stop_smoother.update(CS.vEgo, CS.aEgo, CS.standstill, accel_limits)
    else:
      if self.long_control_state == LongCtrlState.pid and (tapering or self.was_tapering):
        max_change = MAX_JERK * DT_CTRL
        output_accel = float(np.clip(output_accel, prev_output - max_change, prev_output + max_change))
      self.stop_smoother.reset(output_accel)

    self.last_output_accel = output_accel
    self.was_tapering = tapering
    return float(output_accel)
