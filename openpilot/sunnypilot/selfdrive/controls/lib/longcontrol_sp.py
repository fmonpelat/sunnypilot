"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState

STOP_ACCEL_TAPER_GAIN = 3.0  # 1/s
STOP_ACCEL_RAMP = 1.0  # m/s^3
STOP_HOLD_RAMP = 4.0  # m/s^3
STOP_HOLD_SPEED = 0.05  # m/s


class StopSmoother:
  def __init__(self, stop_accel: float):
    self.stop_accel = stop_accel
    self.last_accel = 0.0

  def reset(self, accel: float = 0.0) -> None:
    self.last_accel = accel

  def update(self, v_ego: float, standstill: bool, accel_limits: tuple[float, float]) -> float:
    v_ego = max(v_ego, 0.0)
    stopped = standstill or v_ego <= STOP_HOLD_SPEED

    if self.stop_accel < 0.0:
      target_accel = self.stop_accel if stopped else max(self.stop_accel, -STOP_ACCEL_TAPER_GAIN * v_ego)
    else:
      target_accel = min(self.last_accel, 0.0)

    ramp_rate = STOP_HOLD_RAMP if stopped else STOP_ACCEL_RAMP
    max_change = ramp_rate * DT_CTRL
    self.last_accel = float(np.clip(self.last_accel + np.clip(target_accel - self.last_accel, -max_change, max_change),
                                    accel_limits[0], accel_limits[1]))
    return self.last_accel


class LongControlSP(LongControl):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.stop_smoother = StopSmoother(CP.stopAccel)

  def update(self, active, CS, a_target, should_stop, accel_limits):
    output_accel = super().update(active, CS, a_target, should_stop, accel_limits)

    if self.long_control_state == LongCtrlState.stopping:
      output_accel = self.stop_smoother.update(CS.vEgo, CS.standstill, accel_limits)
      self.last_output_accel = output_accel
    else:
      self.stop_smoother.reset(output_accel)

    return float(output_accel)
