"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.sunnypilot.selfdrive.controls.lib.longcontrol_sp import LongControlSP, STOP_ACCEL_RAMP, STOP_ACCEL_TAPER_GAIN


class TestLongControlSP(OpenpilotTestCase):
  CP_SP = custom.CarParamsSP.new_message()

  def make_control(self, stop_accel=-1.5):
    CP = car.CarParams.new_message()
    CP.stopAccel = stop_accel
    CP.longitudinalTuning.kiBP = [0.0]
    CP.longitudinalTuning.kiV = [0.0]
    return LongControlSP(CP, self.CP_SP)

  @staticmethod
  def make_cs(v_ego, a_ego=0.0, brake_pressed=False, standstill=False):
    return SimpleNamespace(vEgo=v_ego, aEgo=a_ego, brakePressed=brake_pressed, standstill=standstill,
                           cruiseState=SimpleNamespace(standstill=standstill))

  def settle(self, loc, cs, frames=300, limits=(-2.0, 2.0)):
    for _ in range(frames):
      out = loc.update(True, cs, -0.5, should_stop=True, accel_limits=limits)
    return out

  def test_no_step_at_handover(self):
    loc = self.make_control()
    out = loc.update(True, self.make_cs(0.5, a_ego=-0.5), -0.5, should_stop=False, accel_limits=(-2.0, 2.0))
    assert loc.long_control_state == LongCtrlState.pid
    assert abs(out - -0.5) < 1e-6

    out = loc.update(True, self.make_cs(0.3, a_ego=-0.5), -0.5, should_stop=True, accel_limits=(-2.0, 2.0))
    assert loc.long_control_state == LongCtrlState.stopping
    assert abs(out - (-0.5 - STOP_ACCEL_RAMP * DT_CTRL)) < 1e-6

  def test_no_step_from_positive_accel(self):
    loc = self.make_control()
    loc.update(True, self.make_cs(0.4), 0.2, should_stop=False, accel_limits=(-2.0, 2.0))
    out = loc.update(True, self.make_cs(0.25), 0.2, should_stop=True, accel_limits=(-2.0, 2.0))
    assert loc.long_control_state == LongCtrlState.stopping
    assert abs(out - (0.2 - STOP_ACCEL_RAMP * DT_CTRL)) < 1e-6

  def test_decel_tapers_with_speed(self):
    for v_ego in (1.0, 0.3, 0.1):
      out = self.settle(self.make_control(), self.make_cs(v_ego))
      assert abs(out - max(-1.5, -STOP_ACCEL_TAPER_GAIN * v_ego)) < 1e-6

  def test_holds_at_standstill(self):
    assert abs(self.settle(self.make_control(), self.make_cs(0.0)) - -1.5) < 1e-6
    assert abs(self.settle(self.make_control(), self.make_cs(0.5, standstill=True)) - -1.5) < 1e-6

  def test_car_holds_itself(self):
    loc = self.make_control(stop_accel=0.0)
    loc.update(True, self.make_cs(0.5), -0.4, should_stop=False, accel_limits=(-2.0, 2.0))
    assert abs(self.settle(loc, self.make_cs(0.2)) - -0.4) < 1e-6

  def test_stop_change_is_jerk_limited(self):
    loc = self.make_control()
    v_ego = 0.3
    prev = loc.update(True, self.make_cs(v_ego), -0.5, should_stop=True, accel_limits=(-2.0, 2.0))
    for _ in range(100):
      out = loc.update(True, self.make_cs(v_ego), -0.5, should_stop=True, accel_limits=(-2.0, 2.0))
      assert abs(out - prev) <= STOP_ACCEL_RAMP * DT_CTRL + 1e-9
      prev = out

  def test_repeatable(self):
    runs = []
    for _ in range(2):
      loc = self.make_control()
      run = []
      for i in range(200):
        v_ego = max(1.0 - i * 0.01, 0.0)
        run.append(loc.update(True, self.make_cs(v_ego), -0.5, should_stop=v_ego < 0.3, accel_limits=(-2.0, 2.0)))
      runs.append(run)
    assert runs[0] == runs[1]
