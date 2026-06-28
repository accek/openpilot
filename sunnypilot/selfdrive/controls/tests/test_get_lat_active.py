"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging
from cereal import custom

from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral


def _controls_ext(blinker_enabled=True):
  # build a ControlsExt without __init__ (which blocks on CarParamsSP); only get_lat_active state is set
  ce = ControlsExt.__new__(ControlsExt)
  bpl = BlinkerPauseLateral()
  bpl.enabled = blinker_enabled
  bpl.is_metric = False
  bpl.min_speed = 20  # MPH
  bpl.reengage_delay = 0
  bpl.blinker_off_timer = 0.0
  ce.blinker_pause_lateral = bpl
  return ce


def _sm(*, one_blinker, v_ego, long_enabled, mads_active, mads_available=True):
  cs = messaging.new_message('carState').carState
  cs.vEgo = v_ego
  cs.leftBlinker = one_blinker
  cs.rightBlinker = False

  ss = messaging.new_message('selfdriveState').selfdriveState
  ss.enabled = long_enabled
  ss.active = long_enabled

  ss_sp = custom.SelfdriveStateSP.new_message()
  ss_sp.mads.available = mads_available
  ss_sp.mads.active = mads_active

  return {'carState': cs, 'selfdriveState': ss, 'selfdriveStateSP': ss_sp}


class TestGetLatActiveBlinkerLongEngaged:
  # ACSPilot: the blinker pause must never stop lateral while longitudinal (ACC) is engaged.

  def test_blinker_pauses_lateral_when_long_not_engaged(self):
    ce = _controls_ext()
    sm = _sm(one_blinker=True, v_ego=4.5, long_enabled=False, mads_active=True)
    assert ce.get_lat_active(sm) is False  # blinker + low speed -> paused

  def test_blinker_does_not_pause_lateral_when_long_engaged(self):
    ce = _controls_ext()
    sm = _sm(one_blinker=True, v_ego=4.5, long_enabled=True, mads_active=True)
    assert ce.get_lat_active(sm) is True  # ACC engaged -> steering stays active despite blinker

  def test_no_blinker_uses_mads_state(self):
    ce = _controls_ext()
    sm = _sm(one_blinker=False, v_ego=4.5, long_enabled=False, mads_active=True)
    assert ce.get_lat_active(sm) is True
