"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import pytest

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import read_speed_param, MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

ENABLE_KEY = "MadsPauseSpeedEnabled"
SPEED_KEY = "MadsPauseSpeed"
DEFAULT = 4.4  # arbitrary default in m/s


class TestReadSpeedParam:
  # ACSPilot: read_speed_param converts a configured speed threshold (stored in display units) to m/s for
  # the MADS speed-based pause/resume feature, or returns None when the feature toggle is off.
  def test_disabled_returns_none(self):
    params = Params()
    params.put_bool(ENABLE_KEY, False)
    params.put(SPEED_KEY, 30)
    assert read_speed_param(params, ENABLE_KEY, SPEED_KEY, DEFAULT) is None

  def test_zero_speed_falls_back_to_default(self):
    params = Params()
    params.put_bool(ENABLE_KEY, True)
    params.put(SPEED_KEY, 0)
    assert read_speed_param(params, ENABLE_KEY, SPEED_KEY, DEFAULT) == DEFAULT

  def test_metric_converts_kph_to_ms(self):
    params = Params()
    params.put_bool(ENABLE_KEY, True)
    params.put(SPEED_KEY, 30)
    got = read_speed_param(params, ENABLE_KEY, SPEED_KEY, DEFAULT, is_metric=True)
    assert got == pytest.approx(30 * CV.KPH_TO_MS)

  def test_imperial_converts_mph_to_ms(self):
    params = Params()
    params.put_bool(ENABLE_KEY, True)
    params.put(SPEED_KEY, 30)
    got = read_speed_param(params, ENABLE_KEY, SPEED_KEY, DEFAULT, is_metric=False)
    assert got == pytest.approx(30 * CV.MPH_TO_MS)

  def test_unset_speed_falls_back_to_default(self):
    params = Params()
    params.put_bool(ENABLE_KEY, True)
    # SPEED_KEY never written -> treated as 0 -> default
    assert read_speed_param(params, ENABLE_KEY, SPEED_KEY, DEFAULT) == DEFAULT


def _bare_mads():
  # construct a MADS without running __init__ (which needs a full selfdrive object); set only the
  # attributes should_silent_lkas_enable() reads so we can exercise the speed-pause gate in isolation.
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

  mads = ModularAssistiveDrivingSystem.__new__(ModularAssistiveDrivingSystem)
  mads.steering_mode_on_brake = MadsSteeringModeOnBrake.REMAIN_ACTIVE
  mads.events = Events()
  mads.events_sp = EventsSP()
  mads.speed_paused = False
  mads.disengage_on_accelerator = False
  return mads


def _carstate():
  import cereal.messaging as messaging
  return messaging.new_message('carState').carState


class TestSpeedPauseGate:
  # ACSPilot: while paused by a low-speed steering takeover, should_silent_lkas_enable() must return False
  # so the system does not silently re-enable lateral until the resume speed is reached.
  def test_silent_reenable_allowed_when_not_speed_paused(self):
    mads = _bare_mads()
    mads.speed_paused = False
    assert mads.should_silent_lkas_enable(_carstate()) is True

  def test_silent_reenable_blocked_while_speed_paused(self):
    mads = _bare_mads()
    mads.speed_paused = True
    assert mads.should_silent_lkas_enable(_carstate()) is False


def _mads_for_update_events(long_enabled, state):
  # bare MADS with just the attributes update_events() touches on the speed-pause release path
  from types import SimpleNamespace
  import cereal.messaging as messaging
  from openpilot.selfdrive.selfdrived.events import Events
  from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

  mads = ModularAssistiveDrivingSystem.__new__(ModularAssistiveDrivingSystem)
  mads.enabled = True
  mads.speed_paused = True
  mads.steering_mode_on_brake = MadsSteeringModeOnBrake.REMAIN_ACTIVE
  mads.main_enabled_toggle = False
  mads.no_main_cruise = False
  mads.allow_always = False
  mads.disengage_on_accelerator = False
  mads.pause_speed = 4.4
  mads.resume_speed = 18.0  # well above the test CS speed so the speed gate alone would NOT release
  mads.lateral_mismatch_counter = 0
  mads.events = Events()
  mads.events_sp = EventsSP()
  mads.state_machine = SimpleNamespace(state=state)
  cs_prev = messaging.new_message('carState').carState
  cs_prev.cruiseState.available = True
  mads.selfdrive = SimpleNamespace(enabled=long_enabled, enabled_prev=long_enabled, CS_prev=cs_prev)
  return mads


def _cs_available_stopped():
  import cereal.messaging as messaging
  cs = messaging.new_message('carState').carState
  cs.cruiseState.available = True
  cs.vEgoCluster = 0.0  # below resume_speed, so only longitudinal engagement can release the pause
  return cs


class TestSpeedPauseClearedWhenLongEngaged:
  # ACSPilot: a low-speed steering pause must never keep lateral paused while longitudinal (ACC) is
  # engaged. Even below the resume speed, engaging ACC releases the pause AND re-engages lateral: the
  # cleared latch lets should_silent_lkas_enable() emit silentLkasEnable (ET.ENABLE), which drives the
  # MADS state machine paused -> enabled.
  def test_long_engaged_releases_pause_and_reengages_lateral(self):
    from openpilot.sunnypilot.mads.mads import State, EventNameSP
    mads = _mads_for_update_events(long_enabled=True, state=State.paused)
    mads.update_events(_cs_available_stopped())
    assert mads.speed_paused is False
    assert mads.events_sp.has(EventNameSP.silentLkasEnable)  # re-engage requested

  def test_long_not_engaged_keeps_pause_and_does_not_reengage(self):
    from openpilot.sunnypilot.mads.mads import State, EventNameSP
    mads = _mads_for_update_events(long_enabled=False, state=State.paused)
    mads.update_events(_cs_available_stopped())
    assert mads.speed_paused is True  # below resume speed, ACC off -> stays paused
    assert not mads.events_sp.has(EventNameSP.silentLkasEnable)
