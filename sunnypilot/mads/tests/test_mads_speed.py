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
