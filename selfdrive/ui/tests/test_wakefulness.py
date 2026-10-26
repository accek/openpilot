import pytest

from openpilot.selfdrive.ui import ui_state as ui_state_mod
from openpilot.selfdrive.ui.sunnypilot.ui_state import DeviceSP
from openpilot.selfdrive.ui.ui_state import Device, ui_state
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.lib.application import gui_app


# ACSPilot: ignition alone no longer keeps the screen awake, so the device times out while the preset picker
# waits at boot. These tests pin the wake/sleep decisions of Device._update_wakefulness around that.
class TestWakefulness:
  @pytest.fixture(autouse=True)
  def _isolate(self, monkeypatch):
    monkeypatch.setattr(ui_state_mod, "PC", False)
    monkeypatch.setattr(HARDWARE, "set_display_power", lambda on: None)
    monkeypatch.setattr(gui_app, "set_should_render", lambda on: None)
    monkeypatch.setattr(type(gui_app), "mouse_events", property(lambda self: []))
    monkeypatch.setattr(gui_app, "sunnypilot_ui", lambda: False)
    monkeypatch.setattr(ui_state, "ignition", False)
    monkeypatch.setattr(ui_state, "started", False)
    monkeypatch.setattr(ui_state, "boot_offroad_mode", 0)

  def _device(self, ignition: bool) -> Device:
    device = Device()
    device._ignition = ignition
    device._reset_interactive_timeout()
    return device

  @staticmethod
  def _expire(device: Device):
    device._interaction_time = 0.0

  def test_onroad_never_times_out(self):
    ui_state.ignition = ui_state.started = True
    device = self._device(ignition=True)
    self._expire(device)
    device._update_wakefulness()
    assert device.awake

  def test_ignition_on_without_preset_times_out(self):
    ui_state.ignition, ui_state.started = True, False
    device = self._device(ignition=True)
    device._update_wakefulness()
    assert device.awake
    self._expire(device)
    device._update_wakefulness()
    assert not device.awake

  def test_ignition_on_wakes_a_timed_out_screen(self):
    # device went to sleep offroad; the car is then started: the preset picker must be visible
    ui_state.ignition, ui_state.started = False, False
    device = self._device(ignition=False)
    self._expire(device)
    device._update_wakefulness()
    assert not device.awake
    ui_state.ignition = True
    device._update_wakefulness()
    assert device.awake

  def test_ignition_off_wakes_screen(self):
    ui_state.ignition, ui_state.started = True, False
    device = self._device(ignition=True)
    self._expire(device)
    device._update_wakefulness()
    assert not device.awake
    ui_state.ignition = False
    device._update_wakefulness()
    assert device.awake


class TestOffroadWakeMode:
  # "Offroad" wake mode flips Offroad Mode on when the device sleeps parked, not when the screen merely
  # times out with the ignition on while no preset is picked (the car would then stay offroad after picking one)
  class _State:
    def __init__(self, ignition):
      self.boot_offroad_mode = 1
      self.ignition = ignition
      self.puts = []
      self.params = self

    def put_bool(self, key, value):
      self.puts.append((key, value))

  def test_sleep_with_ignition_off_sets_offroad_mode(self):
    s = self._State(ignition=False)
    DeviceSP._set_awake(False, s)
    assert s.puts == [("OffroadMode", True)]

  def test_sleep_with_ignition_on_keeps_offroad_mode(self):
    s = self._State(ignition=True)
    DeviceSP._set_awake(False, s)
    assert s.puts == []

  def test_wake_never_sets_offroad_mode(self):
    s = self._State(ignition=False)
    DeviceSP._set_awake(True, s)
    assert s.puts == []
