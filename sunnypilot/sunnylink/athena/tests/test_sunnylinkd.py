"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import threading

from openpilot.sunnypilot.sunnylink.athena import sunnylinkd


class TestSunnylinkdMethods:
  def setup_method(self):
    self.saved_params = []

    self.original_save = sunnylinkd.save_param_from_base64_encoded_string

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    sunnylinkd.save_param_from_base64_encoded_string = mock_save_param

  def teardown_method(self):
    sunnylinkd.save_param_from_base64_encoded_string = self.original_save

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "SpeedLimitOffset": "5",
      "MyCustomParam": "123"
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "SpeedLimitOffset" in keys_saved
    assert "MyCustomParam" in keys_saved

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "SpeedLimitOffset": "10"
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "SpeedLimitOffset"
    assert self.saved_params[0][1] == "10"


class TestSunnylinkdMainIdle:
  """main() must stay resident while sunnylink is disabled/unregistered instead of returning
  (manage_sunnylinkd would respawn a fresh interpreter every minute otherwise)."""

  def setup_method(self):
    self.exit_event = threading.Event()
    self.waits: list[float] = []
    self.ready = False
    self.need_register = False
    self.ws_loops = 0
    self.initialized = 0

  def _patches(self, monkeypatch, exit_after_waits: int):
    def fake_wait(exit_event, seconds):
      self.waits.append(seconds)
      if len(self.waits) >= exit_after_waits:
        self.exit_event.set()

    def fake_ws_loop(exit_event):
      self.ws_loops += 1
      self.exit_event.set()
      return False

    monkeypatch.setattr(sunnylinkd, "_wait", fake_wait)
    monkeypatch.setattr(sunnylinkd, "sunnylink_ready", lambda params: self.ready)
    monkeypatch.setattr(sunnylinkd, "sunnylink_need_register", lambda params: self.need_register)
    monkeypatch.setattr(sunnylinkd, "run_ws_loop", fake_ws_loop)
    monkeypatch.setattr(sunnylinkd, "set_core_affinity", lambda cores: None)
    def fake_initialize(queue):
      self.initialized += 1

    monkeypatch.setattr(sunnylinkd.UploadQueueCache, "initialize", fake_initialize)
    monkeypatch.setattr(sunnylinkd, "update_car_list_param", lambda: None)

  def test_disabled_sunnylink_idles_in_process(self, monkeypatch):
    self._patches(monkeypatch, exit_after_waits=3)
    sunnylinkd.main(self.exit_event)

    assert self.waits == [sunnylinkd.SUNNYLINK_NOT_READY_POLL_S] * 3
    assert self.ws_loops == 0
    assert self.initialized == 0

  def test_unregistered_sunnylink_polls_registration(self, monkeypatch):
    self.need_register = True
    self._patches(monkeypatch, exit_after_waits=2)
    sunnylinkd.main(self.exit_event)

    assert self.waits == [sunnylinkd.SUNNYLINK_REGISTRATION_POLL_S] * 2
    assert self.ws_loops == 0

  def test_connects_once_ready(self, monkeypatch):
    self._patches(monkeypatch, exit_after_waits=100)

    def fake_wait(exit_event, seconds):
      self.waits.append(seconds)
      if len(self.waits) == 2:
        self.ready = True

    monkeypatch.setattr(sunnylinkd, "_wait", fake_wait)
    sunnylinkd.main(self.exit_event)

    assert self.waits == [sunnylinkd.SUNNYLINK_NOT_READY_POLL_S] * 2
    assert self.ws_loops == 1
    assert self.initialized == 1
