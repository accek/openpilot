"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import pytest

from openpilot.sunnypilot.models import manager
from openpilot.sunnypilot.models.manager import ModelManagerSP


class StopLoop(BaseException):
  """Not an Exception subclass so main_thread's catch-all doesn't swallow it"""


class FakeParams:
  def __init__(self):
    self.store: dict = {}

  def get(self, key):
    return self.store.get(key)

  def put(self, key, value):
    self.store[key] = value

  def remove(self, key):
    self.store.pop(key, None)


class FakeRatekeeper:
  def __init__(self, rate, print_delay_threshold=None):
    self.frame = 0
    self.max_frames = 0

  def keep_time(self):
    self.frame += 1
    if self.frame >= self.max_frames:
      raise StopLoop


class FakePubMaster:
  def __init__(self, services):
    self.sent = 0

  def send(self, service, msg):
    self.sent += 1


class TestModelManagerLoop:
  @pytest.fixture(autouse=True)
  def setup(self, monkeypatch):
    self.params = FakeParams()
    self.rk = FakeRatekeeper(1)
    monkeypatch.setattr(manager, "Params", lambda: self.params)
    monkeypatch.setattr(manager, "Ratekeeper", lambda rate, print_delay_threshold=None: self.rk)
    monkeypatch.setattr(manager.messaging, "PubMaster", FakePubMaster)
    monkeypatch.setattr(manager, "get_active_bundle", lambda params: None)
    self.mm = ModelManagerSP()
    self.fetch_calls = 0

    def fake_get_available_bundles():
      self.fetch_calls += 1
      return []

    monkeypatch.setattr(self.mm.model_fetcher, "get_available_bundles", fake_get_available_bundles)

  def _run(self, ticks: int):
    self.rk.max_frames = ticks
    with pytest.raises(StopLoop):
      self.mm.main_thread()

  def test_bundle_cache_consulted_every_n_ticks(self):
    self._run(25)
    assert self.fetch_calls == 3  # frames 0, 10, 20
    assert self.mm.pm.sent == 25  # status still published every tick

  def test_pending_download_consults_cache_immediately(self):
    self.params.put("ModelManager_DownloadIndex", 42)  # no such bundle in the (empty) list: nothing to download
    self._run(5)
    assert self.fetch_calls == 5
