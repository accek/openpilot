"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import pytest
from requests.exceptions import RequestException

from openpilot.sunnypilot.models import fetcher
from openpilot.sunnypilot.models.fetcher import ModelFetcher


class FakeParams:
  def __init__(self):
    self.store: dict = {}

  def get(self, key):
    return self.store.get(key)

  def put(self, key, value):
    self.store[key] = value


class FakeTime:
  def __init__(self):
    self.now = 1000.0

  def monotonic(self) -> float:
    return self.now

  def advance(self, seconds: float) -> None:
    self.now += seconds


class FakeResponse:
  status_code = 200

  def __init__(self, payload: dict):
    self._payload = payload

  def raise_for_status(self):
    pass

  def json(self):
    return self._payload


EMPTY_CACHE = {"bundles": []}


class TestModelFetcherBackoff:
  @pytest.fixture(autouse=True)
  def setup(self, monkeypatch):
    self.params = FakeParams()
    self.clock = FakeTime()
    monkeypatch.setattr(fetcher, "time", self.clock)
    self.get_calls: list[str] = []
    self.get_result: Exception | FakeResponse = RequestException("offline")

    def fake_get(url, timeout):
      self.get_calls.append(url)
      if isinstance(self.get_result, Exception):
        raise self.get_result
      return self.get_result

    monkeypatch.setattr(fetcher.requests, "get", fake_get)
    self.fetcher = ModelFetcher(self.params)

  def _seed_expired_cache(self):
    self.params.put("ModelManager_ModelsCache", EMPTY_CACHE)
    self.params.put("ModelManager_LastSyncTime", 0)

  def test_backoff_schedule(self):
    expected = [0, 60, 120, 240, 480, 900, 900, 900]
    for failures, wait in enumerate(expected):
      self.fetcher._fetch_failures = failures
      assert self.fetcher.fetch_backoff() == wait

  def test_failed_fetch_is_not_retried_every_tick(self):
    self._seed_expired_cache()

    # first tick with an expired cache attempts a fetch, which fails
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 1

    # the 1 Hz caller keeps polling; nothing should hit the network until the backoff has passed
    for _ in range(59):
      self.clock.advance(1)
      self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 1

    self.clock.advance(1)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 2

    # second failure doubles the wait
    self.clock.advance(119)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 2
    self.clock.advance(1)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 3

  def test_backoff_is_capped(self):
    self._seed_expired_cache()
    for _ in range(10):
      self.fetcher.get_available_bundles()
      self.clock.advance(self.fetcher.FETCH_BACKOFF_MAX_S)
    assert self.fetcher.fetch_backoff() == self.fetcher.FETCH_BACKOFF_MAX_S
    # one attempt per cap interval, no more
    assert len(self.get_calls) == 10

  def test_no_cache_still_backs_off(self):
    self.fetcher.get_available_bundles()
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 1

  def test_successful_fetch_resets_backoff_and_refreshes_cache(self):
    self._seed_expired_cache()
    self.fetcher.get_available_bundles()
    assert self.fetcher._fetch_failures == 1

    self.clock.advance(60)
    self.get_result = FakeResponse(EMPTY_CACHE)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 2
    assert self.fetcher._fetch_failures == 0
    assert self.params.get("ModelManager_LastSyncTime") == int(self.clock.now * 1e9)

    # cache is now fresh: no further network access for the cache lifetime
    self.clock.advance(3599)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 2
    self.clock.advance(1)
    self.fetcher.get_available_bundles()
    assert len(self.get_calls) == 3

  def test_parse_is_memoized_until_invalidated(self, monkeypatch):
    parse_calls = []
    real_parse = self.fetcher.model_parser.parse_models

    def counting_parse(data):
      parse_calls.append(data)
      return real_parse(data)

    monkeypatch.setattr(self.fetcher.model_parser, "parse_models", counting_parse)
    self.params.put("ModelManager_ModelsCache", EMPTY_CACHE)
    self.params.put("ModelManager_LastSyncTime", int(self.clock.now * 1e9))

    first = self.fetcher.get_available_bundles()
    second = self.fetcher.get_available_bundles()
    assert first is second
    assert len(parse_calls) == 1

    self.fetcher.invalidate()
    third = self.fetcher.get_available_bundles()
    assert third is not first
    assert len(parse_calls) == 2

    # changed cache contents are re-parsed
    self.params.put("ModelManager_ModelsCache", {"bundles": [], "v": 2})
    self.fetcher.get_available_bundles()
    assert len(parse_calls) == 3
