import pytest

from openpilot.common.params import Params
from openpilot.sunnypilot.system.params_migration import run_migration


class TestOnroadBrightnessValidation:
  # ACSPilot: out-of-range OnroadScreenOffBrightness (e.g. 50 from older presets) is reset to Auto.
  @pytest.fixture(autouse=True)
  def _migrated(self):
    params = Params()
    params.put("OnroadScreenOffBrightnessMigrated", "1.0")
    params.put("OnroadScreenOffTimerMigrated", "1.0")
    yield params

  @pytest.mark.parametrize("value, expected", [(50, 0), (23, 0), (-1, 0), (0, 0), (1, 1), (2, 2), (22, 22)])
  def test_reset_out_of_range(self, _migrated, value, expected):
    _migrated.put("OnroadScreenOffBrightness", value)
    run_migration(_migrated)
    assert _migrated.get("OnroadScreenOffBrightness") == expected
