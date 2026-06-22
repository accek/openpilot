import pytest

from openpilot.common.utils import is_metered


class TestIsMetered:
  # ACSPilot: is_metered() backs both the loggerd upload gate (UploadOnMetered) and the updated.py fetch
  # gate (UpdateOnMetered). A connection is only treated as metered when the network reports metered AND
  # the user has NOT opted in to force-allow transfers on metered connections.
  @pytest.mark.parametrize("network_metered, allow_override, expected", [
    (True, False, True),    # metered network, no override -> throttle/skip
    (True, True, False),    # metered network, user force-allows -> not metered
    (False, False, False),  # unmetered network -> never metered
    (False, True, False),   # unmetered network, override irrelevant
  ])
  def test_truth_table(self, network_metered, allow_override, expected):
    assert is_metered(network_metered, allow_override) is expected
