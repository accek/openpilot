import pytest

from openpilot.selfdrive.ui.onroad.augmented_road_view import wide_cam_enabled


class TestWideCamEnabled:
  # ACSPilot: WideRoadCameraInChillMode lets the wide camera be used outside experimental mode. The wide
  # stream is selected when (experimental OR chill-mode-option) AND the wide stream is available.
  @pytest.mark.parametrize("experimental, chill, wide_available, expected", [
    (False, False, True, False),   # neither trigger -> road cam
    (True, False, True, True),     # experimental mode -> wide (stock behavior)
    (False, True, True, True),     # ACSPilot: chill mode option -> wide
    (True, True, True, True),
    (True, False, False, False),   # wide not available -> never wide
    (False, True, False, False),   # ACSPilot option on but no wide stream -> road cam
  ])
  def test_condition(self, experimental, chill, wide_available, expected):
    assert wide_cam_enabled(experimental, chill, wide_available) is expected
