import pytest

from openpilot.selfdrive.ui.ui_state import screen_brightness_to_light_sensor


class TestScreenBrightnessToLightSensor:
  # ACSPilot: the car-reported (0..1) screen brightness maps linearly onto the 0..100 light-sensor scale,
  # clamped at full brightness for values >= 1.0.
  @pytest.mark.parametrize("screen_brightness, expected", [
    (0.0, 0.0),
    (0.25, 25.0),
    (0.5, 50.0),
    (1.0, 100.0),
    (1.5, 100.0),
    (10.0, 100.0),
  ])
  def test_mapping(self, screen_brightness, expected):
    assert screen_brightness_to_light_sensor(screen_brightness) == expected
