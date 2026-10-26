import pytest

from openpilot.selfdrive.ui.ui_state import screen_brightness_to_light_sensor, car_light_sensor_to_brightness, CAR_BRIGHTNESS_MIN


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


class TestCarLightSensorToBrightness:
  # ACSPilot: with the car's dimmer as the source, the backlight follows the CIE curve scaled to 6%-100%
  # (as in the old fork), not the stock [30, 100] stretch that keeps the screen bright at night.
  @pytest.mark.parametrize("light_sensor, expected", [
    (0.0, CAR_BRIGHTNESS_MIN),
    (10.0, CAR_BRIGHTNESS_MIN),
    (30.0, 6.24),
    (50.0, 18.42),
    (75.0, 48.28),
    (100.0, 100.0),
  ])
  def test_mapping(self, light_sensor, expected):
    assert car_light_sensor_to_brightness(light_sensor) == pytest.approx(expected, abs=0.01)

  def test_monotonic_and_bounded(self):
    values = [car_light_sensor_to_brightness(x) for x in range(101)]
    assert values == sorted(values)
    assert min(values) == CAR_BRIGHTNESS_MIN
    assert max(values) == 100.0

  def test_night_dimmer_stays_below_stock_floor(self):
    # a typical night dimmer setting must not be lifted to the stock 30% minimum
    assert car_light_sensor_to_brightness(30.0) < 10.0
