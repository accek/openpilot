import pytest

from cereal import car, custom, car_custom
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import VCruiseHelper

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


class TestACCruiseExtensions:
  """ACSPilot: car-param-driven ACC setpoint controls (replaces the sunnypilot custom-increment feature)."""

  def _make_helper(self, **ac_kwargs):
    CP = car.CarParams(pcmCruise=False)
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    CP_AC = car_custom.CarParamsAC.new_message(**ac_kwargs)
    helper = VCruiseHelper(CP, CP_SP, CP_AC)
    # clear any previous cruise speed
    for _ in range(2):
      helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=True)
    return helper

  def _set_initial(self, helper, v_cruise_kph):
    helper.v_cruise_kph = v_cruise_kph
    helper.v_cruise_cluster_kph = v_cruise_kph

  def _press_short(self, helper, button_type, enabled=True):
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    helper.update_v_cruise(CS, enabled=enabled, is_metric=True)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    helper.update_v_cruise(CS, enabled=enabled, is_metric=True)

  def _press_long(self, helper, button_type, enabled=True):
    CS = car.CarState(cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=True)]
    helper.update_v_cruise(CS, enabled=enabled, is_metric=True)
    CS.buttonEvents = []
    for _ in range(50):
      helper.update_v_cruise(CS, enabled=enabled, is_metric=True)
    CS.buttonEvents = [ButtonEvent(type=button_type, pressed=False)]
    helper.update_v_cruise(CS, enabled=enabled, is_metric=True)

  def test_short_press_is_unit_increment(self):
    helper = self._make_helper()
    self._set_initial(helper, 50)
    self._press_short(helper, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 51

  @pytest.mark.parametrize("large_step", [5, 10])
  def test_long_press_uses_cruise_large_step(self, large_step):
    helper = self._make_helper(cruiseLargeStep=large_step)
    self._set_initial(helper, 50)  # multiple of both steps, so no interval rounding
    self._press_long(helper, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 50 + large_step

  def test_long_press_reverse_swaps_steps(self):
    # With cruiseLongPressReverse, a short press applies the large step and a long press the unit step
    helper = self._make_helper(cruiseLongPressReverse=True, cruiseLargeStep=5)
    self._set_initial(helper, 50)
    self._press_short(helper, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 55  # short press -> large step

    self._set_initial(helper, 50)
    self._press_long(helper, ButtonType.accelCruise)
    assert helper.v_cruise_kph == 51  # long press -> unit step

  def test_speed_settable_while_not_engaged(self):
    # ACSPilot: the driver can set/adjust the speed before activating ACC (cruise available, not engaged)
    helper = self._make_helper()
    self._set_initial(helper, 50)
    self._press_short(helper, ButtonType.accelCruise, enabled=False)
    assert helper.v_cruise_kph == 51

    self._press_short(helper, ButtonType.decelCruise, enabled=False)
    assert helper.v_cruise_kph == 50

  def test_set_button_sets_to_current_speed_while_not_engaged(self):
    helper = self._make_helper()
    self._set_initial(helper, 50)
    CS = car.CarState(vEgo=80 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.setCruise, pressed=True)]
    helper.update_v_cruise(CS, enabled=False, is_metric=True)
    CS.buttonEvents = [ButtonEvent(type=ButtonType.setCruise, pressed=False)]
    helper.update_v_cruise(CS, enabled=False, is_metric=True)
    assert helper.v_cruise_kph == pytest.approx(80, abs=0.5)
