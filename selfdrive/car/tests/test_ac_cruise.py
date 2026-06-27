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

  def _engage(self, helper, engage_button, v_ego_kph, resume_sets_default):
    # Mirror card.py: update_v_cruise every frame, then initialize_v_cruise(CS_prev) on the enable edge.
    v_ego = v_ego_kph * CV.KPH_TO_MS
    cs_prev = car.CarState(vEgo=v_ego, cruiseState={"available": True})

    def step(buttons, enabled, prev_enabled):
      nonlocal cs_prev
      CS = car.CarState(vEgo=v_ego, cruiseState={"available": True})
      CS.buttonEvents = buttons
      helper.update_v_cruise(CS, enabled=enabled, is_metric=True)
      if enabled and not prev_enabled:
        helper.initialize_v_cruise(cs_prev, False, False)
      cs_prev = CS

    step([], False, False)
    step([ButtonEvent(type=engage_button, pressed=True)], False, False)   # press
    step([ButtonEvent(type=engage_button, pressed=False)], True, False)   # release -> enable edge

  @pytest.mark.parametrize("engage_button", [ButtonType.resumeCruise, ButtonType.setCruise, ButtonType.accelCruise])
  def test_resume_sets_default_engages_to_current_speed(self, engage_button):
    # ACSPilot: with resumeButtonSetsDefaultVCruise (VW MQB) engaging must set the set speed to the current
    # speed. Regression: the accel-button path used to restore the stale UNSET (255) sentinel instead, because
    # card re-runs initialize_v_cruise after _update_v_cruise_non_pcm already initialized this frame.
    helper = self._make_helper(resumeButtonSetsDefaultVCruise=True)
    self._engage(helper, engage_button, v_ego_kph=100, resume_sets_default=True)
    assert helper.v_cruise_kph == pytest.approx(100, abs=0.5)

  def test_resume_sets_default_uses_current_speed_on_reengage(self):
    # Re-engaging after a disengage (cruise still available, so v_cruise stays initialized at 100) must use
    # the current speed, not the previously stored setpoint. The resume button is present in CS, so without
    # the flag this would take the restore-previous branch and keep 100.
    helper = self._make_helper(resumeButtonSetsDefaultVCruise=True)
    self._engage(helper, ButtonType.resumeCruise, v_ego_kph=100, resume_sets_default=True)
    assert helper.v_cruise_kph == pytest.approx(100, abs=0.5)
    # disengage (cancel) without dropping cruise availability -> v_cruise stays initialized
    for pressed in (True, False):
      CS = car.CarState(vEgo=80 * CV.KPH_TO_MS, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.cancel, pressed=pressed)]
      helper.update_v_cruise(CS, enabled=False, is_metric=True)
    assert helper.v_cruise_initialized  # still 100, not reset
    # resume again at 80 km/h: the resume button is in CS, but the default-speed flag must win
    CS = car.CarState(vEgo=80 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.resumeCruise, pressed=True)]
    helper.initialize_v_cruise(CS, False, False)
    assert helper.v_cruise_kph == pytest.approx(80, abs=0.5)

  def test_resume_restores_previous_setpoint_by_default(self):
    # Without the flag (non-VW default) the stock behavior is preserved: resume restores the previous setpoint.
    helper = self._make_helper()  # resumeButtonSetsDefaultVCruise defaults to False
    self._set_initial(helper, 120)
    helper.v_cruise_kph_last = 120
    CS = car.CarState(vEgo=60 * CV.KPH_TO_MS, cruiseState={"available": True})
    CS.buttonEvents = [ButtonEvent(type=ButtonType.resumeCruise, pressed=True)]
    helper.initialize_v_cruise(CS, False, False)
    assert helper.v_cruise_kph == 120
