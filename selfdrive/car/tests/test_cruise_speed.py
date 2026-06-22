import pytest
import itertools
import numpy as np

from openpilot.common.parameterized import parameterized_class
from cereal import log
from openpilot.selfdrive.car.cruise import VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_INITIAL, IMPERIAL_INCREMENT
from cereal import car, custom, car_custom
from openpilot.common.constants import CV
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type


def run_cruise_simulation(cruise, e2e, personality, t_end=20.):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=max(cruise - 1., 0.0),
    lead_relevancy=True,
    initial_distance_lead=100,
    cruise_values=[cruise],
    prob_lead_values=[0.0],
    breakpoints=[0.],
    e2e=e2e,
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1, 3]


@parameterized_class(("e2e", "personality", "speed"), itertools.product(
                      [True, False], # e2e
                      log.LongitudinalPersonality.schema.enumerants, # personality
                      [5,35])) # speed
class TestCruiseSpeed:
  def test_cruise_speed(self):
    print(f'Testing {self.speed} m/s')
    cruise_speed = float(self.speed)

    simulation_steady_state = run_cruise_simulation(cruise_speed, self.e2e, self.personality)
    assert simulation_steady_state == pytest.approx(cruise_speed, abs=.01), f'Did not reach {self.speed} m/s'


# TODO: test pcmCruise and pcmCruiseSpeed
@parameterized_class(('pcm_cruise', 'pcm_cruise_speed'), [(False, True)])
class TestVCruiseHelper:
  def setup_method(self):
    self.CP = car.CarParams(pcmCruise=self.pcm_cruise)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=self.pcm_cruise_speed)
    self.CP_AC = car_custom.CarParamsAC.new_message()
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP, self.CP_AC)
    self.reset_cruise_speed_state()

  def reset_cruise_speed_state(self):
    # Two resets previous cruise speed
    for _ in range(2):
      self.v_cruise_helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)

  def enable(self, v_ego, experimental_mode, dynamic_experimental_control):
    # Simulates user pressing set with a current speed
    self.v_cruise_helper.initialize_v_cruise(car.CarState(vEgo=v_ego), experimental_mode, dynamic_experimental_control)

  def test_adjust_speed(self):
    """
    Asserts speed changes on falling edges of buttons.
    """

    self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

    for btn in (ButtonType.accelCruise, ButtonType.decelCruise):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True})
        CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]

        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)
        assert pressed == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_rising_edge_enable(self):
    """
    Some car interfaces may enable on rising edge of a button,
    ensure we don't adjust speed if enabled changes mid-press.
    """

    # NOTE: enabled is always one frame behind the result from button press in controlsd
    for enabled, pressed in ((False, False),
                             (False, True),
                             (True, False)):
      CS = car.CarState(cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=pressed)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
      if pressed:
        self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # Expected diff on enabling. Speed should not change on falling edge of pressed
      assert not pressed == self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last

  def test_resume_in_standstill(self):
    """
    Asserts we don't increment set speed if user presses resume/accel to exit cruise standstill.
    """

    self.enable(0, False, False)

    for standstill in (True, False):
      for pressed in (True, False):
        CS = car.CarState(cruiseState={"available": True, "standstill": standstill})
        CS.buttonEvents = [ButtonEvent(type=ButtonType.accelCruise, pressed=pressed)]
        self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

        # speed should only update if not at standstill and button falling edge
        should_equal = standstill or pressed
        assert should_equal == (self.v_cruise_helper.v_cruise_kph == self.v_cruise_helper.v_cruise_kph_last)

  def test_set_gas_pressed(self):
    """
    Asserts pressing set while enabled with gas pressed sets
    the speed to the maximum of vEgo and current cruise speed.
    """

    for v_ego in np.linspace(0, 100, 101):
      self.reset_cruise_speed_state()
      self.enable(V_CRUISE_INITIAL * CV.KPH_TO_MS, False, False)

      # first decrement speed, then perform gas pressed logic
      expected_v_cruise_kph = self.v_cruise_helper.v_cruise_kph - IMPERIAL_INCREMENT
      expected_v_cruise_kph = max(expected_v_cruise_kph, v_ego * CV.MS_TO_KPH)  # clip to min of vEgo
      expected_v_cruise_kph = float(np.clip(round(expected_v_cruise_kph, 1), V_CRUISE_MIN, V_CRUISE_MAX))

      CS = car.CarState(vEgo=float(v_ego), gasPressed=True, cruiseState={"available": True})
      CS.buttonEvents = [ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
      self.v_cruise_helper.update_v_cruise(CS, enabled=True, is_metric=False)

      # TODO: fix skipping first run due to enabled on rising edge exception
      if v_ego == 0.0:
        continue
      assert expected_v_cruise_kph == self.v_cruise_helper.v_cruise_kph

  def test_initialize_v_cruise(self):
    """
    Asserts allowed cruise speeds on enabling with SET.
    """

    for experimental_mode in (True, False):
      for dynamic_experimental_control in (True, False):
        for v_ego in np.linspace(0, 100, 101):
          self.reset_cruise_speed_state()
          assert not self.v_cruise_helper.v_cruise_initialized

          self.enable(float(v_ego), experimental_mode, dynamic_experimental_control)
          assert V_CRUISE_INITIAL <= self.v_cruise_helper.v_cruise_kph <= V_CRUISE_MAX
          assert self.v_cruise_helper.v_cruise_initialized


class TestVCruiseHelperAC:
  # ACSPilot: VCruiseHelper takes a CarParamsAC and adds cruise-control extensions configured by it:
  #  - SET button sets the cruise speed to the current vEgo
  #  - accelButtonResumesCruise: treat ACCEL like RESUME at standstill (don't bump set speed)
  #  - decelButtonLimitedToVEgoWhenOverriding: only clip DECEL up to vEgo while overriding when enabled
  def _helper(self, **ac_kwargs):
    CP = car.CarParams(pcmCruise=False)
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=True)
    CP_AC = car_custom.CarParamsAC.new_message(**ac_kwargs)
    helper = VCruiseHelper(CP, CP_SP, CP_AC)
    for _ in range(2):  # two resets clear previous cruise speed (mirrors TestVCruiseHelper)
      helper.update_v_cruise(car.CarState(cruiseState={"available": False}), enabled=False, is_metric=False)
    return helper

  @staticmethod
  def _press_release(helper, btn, v_ego=0.0, gas=False, standstill=False):
    for pressed in (True, False):
      CS = car.CarState(cruiseState={"available": True, "standstill": standstill}, vEgo=float(v_ego), gasPressed=gas)
      CS.buttonEvents = [ButtonEvent(type=btn, pressed=pressed)]
      helper.update_v_cruise(CS, enabled=True, is_metric=False)

  def test_set_button_sets_cruise_to_vego(self):
    helper = self._helper()
    helper.initialize_v_cruise(car.CarState(vEgo=10.0), False, False)
    self._press_release(helper, ButtonType.setCruise, v_ego=20.0)
    assert helper.v_cruise_kph == pytest.approx(20.0 * CV.MS_TO_KPH, abs=0.5)

  def test_resume_buttons_reflects_accel_resumes_flag(self):
    # accelButtonResumesCruise adds RESUME to the set of buttons that silently exit standstill;
    # ACCEL is always in that set (stock behavior)
    on = self._helper(accelButtonResumesCruise=True).resume_buttons
    off = self._helper(accelButtonResumesCruise=False).resume_buttons
    assert ButtonType.accelCruise in on and ButtonType.accelCruise in off
    assert ButtonType.resumeCruise in on
    assert ButtonType.resumeCruise not in off

  @pytest.mark.parametrize("limited, expect_clipped_to_vego", [(True, True), (False, False)])
  def test_decel_overriding_clip_gated_by_flag(self, limited, expect_clipped_to_vego):
    helper = self._helper(decelButtonLimitedToVEgoWhenOverriding=limited)
    helper.initialize_v_cruise(car.CarState(vEgo=5.0), False, False)
    self._press_release(helper, ButtonType.decelCruise, v_ego=20.0, gas=True)
    clipped = helper.v_cruise_kph >= 20.0 * CV.MS_TO_KPH - 0.5
    assert clipped == expect_clipped_to_vego
