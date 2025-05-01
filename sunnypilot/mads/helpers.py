"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from typing import Optional

from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV

from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP


class MadsParams:
  def __init__(self):
    self.params = Params()

  def read_param(self, key: str):
    return self.params.get_bool(key)

  def read_speed_param(self, enable_key: str, speed_key: str, default: float, is_metric: bool = True) -> Optional[float]:
    enable = self.read_param(enable_key)
    if not enable:
      return None
    speed = float(self.params.get(speed_key) or 0)
    if speed > 0.0:
      return speed * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    return default

  def set_alternative_experience(self, CP):
    enabled = self.read_param("Mads")
    pause_lateral_on_brake = self.read_param("MadsPauseLateralOnBrake")

    if enabled:
      CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS

      if pause_lateral_on_brake:
        CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISENGAGE_LATERAL_ON_BRAKE

  def set_car_specific_params(self, CP, CP_SP):
    if CP.brand == "hyundai":
      # TODO-SP: This should be separated from MADS module for future implementations
      #          Use "HyundaiLongitudinalMainCruiseToggleable" param
      hyundai_cruise_main_toggleable = True
      if hyundai_cruise_main_toggleable:
        CP_SP.flags |= HyundaiFlagsSP.LONGITUDINAL_MAIN_CRUISE_TOGGLEABLE.value
        CP_SP.safetyParam |= HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE

    # MADS is currently not supported in Tesla due to lack of consistent states to engage controls
    # TODO-SP: To enable MADS for Tesla, identify consistent signals for MADS toggling
    if CP.brand == "tesla":
      self.params.remove("Mads")

    # MADS is currently not supported in Rivian due to lack of consistent states to engage controls
    # TODO-SP: To enable MADS for Rivian, identify consistent signals for MADS toggling
    if CP.brand == "rivian":
      self.params.remove("Mads")
