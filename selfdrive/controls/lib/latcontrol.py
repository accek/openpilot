import numpy as np
from abc import abstractmethod, ABC

from openpilot.common.realtime import DT_CTRL

MIN_LATERAL_CONTROL_SPEED = 0.3  # m/s


class LatControl(ABC):
  def __init__(self, CP, CI):
    self.sat_count_rate = 1.0 * DT_CTRL
    self.sat_limit = [CP.steerLimitTimer / 2., CP.steerLimitTimer]
    self.sat_count = [0., 0.]
    self.sat_check_min_speed = 10.

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, calibrated_pose, curvature_limited, model_data=None):
    pass

  def reset(self):
    self.sat_count = [0., 0.]

  def _check_saturating(self, saturating, CS):
    return self.__check_saturation(0, saturating, CS, False, False)

  def _check_saturated(self, saturated, CS, steer_limited_by_controls, curvature_limited):
    return self.__check_saturation(1, saturated, CS, steer_limited_by_controls, curvature_limited)

  def __check_saturation(self, key, saturation, CS, steer_limited_by_controls, curvature_limited):
    if (saturation or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_controls and not CS.steeringPressed:
      self.sat_count[key] += self.sat_count_rate
    else:
      self.sat_count[key] -= self.sat_count_rate
    self.sat_count[key] = np.clip(self.sat_count[key], 0.0, self.sat_limit[key])
    return self.sat_count[key] > (self.sat_limit[key] - 1e-3)
