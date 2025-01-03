from abc import abstractmethod, ABC

from openpilot.common.numpy_fast import clip
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
  def update(self, active, CS, VM, params, steer_limited, desired_curvature, llk, model_data=None):
    pass

  def reset(self):
    self.sat_count = [0., 0.]

  def _check_saturating(self, saturating, CS):
    return self.__check_saturation(0, saturating, CS, False)

  def _check_saturated(self, saturated, CS, steer_limited):
    return self.__check_saturation(1, saturated, CS, steer_limited)

  def __check_saturation(self, key, saturation, CS, steer_limited):
    if not steer_limited and not CS.steeringPressed:
      if saturation and CS.vEgo > self.sat_check_min_speed:
        self.sat_count[key] += self.sat_count_rate
      else:
        self.sat_count[key] -= self.sat_count_rate
      self.sat_count[key] = clip(self.sat_count[key], 0.0, self.sat_limit[key])
    return self.sat_count[key] > (self.sat_limit[key] - 1e-3)
