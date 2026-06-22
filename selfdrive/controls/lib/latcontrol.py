import numpy as np
from abc import abstractmethod, ABC
from openpilot.selfdrive.locationd.helpers import Pose


class LatControl(ABC):
  def __init__(self, CP, CP_SP, CI, dt):
    self.dt = dt
    self.sat_limit = CP.steerLimitTimer
    self.sat_time = 0.
    self.sat_check_min_speed = 10.

    # ACSPilot: early "approaching saturation" warning, triggered before full
    # saturation (steerSaturating onroad event). Uses half the saturation timer.
    self.sat_warning_limit = CP.steerLimitTimer / 2.
    self.sat_warning_time = 0.
    self.saturating = False

    # we define the steer torque scale as [-1.0...1.0]
    self.steer_max = 1.0

  @abstractmethod
  def update(self, active: bool, CS, VM, params, steer_limited_by_safety: bool, desired_curvature: float, calibrated_pose: Pose,
             curvature_limited: bool, lat_delay: float):
    pass

  def reset(self):
    self.sat_time = 0.
    self.sat_warning_time = 0.
    self.saturating = False

  def _check_saturation(self, saturated, CS, steer_limited_by_safety, curvature_limited):
    # Saturated only if control output is not being limited by car torque/angle rate limits
    if (saturated or curvature_limited) and CS.vEgo > self.sat_check_min_speed and not steer_limited_by_safety and not CS.steeringPressed:
      self.sat_time += self.dt
    else:
      self.sat_time -= self.dt
    self.sat_time = np.clip(self.sat_time, 0.0, self.sat_limit)
    return self.sat_time > (self.sat_limit - 1e-3)

  def _check_saturating(self, saturating, CS):
    # ACSPilot: pre-warning that lateral control is approaching saturation.
    # Unlike _check_saturation, this ignores carOutput-based limiting so the
    # driver is warned even while the actuator is being rate/torque limited.
    if saturating and CS.vEgo > self.sat_check_min_speed and not CS.steeringPressed:
      self.sat_warning_time += self.dt
    else:
      self.sat_warning_time -= self.dt
    self.sat_warning_time = np.clip(self.sat_warning_time, 0.0, self.sat_warning_limit)
    return bool(self.sat_warning_time > (self.sat_warning_limit - 1e-3))
