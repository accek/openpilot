#!/usr/bin/env python3
import math
from typing import SupportsFloat

from cereal import car, log, custom, car_custom
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog

from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.locationd.helpers import PoseCalibrator, Pose

State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())


class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")

    cloudlog.info("controlsd is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(self.params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd got CarParamsSP")

    cloudlog.info("controlsd is waiting for CarParamsAC")
    self.CP_AC = messaging.log_from_bytes(self.params.get("CarParamsAC", block=True), car_custom.CarParamsAC)
    cloudlog.info("controlsd got CarParamsAC")

    self.CI = interfaces[self.CP.carFingerprint](self.CP, self.CP_SP, self.CP_AC)

    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'livePose', 'longitudinalPlan', 'carState', 'carOutput',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'radarState'] + ['selfdriveStateSP']
                                   + ['onroadEventsAC', 'driverAssistanceAC'],
                                  poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'] + ['carControlSP'] + ['carControlAC', 'controlsStateAC'])

    self.steer_limited_by_controls = False
    self.desired_curvature = 0.0

    self.pose_calibrator = PoseCalibrator()
    self.calibrated_pose: Pose | None = None

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)

    self.is_metric = self.params.get_bool("IsMetric")
    self.stock_acc_override_speed = float(self.params.get("StockAccOverrideSpeed") or 0.0) * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
    if self.stock_acc_override_speed == 0.0:
      self.stock_acc_override_speed = float("inf")

  def update(self):
    self.sm.update(15)
    if self.sm.updated["liveCalibration"]:
      self.pose_calibrator.feed_live_calib(self.sm['liveCalibration'])
    if self.sm.updated["livePose"]:
      device_pose = Pose.from_live_pose(self.sm['livePose'])
      self.calibrated_pose = self.pose_calibrator.build_calibrated_pose(device_pose)

  def state_control(self):
    CS = self.sm['carState']

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']

    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled

    # Check which actuators can be enabled
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill

    ss_sp = self.sm['selfdriveStateSP']
    if ss_sp.mads.available:
      _lat_active = ss_sp.mads.active
    else:
      _lat_active = self.sm['selfdriveState'].active

    CC.latActive = _lat_active and not CS.steerFaultTemporary and not CS.steerFaultPermanent and not standstill
    CC.longActive = CC.enabled and \
      not any(e.overrideLongitudinal for e in self.sm['onroadEvents']) and \
      not any(e.overrideLongitudinal for e in self.sm['onroadEventsAC']) and \
      self.CP.openpilotLongitudinalControl

    CC_AC = car_custom.CarControlAC.new_message()
    if self.CP.openpilotLongitudinalControl:
      CC_AC.stockAccOverrideArmed = (CS.vCruiseCluster if CS.vCruiseCluster != V_CRUISE_UNSET else CS.vEgoCluster) >= self.stock_acc_override_speed
      CC_AC.stockAccOverrideActive = CC_AC.stockAccOverrideArmed and CC.enabled
    # TODO: fill CC_AC

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()

    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    actuators.accel = float(self.LoC.update(CC.longActive, CS, long_plan.aTarget, long_plan.shouldStop, pid_accel_limits))

    # Steering PID loop and lateral MPC
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature, lp.roll)
    actuators.curvature = self.desired_curvature
    steer, steeringAngleDeg, lac_log, lac_log_ac = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                   self.steer_limited_by_controls, self.desired_curvature,
                                                                   self.calibrated_pose, curvature_limited, model_data=model_v2)  # TODO what if not available
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    CC_SP = custom.CarControlSP.new_message()
    CC_SP.mads = ss_sp.mads

    return CC, CC_SP, CC_AC, lac_log, lac_log_ac

  def publish(self, CC, CC_SP, CC_AC, lac_log, lac_log_ac):
    CS = self.sm['carState']

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    if self.calibrated_pose is not None:
      CC.orientationNED = self.calibrated_pose.orientation.xyz.tolist()
      CC.angularVelocity = self.calibrated_pose.angular_velocity.xyz.tolist()

    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1

    hudControl = CC.hudControl
    hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual

    if self.sm.valid['driverAssistanceAC']:
      hudControl.leftLaneVisible = self.sm['driverAssistanceAC'].leftLaneVisible
      hudControl.rightLaneVisible = self.sm['driverAssistanceAC'].rightLaneVisible
    else:
      hudControl.leftLaneVisible = True
      hudControl.rightLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture

    hudControlAC = CC_AC.hudControl
    hudControlAC.leadDistance = float('nan')
    hudControlAC.leadAccel = float('nan')
    radar_state = self.sm['radarState'] if self.sm.valid['radarState'] else None
    if radar_state is not None:
      lead_one = radar_state.leadOne
      lead_two = radar_state.leadTwo
      lead = None
      if lead_one.status and (not lead_two.status or lead_one.dRel < lead_two.dRel):
        lead = lead_one
      elif lead_two.status:
        lead = lead_two
      if lead is not None:
        hudControlAC.leadDistance = lead.dRel
        hudControlAC.leadAccel = lead.aRel

    dm_state = self.sm['driverMonitoringState'] if self.sm.valid['driverMonitoringState'] else None
    CC_AC.stockDriverMonitoring = dm_state is None or dm_state.isDistracted or not dm_state.faceDetected

    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_controls = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_controls = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2

    # TODO: both controlsState and carControl valids should be set by
    #       sm.all_checks(), but this creates a circular dependency

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState

    lp = self.sm['liveParameters']
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    cs.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0.) or
                         (self.sm['selfdriveState'].state == State.softDisabling))

    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    # controlsStateAC
    dat_ac = messaging.new_message('controlsStateAC')
    dat_ac.valid = CS.canValid
    cs_ac = dat_ac.controlsStateAC
    cs_ac.lateralControlState = lac_log_ac
    self.pm.send('controlsStateAC', dat_ac)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # carControlSP
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = CS.canValid
    cc_sp_send.carControlSP = CC_SP
    self.pm.send('carControlSP', cc_sp_send)

    # carControlAC
    cc_ac_send = messaging.new_message('carControlAC')
    cc_ac_send.valid = CS.canValid
    cc_ac_send.carControlAC = CC_AC
    self.pm.send('carControlAC', cc_ac_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, CC_SP, CC_AC, lac_log, lac_log_ac = self.state_control()
      self.publish(CC, CC_SP, CC_AC, lac_log, lac_log_ac)
      rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()


if __name__ == "__main__":
  main()
