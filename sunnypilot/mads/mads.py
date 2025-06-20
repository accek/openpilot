"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import Flag, auto

from cereal import log, custom

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, read_steering_mode_param, get_mads_limited_brands, read_speed_param
from openpilot.sunnypilot.mads.state import StateMachine

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

DEFAULT_MADS_PAUSE_SPEED_WITH_BLINKER = 20 * CV.MPH_TO_MS
DEFAULT_MADS_PAUSE_SPEED = 10 * CV.MPH_TO_MS
DEFAULT_MADS_RESUME_SPEED = 40 * CV.MPH_TO_MS

class PauseReason(Flag):
  PRECONDITION_FAILED = auto()
  BRAKE = auto()
  SPEED = auto()

class ResumeReason(Flag):
  BRAKE_RELEASED = auto()
  SPEED = auto()
  CRUISE_ENABLED = auto()

class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.CP = selfdrive.CP
    self.params = selfdrive.params

    self.enabled = False
    self.active = False
    self.available = False
    self.allow_always = False
    self.no_main_cruise = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.paused_for_braking = False
    self.first_blinker_frame = 0
    self.disengage_on_accelerator = Params().get_bool("DisengageOnAccelerator")
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True

    if get_mads_limited_brands(self.CP):
      self.no_main_cruise = True

    # read params on init
    self.enabled_toggle = self.params.get_bool("Mads")
    self.steering_mode_on_brake = read_steering_mode_param(self.CP, self.params)
    self.read_params()

  def read_params(self):
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")
    self.pause_speed_with_blinker = read_speed_param(self.params, "MadsPauseSpeedWithBlinkerEnabled", "MadsPauseSpeedWithBlinker",
                                                     DEFAULT_MADS_PAUSE_SPEED_WITH_BLINKER, self.selfdrive.is_metric)
    self.pause_speed = read_speed_param(self.params, "MadsPauseSpeedEnabled", "MadsPauseSpeed", DEFAULT_MADS_PAUSE_SPEED, self.selfdrive.is_metric)
    self.resume_speed = read_speed_param(self.params, "MadsResumeSpeedEnabled", "MadsResumeSpeed", DEFAULT_MADS_RESUME_SPEED, self.selfdrive.is_metric)

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def update_events(self, CS: structs.CarState):
    pause_reason = PauseReason(0)
    resume_reason = ResumeReason(0)

    if not self.selfdrive.enabled and self.enabled:
      for old_event, new_event in [
        (EventName.doorOpen, EventNameSP.silentDoorOpen),
        (EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched),
        (EventName.wrongGear, EventNameSP.silentWrongGear),
        (EventName.reverseGear, EventNameSP.silentReverseGear),
        (EventName.brakeHold, EventNameSP.silentBrakeHold),
        (EventName.parkBrake, EventNameSP.silentParkBrake),
      ]:
        if self.events.has(old_event):
          self.events.remove(old_event)
          self.events_sp.add(new_event)
          pause_reason |= PauseReason.PRECONDITION_FAILED

      if self.pause_speed_with_blinker is not None \
          and (CS.leftBlinker or CS.rightBlinker) \
          and self.selfdrive.CS_prev.vEgoCluster < self.pause_speed_with_blinker \
          and not self.selfdrive.enabled:
        pause_reason |= PauseReason.SPEED

      if self.pause_speed is not None and CS.vEgoCluster < self.pause_speed and CS.steeringPressed \
          and not self.selfdrive.enabled:
        pause_reason |= PauseReason.SPEED

      if self.resume_speed is not None and CS.vEgoCluster >= self.resume_speed:
        resume_reason |= ResumeReason.SPEED

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
      if CS.brakePressed and (self.state_machine.state != State.paused or self.paused_for_braking):
        pause_reason |= PauseReason.BRAKE
        self.paused_for_braking = True
      elif self.paused_for_braking and not CS.brakePressed:
        resume_reason |= ResumeReason.BRAKE_RELEASED
        self.paused_for_braking = False

    if self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable):
      if not self.unified_engagement_mode or self.enabled or (self.selfdrive.enabled and self.selfdrive.enabled_prev):
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)

    if self.unified_engagement_mode and self.selfdrive.enabled:
      resume_reason |= ResumeReason.CRUISE_ENABLED
    if self.main_enabled_toggle and CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
      self.events_sp.add(EventNameSP.lkasEnable)

    if pause_reason:
      if self.state_machine.state != State.paused:
        self.events_sp.add(EventNameSP.silentLkasDisable)
    elif resume_reason:
      if self.state_machine.state == State.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)

    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.selfdrive.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      if be.type == ButtonType.lkas and be.pressed and (CS.cruiseState.available or self.allow_always):
        if self.enabled:
          if self.selfdrive.enabled:
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.events_sp.add(EventNameSP.lkasEnable)

    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState):
    if not self.enabled_toggle:
      return

    self.update_events(CS)

    if not self.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled
