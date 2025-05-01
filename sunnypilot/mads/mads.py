"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import Flag, auto

from cereal import car, log, custom

from opendbc.car.hyundai.values import HyundaiFlags

from openpilot.common.conversions import Conversions as CV
from openpilot.sunnypilot.mads.helpers import MadsParams
from openpilot.sunnypilot.mads.state import StateMachine

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = car.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
SafetyModel = car.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

DEFAULT_MADS_PAUSE_SPEED_WITH_BLINKER = 20 * CV.MPH_TO_MS
DEFAULT_MADS_PAUSE_SPEED = 10 * CV.MPH_TO_MS
DEFAULT_MADS_RESUME_SPEED = 40 * CV.MPH_TO_MS

class PauseReason(Flag):
  PRECONDITION_FAILED = auto()
  BRAKE = auto()
  SPEED = auto()

class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.mads_params = MadsParams()

    self.enabled = False
    self.active = False
    self.available = False
    self.allow_always = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.pause_reason = PauseReason(0)
    self.first_blinker_frame = 0

    if self.selfdrive.CP.brand == "hyundai":
      if self.selfdrive.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True

    # read params on init
    self.enabled_toggle = self.mads_params.read_param("Mads")
    self.read_params()

  def read_params(self):
    self.main_enabled_toggle = self.mads_params.read_param("MadsMainCruiseAllowed")
    self.pause_lateral_on_brake_toggle = self.mads_params.read_param("MadsPauseLateralOnBrake")
    self.unified_engagement_mode = self.mads_params.read_param("MadsUnifiedEngagementMode")
    self.pause_speed_with_blinker = self.mads_params.read_speed_param("MadsPauseSpeedWithBlinkerEnabled", "MadsPauseSpeedWithBlinker",
                                                                      DEFAULT_MADS_PAUSE_SPEED_WITH_BLINKER, self.selfdrive.is_metric)
    self.pause_speed = self.mads_params.read_speed_param("MadsPauseSpeedEnabled", "MadsPauseSpeed", DEFAULT_MADS_PAUSE_SPEED, self.selfdrive.is_metric)
    self.resume_speed = self.mads_params.read_speed_param("MadsResumeSpeedEnabled", "MadsResumeSpeed", DEFAULT_MADS_RESUME_SPEED, self.selfdrive.is_metric)

  def update_events(self, CS: car.CarState):
    def update_unified_engagement_mode():
      uem_blocked = self.enabled or (self.selfdrive.enabled and self.selfdrive.enabled_prev)
      if (self.unified_engagement_mode and uem_blocked) or not self.unified_engagement_mode:
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)

    def precondition_failed(old_event: int, new_event: int):
      self.events.remove(old_event)
      self.events_sp.add(new_event)
      self.pause_reason |= PauseReason.PRECONDITION_FAILED

    if not self.selfdrive.enabled and self.enabled:

      self.pause_reason &= ~PauseReason.PRECONDITION_FAILED
      if self.events.has(EventName.doorOpen):
        precondition_failed(EventName.doorOpen, EventNameSP.silentDoorOpen)
      if self.events.has(EventName.seatbeltNotLatched):
        precondition_failed(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
      if self.events.has(EventName.wrongGear):
        precondition_failed(EventName.wrongGear, EventNameSP.silentWrongGear)
      if self.events.has(EventName.reverseGear):
        precondition_failed(EventName.reverseGear, EventNameSP.silentReverseGear)
      if self.events.has(EventName.brakeHold):
        precondition_failed(EventName.brakeHold, EventNameSP.silentBrakeHold)
      if self.events.has(EventName.parkBrake):
        precondition_failed(EventName.parkBrake, EventNameSP.silentParkBrake)

      if self.pause_lateral_on_brake_toggle:
        if CS.brakePressed:
          self.pause_reason |= PauseReason.BRAKE
        else:
          self.pause_reason &= ~PauseReason.BRAKE

      if self.pause_speed_with_blinker is not None \
          and (CS.leftBlinker or CS.rightBlinker) \
          and self.selfdrive.CS_prev.vEgoCluster < self.pause_speed_with_blinker:
        self.pause_reason |= PauseReason.SPEED

      if self.pause_speed is not None and CS.vEgoCluster < self.pause_speed and CS.steeringPressed:
        self.pause_reason |= PauseReason.SPEED

      if self.resume_speed is not None and CS.vEgoCluster >= self.resume_speed:
        self.pause_reason &= ~PauseReason.SPEED

      if self.pause_reason:
        if self.state_machine.state != State.paused:
          self.events_sp.add(EventNameSP.silentLkasDisable)
      else:
        if self.state_machine.state == State.paused:
          self.events_sp.add(EventNameSP.silentLkasEnable)

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)

    if self.selfdrive.enabled:
      self.pause_reason = PauseReason(0)

    if self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable):
      update_unified_engagement_mode()
    else:
      if self.main_enabled_toggle:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)

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

    if not CS.cruiseState.available:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available:
        self.events_sp.add(EventNameSP.lkasDisable)

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)
    if not any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents):
      self.events.remove(EventName.wrongCarMode)

  def update(self, CS: car.CarState):
    if not self.enabled_toggle:
      return

    self.update_events(CS)

    if not self.selfdrive.CP.passive and self.selfdrive.initialized:
      self.enabled, self.active = self.state_machine.update()

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled
