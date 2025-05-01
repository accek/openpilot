"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import car, car_custom, custom
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
ButtonTypeAC = car_custom.CarStateAC.ButtonEvent.Type
EventNameSP = custom.OnroadEventSP.EventName

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  def __init__(self, CP: structs.CarParams, CP_AC: structs.CarParamsAC) -> None:
    self.CP = CP
    self.CP_AC = CP_AC
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self.button_frame_counts_ac = {ButtonTypeAC.gapAdjustCruiseUp: 0, ButtonTypeAC.gapAdjustCruiseDown: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False

  def update(self, CS, CS_AC, events, experimental_mode) -> None:
    if self.CP.openpilotLongitudinalControl:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)
        self.update_button_frame_counts_ac(CS_AC)

        # toggle experimental mode once on distance button hold
        self.update_experimental_mode(events, experimental_mode)

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_button_frame_counts_ac(self, CS_AC) -> None:
    for button in self.button_frame_counts_ac:
      if self.button_frame_counts_ac[button] > 0:
        self.button_frame_counts_ac[button] += 1

    for button_event in CS_AC.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts_ac:
        self.button_frame_counts_ac[button] = int(button_event.pressed)

  def any_button_long_pressed(self) -> bool:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] >= DISTANCE_LONG_PRESS:
        return True
    for button in self.button_frame_counts_ac:
      if self.button_frame_counts_ac[button] >= DISTANCE_LONG_PRESS:
        return True
    return False

  def update_experimental_mode(self, events, experimental_mode) -> None:
    if self.any_button_long_pressed() and not self.experimental_mode_switched:
        self._experimental_mode = not experimental_mode
        self.params.put_bool_nonblocking("ExperimentalMode", self._experimental_mode)
        events.add(EventNameSP.experimentalModeSwitched)
        self.experimental_mode_switched = True
