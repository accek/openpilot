from cereal import log, car, custom
import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, ImmediateDisableAlert, SoftDisableAlert, AlertCallbackType


AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventNameAC = custom.OnroadEventAC.EventName


# get event name from enum
EVENT_NAME_AC = {v: k for k, v in EventNameAC.schema.enumerants.items()}


class EventsAC(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_AC.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_AC

  def get_event_name(self, event: int):
    return EVENT_NAME_AC[event]

  def get_event_msg_type(self):
    return custom.OnroadEventAC


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func


EVENTS_AC: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # ACSPilot

  EventNameAC.steerSaturating: {
    ET.WARNING: Alert(
      "Monitor Steering",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, 2.),
  },

  EventNameAC.stockAccOverride: {
    ET.OVERRIDE_LONGITUDINAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },

  EventNameAC.accFaultedTemporary: {
    ET.SOFT_DISABLE: soft_disable_alert("Cruise Fault"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault"),
  },
}
