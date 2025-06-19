from cereal import log
from openpilot.selfdrive.selfdrived.events import Events, ET
from openpilot.common.realtime import DT_CTRL

State = log.SelfdriveState.OpenpilotState

SOFT_DISABLE_TIME = 3  # seconds
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)

class StateMachine:
  def __init__(self):
    self.current_alert_types = [ET.PERMANENT]
    self.state = State.disabled
    self.soft_disable_timer = 0

  def update(self, active_event_types: set[str]):
    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, SOFT DISABLING, PRE ENABLING, OVERRIDING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if ET.USER_DISABLE in active_event_types:
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif ET.IMMEDIATE_DISABLE in active_event_types:
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if ET.SOFT_DISABLE in active_event_types:
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif ET.OVERRIDE_LATERAL in active_event_types or ET.OVERRIDE_LONGITUDINAL in active_event_types:
            self.state = State.overriding
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if ET.SOFT_DISABLE not in active_event_types:
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if ET.PRE_ENABLE not in active_event_types:
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

        # OVERRIDING
        elif self.state == State.overriding:
          if ET.SOFT_DISABLE in active_event_types:
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif not (ET.OVERRIDE_LATERAL in active_event_types or ET.OVERRIDE_LONGITUDINAL in active_event_types):
            self.state = State.enabled
          else:
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

    # DISABLED
    elif self.state == State.disabled:
      if ET.ENABLE in active_event_types:
        if ET.NO_ENTRY in active_event_types:
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if ET.PRE_ENABLE in active_event_types:
            self.state = State.preEnabled
          elif ET.OVERRIDE_LATERAL in active_event_types or ET.OVERRIDE_LONGITUDINAL in active_event_types:
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)

    # Check if openpilot is engaged and actuators are enabled
    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES
    if active:
      self.current_alert_types.append(ET.WARNING)
    return enabled, active

