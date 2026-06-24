"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise_sub_layouts.speed_limit_settings import SpeedLimitSettingsLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp, option_item_sp, simple_button_item_sp
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller


class PanelType(IntEnum):
  CRUISE = 0
  SLA = 1


ICBM_DESC = tr_noop("When enabled, sunnypilot will attempt to manage the built-in cruise control buttons " +
                    "by emulating button presses for limited longitudinal control.")
ICMB_UNAVAILABLE = tr_noop("Intelligent Cruise Button Management is currently unavailable on this platform.")
ICMB_UNAVAILABLE_LONG_AVAILABLE = tr_noop("Disable the sunnypilot Longitudinal Control (alpha) toggle to allow Intelligent Cruise Button Management.")
ICMB_UNAVAILABLE_LONG_UNAVAILABLE = tr_noop("sunnypilot Longitudinal Control is the default longitudinal control for this platform.")

ONROAD_ONLY_DESCRIPTION = tr_noop("Start the vehicle to check vehicle compatibility.")


class CruiseLayout(Widget):
  def __init__(self):
    super().__init__()
    self._current_panel = PanelType.CRUISE
    self._speed_limit_layout = SpeedLimitSettingsLayout(lambda: self._set_current_panel(PanelType.CRUISE))

    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):

    self.icbm_toggle = toggle_item_sp(
      title=tr("Intelligent Cruise Button Management (ICBM) (Alpha)"),
      description="",
      param="IntelligentCruiseButtonManagement")

    self.scc_v_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Vision"),
      description=tr("Use vision path predictions to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlVision")

    self.scc_m_toggle = toggle_item_sp(
      title=tr("Smart Cruise Control - Map"),
      description=tr("Use map data to estimate the appropriate speed to drive through turns ahead."),
      param="SmartCruiseControlMap")

    self.sla_settings_button = simple_button_item_sp(
      button_text=lambda: tr("Speed Limit"),
      button_width=800,
      callback=lambda: self._set_current_panel(PanelType.SLA)
    )

    self.dec_toggle = toggle_item_sp(
      title=tr("Enable Dynamic Experimental Control"),
      description=tr("Enable toggle to allow the model to determine when to use sunnypilot ACC or sunnypilot End to End Longitudinal."),
      param="DynamicExperimentalControl")

    # ACSPilot: use stock ACC above a configured set speed (0 = never)
    self.stock_acc_override_speed = option_item_sp(
      title=tr("Stock ACC Override Speed"),
      description=tr("Use stock ACC when set speed exceeds the given speed."),
      param="StockAccOverrideSpeed",
      min_value=0, max_value=255, value_change_step=5,
      label_callback=self._stock_acc_override_label)

    items = [
      self.icbm_toggle,
      self.dec_toggle,
      self.scc_v_toggle,
      self.scc_m_toggle,
      self.stock_acc_override_speed,
      self.sla_settings_button,
    ]
    return items

  def _stock_acc_override_label(self, value: int) -> str:
    if value <= 0:
      return tr("Never")
    return f"{value} " + (tr("km/h") if ui_state.is_metric else tr("mph"))

  def _render(self, rect):
    if self._current_panel == PanelType.SLA:
      self._speed_limit_layout.render(rect)
    else:
      self._scroller.render(rect)

  def show_event(self):
    self._set_current_panel(PanelType.CRUISE)
    self._scroller.show_event()
    self.icbm_toggle.show_description(True)

  def _set_current_panel(self, panel: PanelType):
    self._current_panel = panel
    if panel == PanelType.SLA:
      self._speed_limit_layout.show_event()

  def _update_state(self):
    super()._update_state()

    if ui_state.CP is not None and ui_state.CP_SP is not None:
      has_icbm = ui_state.has_icbm
      has_long = ui_state.has_longitudinal_control

      if ui_state.CP_SP.intelligentCruiseButtonManagementAvailable and not has_long:
        self.icbm_toggle.action_item.set_enabled(ui_state.is_offroad())
        self.icbm_toggle.set_description(tr(ICBM_DESC))
      else:
        ui_state.params.remove("IntelligentCruiseButtonManagement")
        self.icbm_toggle.action_item.set_enabled(False)

        long_desc = ICMB_UNAVAILABLE
        if has_long:
          if ui_state.CP.alphaLongitudinalAvailable:
            long_desc += " " + ICMB_UNAVAILABLE_LONG_AVAILABLE
          else:
            long_desc += " " + ICMB_UNAVAILABLE_LONG_UNAVAILABLE

        new_desc = "<b>" + tr(long_desc) + "</b>\n\n" + tr(ICBM_DESC)
        if self.icbm_toggle.description != new_desc:
          self.icbm_toggle.set_description(new_desc)
          self.icbm_toggle.show_description(True)

      if has_long or has_icbm:
        self.dec_toggle.action_item.set_enabled(has_long)
        self.scc_v_toggle.action_item.set_enabled(True)
        self.scc_m_toggle.action_item.set_enabled(True)
      else:
        ui_state.params.remove("DynamicExperimentalControl")
        ui_state.params.remove("SmartCruiseControlVision")
        ui_state.params.remove("SmartCruiseControlMap")
        self.dec_toggle.action_item.set_enabled(False)
        self.scc_v_toggle.action_item.set_enabled(False)
        self.scc_m_toggle.action_item.set_enabled(False)

    else:
      has_icbm = has_long = False
      self.icbm_toggle.action_item.set_enabled(False)
      self.icbm_toggle.set_description(tr(ONROAD_ONLY_DESCRIPTION))

    # ACSPilot: stock-ACC override speed — only shown if the car supports it, enabled only with op long
    stock_acc_available = ui_state.CP_AC is not None and ui_state.CP_AC.stockAccOverrideAvailable
    self.stock_acc_override_speed.set_visible(stock_acc_available)
    if stock_acc_available:
      self.stock_acc_override_speed.action_item.set_enabled(ui_state.has_longitudinal_control)
