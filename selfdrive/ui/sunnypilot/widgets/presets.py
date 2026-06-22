"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle

# ACSPilot: operating-mode presets. Each preset writes a group of params.
# Note: the old fork's OnroadScreenOff param does not exist in 2026, so only the
# params that still exist are written (engagement mode + onroad-off brightness).
PRESETS = [
  (tr("Dashcam"), {"OpenpilotEnabledToggle": False, "AlphaLongitudinalEnabled": False, "OnroadScreenOffBrightness": 0}),
  (tr("Openpilot\nSteering"), {"OpenpilotEnabledToggle": True, "AlphaLongitudinalEnabled": False, "OnroadScreenOffBrightness": 50}),
  (tr("Openpilot\nFull"), {"OpenpilotEnabledToggle": True, "AlphaLongitudinalEnabled": True, "OnroadScreenOffBrightness": 50}),
]

TITLE_HEIGHT = 140
TITLE_FONT_SIZE = 90
BUTTON_SPACING = 30


def apply_preset(params: Params, preset: dict) -> None:
  # ACSPilot: write a preset's param group (bool params via put_bool, ints via put) and mark a preset
  # as selected so the boot-time picker (RequirePresetAtBoot) is satisfied.
  for key, value in preset.items():
    if isinstance(value, bool):
      params.put_bool(key, value)
    else:
      params.put(key, value)
  params.put_bool("PresetSelected", True)


class PresetsWidget(Widget):
  def __init__(self, on_selected: Callable[[], None] | None = None):
    super().__init__()
    self._params = Params()
    self._on_selected = on_selected
    self._font = gui_app.font(FontWeight.MEDIUM)
    self._buttons = [Button(label, click_callback=lambda p=preset: self._select(p), button_style=ButtonStyle.NORMAL)
                     for label, preset in PRESETS]

  def _select(self, preset: dict):
    apply_preset(self._params, preset)
    if self._on_selected is not None:
      self._on_selected()

  def _render(self, rect: rl.Rectangle):
    # Title
    rl.draw_text_ex(self._font, tr("Choose Operating Mode"), rl.Vector2(int(rect.x), int(rect.y)), TITLE_FONT_SIZE, 0, rl.WHITE)

    # Buttons row
    buttons_y = rect.y + TITLE_HEIGHT
    buttons_h = rect.height - TITLE_HEIGHT
    n = len(self._buttons)
    button_w = (rect.width - BUTTON_SPACING * (n - 1)) / n
    for i, button in enumerate(self._buttons):
      x = rect.x + i * (button_w + BUTTON_SPACING)
      button.set_rect(rl.Rectangle(x, buttons_y, button_w, buttons_h))
      button.render()
