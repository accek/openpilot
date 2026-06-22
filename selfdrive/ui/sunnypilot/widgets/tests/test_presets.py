from openpilot.common.params import Params
from openpilot.selfdrive.ui.sunnypilot.widgets.presets import apply_preset, PRESETS


class TestApplyPreset:
  # ACSPilot: each operating-mode preset writes a fixed group of params (bool via put_bool, int via put)
  # and marks PresetSelected so the boot-time picker (RequirePresetAtBoot) is satisfied.
  def test_each_preset_writes_its_param_group_and_marks_selected(self):
    params = Params()
    for _label, preset in PRESETS:
      params.put_bool("PresetSelected", False)
      apply_preset(params, preset)
      assert params.get_bool("PresetSelected") is True
      for key, value in preset.items():
        if isinstance(value, bool):
          assert params.get_bool(key) is value
        else:
          assert int(params.get(key)) == value

  def test_presets_are_distinct_and_nonempty(self):
    labels = [label for label, _ in PRESETS]
    assert len(labels) == len(set(labels))
    assert all(len(preset) > 0 for _, preset in PRESETS)

  def test_selecting_one_preset_overwrites_a_previous_one(self):
    params = Params()
    dashcam = dict(PRESETS[0][1])
    full = dict(PRESETS[-1][1])
    apply_preset(params, dashcam)
    apply_preset(params, full)
    for key, value in full.items():
      if isinstance(value, bool):
        assert params.get_bool(key) is value
      else:
        assert int(params.get(key)) == value
