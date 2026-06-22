from cereal import log
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.selfdrived import override_by_stock_acc_only

State = log.SelfdriveState.OpenpilotState


class TestOverrideByStockAccOnly:
  # ACSPilot: selfdriveStateAC.overrideByStockAccOnly drives whether the HUD greys out while overriding.
  # It must be True only when openpilot is in the overriding state AND neither an openpilot lateral nor
  # longitudinal override is active (i.e. the stock ACC alone is in control of longitudinal).
  def test_false_when_not_overriding(self):
    for state in State.schema.enumerants.values():
      if state == State.overriding:
        continue
      # even with no override events, a non-overriding state is never "stock-acc-only"
      assert override_by_stock_acc_only(state, set()) is False

  def test_true_when_overriding_without_op_override(self):
    assert override_by_stock_acc_only(State.overriding, set()) is True

  def test_false_when_lateral_override_active(self):
    assert override_by_stock_acc_only(State.overriding, {ET.OVERRIDE_LATERAL}) is False

  def test_false_when_longitudinal_override_active(self):
    assert override_by_stock_acc_only(State.overriding, {ET.OVERRIDE_LONGITUDINAL}) is False

  def test_false_when_both_overrides_active(self):
    assert override_by_stock_acc_only(State.overriding, {ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL}) is False

  def test_unrelated_events_do_not_matter(self):
    # an unrelated active event type while overriding still counts as stock-acc-only
    assert override_by_stock_acc_only(State.overriding, {ET.WARNING}) is True
