import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.ldw import LaneDepartureWarning, LDW_MIN_SPEED


def _model(left_prob=0.9, right_prob=0.9, with_desire=True):
  m = messaging.new_message('modelV2')
  mv = m.modelV2
  mv.laneLineProbs = [0.0, left_prob, right_prob, 0.0]
  if with_desire:
    mv.meta.desirePrediction = [0.0] * 7
  lane_lines = mv.init('laneLines', 4)
  for ll in lane_lines:
    ll.y = [0.0]
  return mv


def _cs(v_ego=30.0):
  m = messaging.new_message('carState')
  m.carState.vEgo = v_ego
  return m.carState


def _cc(lat_active=False):
  m = messaging.new_message('carControl')
  m.carControl.latActive = lat_active
  return m.carControl


class TestLaneDepartureWarning:
  def test_visibility_tracks_lane_probs(self):
    # ACSPilot: visibility follows model confidence (prob > 0.5), independent of the LDW gating
    ldw = LaneDepartureWarning()
    ldw.update(0, _model(left_prob=0.9, right_prob=0.2), _cs(), _cc())
    assert ldw.left_lane_visible is True
    assert ldw.right_lane_visible is False

  def test_visibility_independent_of_ldw_gate(self):
    # below LDW min speed and with lateral active the departure warning is suppressed,
    # but lane visibility must still reflect the model so the HUD keeps drawing lines
    ldw = LaneDepartureWarning()
    ldw.update(0, _model(left_prob=0.9, right_prob=0.9), _cs(v_ego=LDW_MIN_SPEED - 1.0), _cc(lat_active=True))
    assert ldw.left is False and ldw.right is False
    assert ldw.left_lane_visible is True
    assert ldw.right_lane_visible is True

  def test_visibility_false_without_desire_prediction(self):
    ldw = LaneDepartureWarning()
    ldw.update(0, _model(with_desire=False), _cs(), _cc())
    assert ldw.left_lane_visible is False
    assert ldw.right_lane_visible is False

  def test_visibility_types_are_bool(self):
    ldw = LaneDepartureWarning()
    ldw.update(0, _model(), _cs(), _cc())
    assert isinstance(ldw.left_lane_visible, bool)
    assert isinstance(ldw.right_lane_visible, bool)
