import pytest

import cereal.messaging as messaging
from openpilot.selfdrive.controls.radard import get_RadarState_from_vision, RADAR_TO_CAMERA


def _vision_lead(x=30.0, y=0.5, v=20.0, a=1.0, prob=0.9):
  m = messaging.new_message('modelV2')
  leads = m.modelV2.init('leadsV3', 1)
  lead = leads[0]
  lead.x = [x]
  lead.y = [y]
  lead.v = [v]
  lead.a = [a]
  lead.prob = prob
  return m.as_reader().modelV2.leadsV3[0]


class TestRadarStateFromVision:
  # ACSPilot: vision-only leads now report aRel (relative acceleration) = lead accel - ego accel, so the
  # longitudinal planner can react to a lead's acceleration even without a radar track.
  def test_arel_is_lead_accel_minus_ego_accel(self):
    out = get_RadarState_from_vision(_vision_lead(a=2.0), v_ego=20.0, model_v_ego=20.0, a_ego=0.5)
    assert out["aRel"] == pytest.approx(2.0 - 0.5)
    assert out["aLeadK"] == pytest.approx(2.0)

  def test_arel_negative_when_ego_accelerates_faster_than_lead(self):
    out = get_RadarState_from_vision(_vision_lead(a=0.0), v_ego=20.0, model_v_ego=20.0, a_ego=1.5)
    assert out["aRel"] == pytest.approx(-1.5)

  def test_arel_zero_when_matched(self):
    out = get_RadarState_from_vision(_vision_lead(a=1.0), v_ego=20.0, model_v_ego=20.0, a_ego=1.0)
    assert out["aRel"] == pytest.approx(0.0)

  def test_drel_still_offset_by_radar_to_camera(self):
    # sanity: the ACSPilot aRel addition must not disturb the existing fields
    out = get_RadarState_from_vision(_vision_lead(x=30.0), v_ego=20.0, model_v_ego=20.0, a_ego=0.0)
    assert out["dRel"] == pytest.approx(30.0 - RADAR_TO_CAMERA)
    assert out["status"] is True
