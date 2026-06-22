import numpy as np

import cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc


def _radarstate(lead_d=None, lead_v=0.0, lead_a=0.0):
  rs = messaging.new_message('radarState').radarState
  if lead_d is not None:
    lead = rs.leadOne
    lead.status = True
    lead.dRel = lead_d
    lead.vLead = lead_v
    lead.aLeadK = lead_a
    lead.aLeadTau = 0.3
    lead.modelProb = 0.95
  return rs


def _converge(mpc, v_ego, a_ego, v_cruise, radarstate, iters=20):
  mpc.set_weights()
  for _ in range(iters):
    mpc.set_cur_state(v_ego, a_ego)
    mpc.update(radarstate, v_cruise)
  return mpc


class TestLongMpcLowSpeed:
  # ACSPilot: the long MPC cost/constraint denominators were changed from (v_ego + 10) to fmax(v_ego, 2.5),
  # which sharpens distance tracking at low speed. These tests pin that the solver still converges to a
  # finite, sane solution across the low-speed regime the change affects.
  def setup_method(self):
    self.mpc = LongitudinalMpc()

  def test_low_speed_cruise_converges_and_accelerates(self):
    rs = _radarstate()  # no lead
    self._assert_finite(_converge(self.mpc, v_ego=2.0, a_ego=0.0, v_cruise=15.0, radarstate=rs))
    # below the cruise target with no lead, the planned speed should rise over the horizon
    # (a_solution[0] is pinned to the current accel, so check the trajectory, not the first node)
    assert self.mpc.v_solution[-1] > self.mpc.v_solution[0]

  def test_low_speed_close_lead_does_not_accelerate(self):
    rs = _radarstate(lead_d=6.0, lead_v=0.0)  # stopped lead one safe-distance ahead
    self._assert_finite(_converge(self.mpc, v_ego=3.0, a_ego=0.0, v_cruise=15.0, radarstate=rs))
    # must not speed up toward a close stopped lead, and must not flag a crash
    assert self.mpc.v_solution[-1] <= self.mpc.v_solution[0] + 0.5
    assert self.mpc.crash_cnt == 0

  def test_zero_speed_stopped_lead_stays_put(self):
    rs = _radarstate(lead_d=5.0, lead_v=0.0)
    self._assert_finite(_converge(self.mpc, v_ego=0.0, a_ego=0.0, v_cruise=15.0, radarstate=rs))
    # from a standstill behind a close stopped lead the plan should not lunge forward
    assert self.mpc.a_solution[0] <= 0.2
    assert np.max(self.mpc.v_solution) < 2.0

  @staticmethod
  def _assert_finite(mpc):
    # the solver must converge to a finite solution (the old (v_ego + 10) denominator could not blow up
    # near zero speed, and neither may the new fmax(v_ego, 2.5) form). Soft constraints permit small
    # negative-velocity slack near a stop, so only guard against real divergence here.
    assert np.all(np.isfinite(mpc.a_solution))
    assert np.all(np.isfinite(mpc.v_solution))
    assert np.all(mpc.v_solution >= -1.0)
