import cereal.messaging as messaging
from cereal import log
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.test.process_replay.process_replay import get_process_config, replay_process

FINGERPRINT = TOYOTA.TOYOTA_COROLLA_TSS2
_DESIRE_LEN = len(log.Desire.schema.enumerants)


def _model(left_prob, right_prob):
  m = messaging.new_message('modelV2')
  mv = m.modelV2
  mv.laneLineProbs = [0.0, left_prob, right_prob, 0.0]
  mv.meta.desirePrediction = [0.0] * _DESIRE_LEN
  lane_lines = mv.init('laneLines', 4)
  for ll in lane_lines:
    ll.y = [0.0]
  return m.as_reader()


def _plannerd_msgs(left_prob, right_prob, n=50):
  msgs = []
  for _ in range(n):
    for svc in ('carState', 'carControl', 'controlsState', 'liveParameters', 'radarState', 'selfdriveState'):
      msgs.append(messaging.new_message(svc).as_reader())
    msgs.append(_model(left_prob, right_prob))
  return msgs


class TestLdwVisibilityFlow:
  # ACSPilot integration: plannerd runs the LDW model-visibility logic and publishes it on
  # driverAssistanceAC. Verify the published visibility tracks the fed modelV2 lane-line probabilities
  # end to end (the gate-independent behavior covered by the unit tests, through the real process).
  def _run(self, left_prob, right_prob):
    cfg = get_process_config("plannerd")
    cfg.subs = ["driverAssistanceAC"]  # local copy; does not touch the global CONFIGS
    out = replay_process(cfg, _plannerd_msgs(left_prob, right_prob), fingerprint=FINGERPRINT, disable_progress=True)
    return [m for m in out if m.which() == "driverAssistanceAC"]

  def test_left_visible_right_not(self):
    msgs = self._run(left_prob=0.9, right_prob=0.2)
    assert len(msgs) > 0
    last = msgs[-1].driverAssistanceAC
    assert last.leftLaneVisible is True
    assert last.rightLaneVisible is False

  def test_both_visible(self):
    msgs = self._run(left_prob=0.9, right_prob=0.9)
    assert len(msgs) > 0
    last = msgs[-1].driverAssistanceAC
    assert last.leftLaneVisible is True
    assert last.rightLaneVisible is True


# Note: a selfdrived -> selfdriveStateAC process-replay test is intentionally omitted. selfdrived has a
# heavy startup state machine that requires a realistic, recorded multi-service timeline to reach its
# main loop; synthetic carState-only input hangs it. The selfdriveStateAC override computation is covered
# by the fast unit tests in selfdrive/selfdrived/tests/test_selfdrive_state_ac.py instead.
