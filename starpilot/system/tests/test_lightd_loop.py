import threading
import time

import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.starpilot.system.lightd import lightd as L
from openpilot.starpilot.system.lightd.hba import FCAM_H, FCAM_W, Nv12Frame

DT = 0.05
GPS = "gpsLocation"


class NoParams:
  def get(self, key):
    return None

  def get_bool(self, key):
    return False


class FakeCameraRunner(L.HbaRunner):
  """The real runner and classifier on a dark synthetic frame instead of VisionIPC. `hang` stalls the classifier."""

  def __init__(self):
    super().__init__()
    self.frame_id = 0
    self.hang = threading.Event()
    self.release = threading.Event()
    self.calls = 0
    update = self.clf.update

    def slow_update(*args, **kwargs):
      self.calls += 1
      if self.hang.is_set():
        self.release.wait(10.0)
      return update(*args, **kwargs)
    self.clf.update = slow_update

  def _connect(self):
    return True

  def _latest_frame(self):
    return Nv12Frame(np.full((FCAM_H, FCAM_W), 16, np.uint8), np.full((FCAM_H // 2, FCAM_W), 128, np.uint8)), self.frame_id


def new(svc, t):
  return messaging.new_message(svc, valid=True, logMonoTime=int(t * 1e9))


def camera_msg(svc, t, frame_id):
  m = new(svc, t)
  cs = getattr(m, svc)
  cs.frameId = frame_id
  cs.measuredGreyFraction = 0.3
  cs.exposureValPercent = 0.5  # bright daylight
  return m


def inputs(t, frame_id):
  msgs = [camera_msg("wideRoadCameraState", t, frame_id), camera_msg("roadCameraState", t, frame_id)]
  m = new("carState", t)
  m.carState.vEgo = 25.0
  msgs.append(m)
  msgs.append(new("starpilotCarState", t))
  m = new("liveCalibration", t)
  m.liveCalibration.calStatus = log.LiveCalibrationData.Status.calibrated
  m.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  m.liveCalibration.height = [1.22]
  msgs.append(m)
  m = new("modelV2", t)
  md = m.modelV2
  md.frameId = frame_id
  xs = np.linspace(0, 190, 33).tolist()
  md.position.x, md.position.y, md.position.z = xs, [0.0] * 33, [0.0] * 33
  md.init("laneLines", 4)
  for line, y in zip(md.laneLines, (-5.4, -1.8, 1.8, 5.4), strict=True):
    line.x, line.y, line.z = xs, [y] * 33, [0.0] * 33
  md.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  msgs.append(m)
  return [msg.as_reader() for msg in msgs]


def drive(ld, sm, runner, mode, t0, seconds, on_step=None):
  outs = []
  for i in range(int(round(seconds / DT))):
    t = t0 + i * DT
    runner.frame_id += 1
    sm.update_msgs(t, inputs(t, runner.frame_id))
    w0 = time.monotonic()
    ls = ld.step(sm, mode)
    outs.append((t, time.monotonic() - w0, ls))
    if on_step:
      on_step(t, ls)
    time.sleep(0.004)  # let the worker thread run; simulated time still runs ~10x faster than real time
  return outs


def test_slow_hba_keeps_low_beam_and_lapses_hba():
  runner = FakeCameraRunner()
  ld = L.LightD(False, GPS, L.SolarTracker(NoParams()), L.HbaWorker(runner))
  sm = ld.submaster()
  try:
    # settle the low-beam estimator with a working HBA
    outs = drive(ld, sm, runner, L.MODE_LOW_AND_HIGH_BEAM, 0.0, 8.0)
    t, _, last = outs[-1]
    assert last.valid and last.starpilotLightState.lowBeamValid
    assert last.starpilotLightState.hbaValid, "a working HBA should be valid"

    # now every HBA frame hangs: like 100+ ms frames on the device, only worse
    runner.hang.set()
    calls = runner.calls
    outs = drive(ld, sm, runner, L.MODE_LOW_AND_HIGH_BEAM, t + DT, 2.0)
    assert max(wall for _, wall, _ in outs) < 0.03, "the loop must never wait for the HBA"
    assert all(ls.valid and ls.starpilotLightState.lowBeamValid for _, _, ls in outs)
    assert runner.calls <= calls + 1, "the worker should be stuck in one frame"
    lapsed = [t_ for t_, _, ls in outs if not ls.starpilotLightState.hbaValid]
    assert lapsed and lapsed[0] - outs[0][0] <= L.HBA_STALE_S + L.HBA_PERIOD_S + DT
    assert all(not ls.starpilotLightState.hbaValid for t_, _, ls in outs if t_ >= lapsed[0])
    assert all(ls.starpilotLightState.hbaReason == 255 for t_, _, ls in outs if t_ >= lapsed[0])
  finally:
    runner.release.set()


def test_dead_worker_invalidates_hba():
  runner = FakeCameraRunner()
  ld = L.LightD(False, GPS, L.SolarTracker(NoParams()), L.HbaWorker(runner))
  sm = ld.submaster()
  outs = drive(ld, sm, runner, L.MODE_LOW_AND_HIGH_BEAM, 0.0, 1.0)
  assert outs[-1][2].starpilotLightState.hbaValid

  # the worker thread ends (an error outside its per-frame handler); its last result is still fresh by time
  dead = threading.Thread(target=lambda: None)
  dead.start()
  dead.join()
  ld.hba._thread = dead
  ls = drive(ld, sm, runner, L.MODE_LOW_AND_HIGH_BEAM, outs[-1][0] + DT, DT)[-1][2]
  assert ld.hba.result() is not None and outs[-1][0] + DT - ld.hba.result().t < L.HBA_STALE_S
  assert ls.valid and not ls.starpilotLightState.hbaValid


def test_low_beam_mode_never_starts_hba():
  runner = FakeCameraRunner()
  ld = L.LightD(False, GPS, L.SolarTracker(NoParams()), L.HbaWorker(runner))
  outs = drive(ld, ld.submaster(), runner, L.MODE_LOW_BEAM, 0.0, 1.0)
  assert runner.calls == 0 and not ld.hba.alive
  assert not any(ls.starpilotLightState.hbaValid for _, _, ls in outs)


def test_message_validity_ignores_loop_rate():
  """The bug: an 8 Hz loop failed the poll service's 16 Hz average-rate check and card dropped the low beam."""
  ld = L.LightD(False, GPS, L.SolarTracker(NoParams()))
  sm = ld.submaster()
  strict = messaging.SubMaster(list(sm.services), poll=ld.poll)
  for i in range(40):
    t = i * 0.125
    for s in (sm, strict):
      s.update_msgs(t, inputs(t, i))
  assert not strict.all_checks([ld.poll])
  assert sm.all_checks([ld.poll])

  # a camera that stops is still caught
  sm.update_msgs(40 * 0.125 + 1.0, [])
  assert not sm.all_checks([ld.poll])
