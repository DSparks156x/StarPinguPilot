#!/usr/bin/env python3
"""lightd: camera auto headlights + high beam assist for the Audi TT Mk2. StarPinguPilot.

Publishes starpilotLightState. card turns it into mFLA_1 (0x65A) for the patched J519 when VolkswagenLightdMode
is 2 (low beam) or 3 (low + high beam); mode 1 only logs. See DESIGN.md.

Low beam and HBA have independent validity, as the J519 contract requires:
  lowBeamValid: fresh, advancing camera AE state and a settled estimator.
  hbaValid:     a narrow-camera frame processed within HBA_STALE_S with a fresh model and a valid calibration.
                The wide camera only adds a glare guard every WIDE_PERIOD_S; without it the HBA still works.
The HBA runs on its own thread (HbaWorker), so its cost never slows the loop or the message's validity.
"""
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import UTC, datetime

import cv2
import numpy as np

import cereal.messaging as messaging
from cereal import custom, log
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.starpilot.system.lightd.ambient import AmbientEstimator, CameraSample
from openpilot.starpilot.system.lightd.hba import HbaClassifier, HbaOut, Nv12Frame, REASON_FAULT
from openpilot.starpilot.system.lightd.solar import solar_position

MODE_OFF, MODE_SHADOW, MODE_LOW_BEAM, MODE_LOW_AND_HIGH_BEAM = range(4)  # same as opendbc volkswagen/fla.py

HBA_PERIOD_S = 0.1     # run the HBA classifier at up to ~10 Hz
HBA_STALE_S = 0.3      # no processed frame for this long (slow or dead worker): HBA invalid
WIDE_PERIOD_S = 0.2    # the wide-camera glare guard looks this often (it only needs big lamps)
MODEL_MAX_FRAME_LAG = 5
WIDE_MAX_FRAME_LAG = 2  # the wide frame must be from about the same time as the narrow one
SOLAR_PERIOD_S = 1.0
GPS_CACHE_MAX_AGE_S = 7 * 24 * 3600  # a week-old position still puts sunset within minutes


def wall_time() -> float:
  """UTC seconds. The sun position needs the real time of day, not a monotonic clock."""
  return datetime.now(UTC).timestamp()


def xyz(p):
  return np.array([list(p.x), list(p.y), list(p.z)], dtype=np.float32).T


def model_dict(md):
  """The parts of modelV2 the HBA corridor uses, as numpy arrays."""
  return {
    "position": xyz(md.position),
    "lanes": [xyz(line) for line in md.laneLines],
    "lane_probs": list(md.laneLineProbs),
    "leads": [(lead.prob, lead.x[0], lead.y[0]) for lead in md.leadsV3 if len(lead.x)],
  }


def mapd_speed_limit(mapd) -> float:
  """mapd's posted limit (m/s) for the way it matched us to, 0 when it only guesses the way or has no limit."""
  matched = mapd.waySelectionType in (custom.WaySelectionType.current, custom.WaySelectionType.extended)
  return float(mapd.speedLimit) if matched else 0.0


class SolarTracker:
  """Sun position from live GPS, falling back to the last known position and the system clock."""

  def __init__(self, params):
    self.params = params
    self.lat = self.lon = None
    self.elev = self.azim = None
    self._last_calc = -math.inf
    self._load_cached()

  def _load_cached(self):
    try:
      raw = self.params.get("LastGPSPosition")
      pos = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
      if pos and wall_time() - float(pos.get("updatedAtSec", 0)) < GPS_CACHE_MAX_AGE_S:
        self.lat, self.lon = float(pos["latitude"]), float(pos["longitude"])
    except Exception:
      cloudlog.exception("lightd: bad LastGPSPosition")

  def update(self, now, gps):
    if gps is not None and gps.hasFix:
      self.lat, self.lon = gps.latitude, gps.longitude
    if now - self._last_calc < SOLAR_PERIOD_S:
      return
    self._last_calc = now
    if self.lat is None or not system_time_valid():
      self.elev = self.azim = None
      return
    self.elev, self.azim = solar_position(self.lat, self.lon, wall_time())


@dataclass
class HbaContext:
  """What the HBA needs from lightd's loop, captured on the main thread. capnp readers are immutable."""
  now: float
  dark: bool
  calib: log.LiveCalibrationData
  model: log.ModelDataV2
  model_alive: bool
  car_alive: bool
  v_ego: float
  high_beam_on: bool | None = None          # our high beam as the car reports it (TT: Gate_Komf_1), for the reflector cue
  ang_vel: tuple[float, float, float] | None = None  # livePose device-frame angular velocity, rad/s
  speed_limit: float | None = None          # mapd posted limit, m/s, for the town gate


@dataclass
class HbaResult:
  t: float                # lightd clock of the context the frame was processed with
  out: HbaOut | None      # None: the classifier raised
  frame_id: int
  processing_ms: float


class HbaRunner:
  """Pulls the newest narrow-camera frame and runs the HBA classifier on it, with the newest wide-camera frame every
  WIDE_PERIOD_S in the dark (wide=False: never, e.g. DisableWideRoad)."""

  def __init__(self, wide: bool = True):
    self.client = None
    self.wide_client = None
    self.use_wide = wide
    self.clf = HbaClassifier()
    self.last_run = -math.inf
    self.last_wide_run = -math.inf
    self.last_frame_id = 0

  def _connect(self):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    if self.client is None:
      self.client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    if not self.client.is_connected():
      self.client.connect(False)
    return self.client.is_connected()

  def _latest_frame(self):
    return self._drain(self.client)

  def _latest_wide(self):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    if self.wide_client is None:
      self.wide_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, True)
    if not self.wide_client.is_connected():
      self.wide_client.connect(False)
    return self._drain(self.wide_client) if self.wide_client.is_connected() else (None, None)

  @staticmethod
  def _drain(client):
    buf = None
    while True:
      b = client.recv(timeout_ms=0)
      if b is None:
        break
      buf = b
    if buf is None:
      return None, None
    # left in NV12: the classifier converts only the corridor's box
    data = np.frombuffer(buf.data, dtype=np.uint8)
    w, h, stride = buf.width, buf.height, buf.stride
    y = data[:h * stride].reshape(h, stride)[:, :w]
    uv = data[buf.uv_offset:buf.uv_offset + h // 2 * stride].reshape(h // 2, stride)[:, :w]
    return Nv12Frame(y, uv), client.frame_id

  def update(self, ctx: HbaContext) -> HbaResult | None:
    """None when nothing was processed (not due yet, no camera, no new frame)."""
    if ctx.now - self.last_run < HBA_PERIOD_S:
      return None
    if not self._connect():
      return None
    t0 = time.monotonic()
    frame, frame_id = self._latest_frame()
    if frame is None:
      return None
    calib, md = ctx.calib, ctx.model
    calibrated = calib.calStatus == log.LiveCalibrationData.Status.calibrated and len(calib.rpyCalib) == 3
    model_fresh = ctx.model_alive and abs(int(frame_id) - int(md.frameId)) <= MODEL_MAX_FRAME_LAG
    valid = bool(calibrated and model_fresh and ctx.car_alive)
    height = calib.height[0] if len(calib.height) else 1.22
    wide = wide_euler = None
    if (self.use_wide and valid and ctx.dark and ctx.now - self.last_wide_run >= WIDE_PERIOD_S
        and len(calib.wideFromDeviceEuler) == 3):
      wf, wide_id = self._latest_wide()
      if wf is not None and abs(int(wide_id) - int(frame_id)) <= WIDE_MAX_FRAME_LAG:
        wide, wide_euler = wf, list(calib.wideFromDeviceEuler)
        self.last_wide_run = ctx.now
    out = self.clf.update(ctx.now, frame, model_dict(md) if valid else None, list(calib.rpyCalib) if calibrated else None,
                          height, ctx.v_ego, dark=ctx.dark, valid=valid, high_beam_on=ctx.high_beam_on, ang_vel=ctx.ang_vel,
                          speed_limit=ctx.speed_limit, wide=wide, wide_euler=wide_euler)
    self.last_run = ctx.now
    self.last_frame_id = int(frame_id)
    return HbaResult(ctx.now, out, self.last_frame_id, (time.monotonic() - t0) * 1e3)


class HbaWorker:
  """Runs the HBA on its own thread, at whatever rate it manages. lightd's loop hands it the newest context and
  reads the newest result, never waiting: a slow or dead HBA only lets hbaValid lapse, the low beam keeps going."""

  def __init__(self, runner: HbaRunner | None = None):
    self.runner = runner or HbaRunner()
    self._lock = threading.Lock()
    self._wake = threading.Event()
    self._ctx: HbaContext | None = None
    self._result: HbaResult | None = None
    self._thread: threading.Thread | None = None

  def submit(self, ctx: HbaContext):
    with self._lock:
      self._ctx = ctx
    self._wake.set()
    if self._thread is None:
      self._thread = threading.Thread(target=self._run, name="lightd_hba", daemon=True)
      self._thread.start()

  @property
  def alive(self) -> bool:
    return self._thread is not None and self._thread.is_alive()

  def result(self) -> HbaResult | None:
    with self._lock:
      return self._result

  def _run(self):
    # Pin this thread (pid 0 from a thread is the thread on Linux) to the little cores, where openpilot keeps the
    # non-realtime daemons like locationd/paramsd. card/controlsd (4) and plannerd/radard (5) are on the big cores,
    # modeld on 7, 6-7 are isolated. The HBA needs ~10-14 % of one A55 at 10 Hz.
    try:
      os.sched_setaffinity(0, {0, 1, 2, 3})
    except (OSError, AttributeError):
      cloudlog.warning("lightd: could not pin hba thread to cores 0-3")
    while True:
      self._wake.wait()
      self._wake.clear()
      with self._lock:
        ctx, self._ctx = self._ctx, None
      if ctx is None:
        continue
      try:
        res = self.runner.update(ctx)
      except Exception:
        cloudlog.exception("lightd: hba failed")
        res = HbaResult(ctx.now, None, self.runner.last_frame_id, 0.0)
      if res is not None:
        with self._lock:
          self._result = res


class LightD:
  def __init__(self, wide_disabled: bool, gps_service: str, solar: SolarTracker, hba: HbaWorker | None = None):
    self.wide_disabled = wide_disabled
    self.poll = "roadCameraState" if wide_disabled else "wideRoadCameraState"
    self.gps_service = gps_service
    self.ambient = AmbientEstimator()
    self.solar = solar
    self.hba = hba or HbaWorker(HbaRunner(wide=not wide_disabled))
    self.last_frame = {"wideRoadCameraState": -1, "roadCameraState": -1}
    self.last_sample: dict[str, CameraSample | None] = {"wideRoadCameraState": None, "roadCameraState": None}

  def submaster(self):
    # No average-rate check on the poll service: it would measure this loop, not the cameras. Camera freshness is
    # covered by alive here and by the estimator's frame ages (lowBeamValid); card checks this message's own age.
    return messaging.SubMaster(["wideRoadCameraState", "roadCameraState", self.gps_service, "carState",
                                "starpilotCarState", "modelV2", "liveCalibration", "livePose", "starpilotPlan", "mapdOut"],
                               poll=self.poll, ignore_avg_freq=[self.poll])

  def step(self, sm, mode: int):
    now = sm.logMonoTime[self.poll] * 1e-9

    # camera samples only count while their frameId advances; a frozen camera goes stale
    for svc in self.last_frame:
      if sm.updated[svc] and sm.valid[svc]:
        cs = sm[svc]
        if cs.frameId != self.last_frame[svc]:
          self.last_frame[svc] = cs.frameId
          self.last_sample[svc] = CameraSample(sm.logMonoTime[svc] * 1e-9, cs.measuredGreyFraction, cs.exposureValPercent,
                                               cs.frameId)
    wide = None if self.wide_disabled else self.last_sample["wideRoadCameraState"]
    narrow = self.last_sample["roadCameraState"]

    solar = self.solar
    solar.update(now, sm[self.gps_service] if sm.alive[self.gps_service] else None)
    fpcs = sm["starpilotCarState"]
    amb = self.ambient.update(now, wide, narrow, solar.elev, sm["carState"].vEgo, fpcs.lowBeamOn)

    run_hba = mode in (MODE_SHADOW, MODE_LOW_AND_HIGH_BEAM)
    if run_hba:
      w = sm["livePose"].angularVelocityDevice
      ang_vel = (w.x, w.y, w.z) if sm.alive["livePose"] and w.valid else None
      high_beam = bool(fpcs.highBeamOn) if sm.alive["starpilotCarState"] else None
      limit = mapd_speed_limit(sm["mapdOut"]) if sm.alive["mapdOut"] else None
      self.hba.submit(HbaContext(now, bool(amb.valid and amb.dark_request), sm["liveCalibration"], sm["modelV2"],
                                 sm.alive["modelV2"], sm.alive["carState"], sm["carState"].vEgo, high_beam, ang_vel, limit))
    res = self.hba.result()
    fresh = run_hba and self.hba.alive and res is not None and now - res.t < HBA_STALE_S
    ho = res.out if fresh else None
    hba_valid = ho is not None and ho.valid

    msg = messaging.new_message("starpilotLightState", valid=sm.all_checks([self.poll]))
    ls = msg.starpilotLightState
    ls.mode = mode
    ls.lowBeamValid = bool(amb.valid)
    ls.darkRequest = bool(amb.dark_request)
    ls.brightnessCode = int(amb.bucket)
    for field, value in (("ambientLog2Lux", amb.ambient_b), ("fastLog2Lux", amb.fast_b), ("slowLog2Lux", amb.slow_b),
                         ("wideLog2Lux", amb.wide_b), ("narrowLog2Lux", amb.narrow_b), ("tunnelCue", amb.tunnel_cue)):
      setattr(ls, field, float(value) if math.isfinite(value) else 0.0)
    ls.darkSlow, ls.darkTunnel, ls.nightLatch = bool(amb.dark_slow), bool(amb.dark_tunnel), bool(amb.night_latch)
    ls.confidence = float(amb.confidence)
    ls.regime = int(amb.regime)
    ls.solarValid = solar.elev is not None
    ls.solarElevationDeg = float(solar.elev or 0.0)
    ls.solarAzimuthDeg = float(solar.azim or 0.0)
    ls.hbaValid = bool(hba_valid)
    ls.highBeamPermitted = bool(hba_valid and ho.high_beam_permitted)
    ls.hbaReason = int(ho.reason) if hba_valid else REASON_FAULT
    ls.oncomingCount = int(ho.n_oncoming) if ho else 0
    ls.precedingCount = int(ho.n_preceding) if ho else 0
    ls.candidateCount = int(ho.n_candidates) if ho else 0
    ls.hbaClearForS = float(ho.clear_for_s) if ho else 0.0
    ls.hbaFrameId = res.frame_id if res else 0
    ls.hbaProcessingMs = float(res.processing_ms) if res else 0.0
    for field, svc in (("wideFrameAgeMs", "wideRoadCameraState"), ("narrowFrameAgeMs", "roadCameraState")):
      s = self.last_sample[svc]
      setattr(ls, field, float((now - s.t) * 1e3) if s is not None else -1.0)
    return msg


def main():
  # imported here so LightD can be tested without the compiled params module and the full toggle stack
  from openpilot.common.gps import get_gps_location_service
  from openpilot.common.params import Params
  from openpilot.starpilot.common.starpilot_variables import get_starpilot_toggles

  # lightd runs at nice 10 next to the realtime daemons. One OpenCV thread: a thread pool on contended cores waits
  # for its slowest worker on every call, and the HBA's images are small strips.
  cv2.setNumThreads(1)
  params = Params()
  lightd = LightD(params.get_bool("DisableWideRoad"), get_gps_location_service(params), SolarTracker(params))
  sm = lightd.submaster()
  pm = messaging.PubMaster(["starpilotLightState"])
  while True:
    sm.update()
    mode = int(getattr(get_starpilot_toggles(sm), "volkswagen_lightd_mode", MODE_SHADOW))
    pm.send("starpilotLightState", lightd.step(sm, mode))


if __name__ == "__main__":
  main()
