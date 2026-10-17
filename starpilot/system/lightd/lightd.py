#!/usr/bin/env python3
"""lightd: camera auto headlights + high beam assist for the Audi TT Mk2. StarPinguPilot.

Publishes starpilotLightState. card turns it into mFLA_1 (0x65A) for the patched J519 when VolkswagenLightdMode
is 2 (low beam) or 3 (low + high beam); mode 1 only logs. See DESIGN.md.

Low beam and HBA have independent validity, as the J519 contract requires:
  lowBeamValid: fresh, advancing camera AE state and a settled estimator.
  hbaValid:     a narrow-camera frame processed within HBA_STALE_S with a fresh model and a valid calibration.
"""
import json
import math
import time
from datetime import UTC, datetime

import cv2
import numpy as np

import cereal.messaging as messaging
from cereal import log
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import system_time_valid
from openpilot.starpilot.common.starpilot_variables import get_starpilot_toggles
from openpilot.starpilot.system.lightd.ambient import AmbientEstimator, CameraSample
from openpilot.starpilot.system.lightd.hba import HbaClassifier, REASON_FAULT
from openpilot.starpilot.system.lightd.solar import solar_position

MODE_OFF, MODE_SHADOW, MODE_LOW_BEAM, MODE_LOW_AND_HIGH_BEAM = range(4)  # same as opendbc volkswagen/fla.py

HBA_PERIOD_S = 0.1     # run the HBA classifier at ~10 Hz
HBA_STALE_S = 0.3      # no processed frame for this long: HBA invalid
MODEL_MAX_FRAME_LAG = 5
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
    "edges": [xyz(edge) for edge in md.roadEdges],
    "leads": [(lead.prob, lead.x[0], lead.y[0]) for lead in md.leadsV3 if len(lead.x)],
  }


class SolarTracker:
  """Sun position from live GPS, falling back to the last known position and the system clock."""

  def __init__(self, params: Params):
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


class HbaRunner:
  """Pulls the newest narrow-camera frame and runs the HBA classifier on it."""

  def __init__(self):
    self.client = None
    self.clf = HbaClassifier()
    self.last_run = -math.inf
    self.last_out = None
    self.last_frame_id = 0
    self.processing_ms = 0.0

  def _connect(self):
    from msgq.visionipc import VisionIpcClient, VisionStreamType
    if self.client is None:
      self.client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    if not self.client.is_connected():
      self.client.connect(False)
    return self.client.is_connected()

  def _latest_bgr(self):
    buf = None
    while True:
      b = self.client.recv(timeout_ms=0)
      if b is None:
        break
      buf = b
    if buf is None:
      return None, None
    c = self.client
    img = np.frombuffer(buf.data, dtype=np.uint8).reshape((len(buf.data) // c.stride, c.stride))
    return cv2.cvtColor(img[:c.height * 3 // 2, :c.width], cv2.COLOR_YUV2BGR_NV12), c.frame_id

  def update(self, now, sm, dark):
    if now - self.last_run < HBA_PERIOD_S:
      return
    if not self._connect():
      return
    t0 = time.monotonic()
    bgr, frame_id = self._latest_bgr()
    if bgr is None:
      return
    calib = sm["liveCalibration"]
    md = sm["modelV2"]
    calibrated = calib.calStatus == log.LiveCalibrationData.Status.calibrated and len(calib.rpyCalib) == 3
    model_fresh = sm.alive["modelV2"] and abs(int(frame_id) - int(md.frameId)) <= MODEL_MAX_FRAME_LAG
    valid = bool(calibrated and model_fresh and sm.alive["carState"])
    height = calib.height[0] if len(calib.height) else 1.22
    self.last_out = self.clf.update(now, bgr, model_dict(md) if valid else None,
                                    list(calib.rpyCalib) if calibrated else None, height, sm["carState"].vEgo,
                                    dark=dark, valid=valid)
    self.last_run = now
    self.last_frame_id = int(frame_id)
    self.processing_ms = (time.monotonic() - t0) * 1e3


def main():
  params = Params()
  wide_disabled = params.get_bool("DisableWideRoad")
  poll = "roadCameraState" if wide_disabled else "wideRoadCameraState"
  gps_service = get_gps_location_service(params)
  sm = messaging.SubMaster(["wideRoadCameraState", "roadCameraState", gps_service, "carState", "starpilotCarState",
                            "modelV2", "liveCalibration", "starpilotPlan"], poll=poll)
  pm = messaging.PubMaster(["starpilotLightState"])

  ambient = AmbientEstimator()
  solar = SolarTracker(params)
  hba = HbaRunner()
  last_frame = {"wideRoadCameraState": -1, "roadCameraState": -1}
  last_sample: dict[str, CameraSample | None] = {"wideRoadCameraState": None, "roadCameraState": None}

  while True:
    sm.update()
    now = sm.logMonoTime[poll] * 1e-9
    mode = int(getattr(get_starpilot_toggles(sm), "volkswagen_lightd_mode", MODE_SHADOW))

    # camera samples only count while their frameId advances; a frozen camera goes stale
    for svc in last_frame:
      if sm.updated[svc] and sm.valid[svc]:
        cs = sm[svc]
        if cs.frameId != last_frame[svc]:
          last_frame[svc] = cs.frameId
          last_sample[svc] = CameraSample(sm.logMonoTime[svc] * 1e-9, cs.measuredGreyFraction, cs.exposureValPercent,
                                          cs.frameId)
    wide = None if wide_disabled else last_sample["wideRoadCameraState"]
    narrow = last_sample["roadCameraState"]

    solar.update(now, sm[gps_service] if sm.alive[gps_service] else None)
    fpcs = sm["starpilotCarState"]
    amb = ambient.update(now, wide, narrow, solar.elev, sm["carState"].vEgo, fpcs.lowBeamOn)

    run_hba = mode in (MODE_SHADOW, MODE_LOW_AND_HIGH_BEAM)
    if run_hba:
      try:
        hba.update(now, sm, dark=amb.valid and amb.dark_request)
      except Exception:
        cloudlog.exception("lightd: hba failed")
        hba.last_out = None
    ho = hba.last_out
    hba_valid = run_hba and ho is not None and ho.valid and now - hba.last_run < HBA_STALE_S

    msg = messaging.new_message("starpilotLightState", valid=sm.all_checks([poll]))
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
    ls.hbaFrameId = hba.last_frame_id
    ls.hbaProcessingMs = float(hba.processing_ms)
    for field, svc in (("wideFrameAgeMs", "wideRoadCameraState"), ("narrowFrameAgeMs", "roadCameraState")):
      s = last_sample[svc]
      setattr(ls, field, float((now - s.t) * 1e3) if s is not None else -1.0)
    pm.send("starpilotLightState", msg)


if __name__ == "__main__":
  main()
