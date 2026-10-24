#!/usr/bin/env python3
"""Replay logged drives through the HBA classifier and score it with hindsight labels. StarPinguPilot.

  replay_hba.py <route dir> [...] [--segs 3-6] [--out prefix] [--jpg N] [--clf other_hba.py] [--no-wide]

A route dir is the flat layout pingulink uploads: rlog--<n>.zst plus fcamera--<n>.hevc, and ecamera--<n>.hevc for the
wide guard if it was uploaded. Video is decoded with ffmpeg to NV12 and fed to the classifier at HBA_PERIOD_S like lightd
does (the wide frame every WIDE_PERIOD_S), with the model, calibration, speed, gyro, the car's own high-beam state
(Gate_Komf_1) and the mapd speed limit (if the log has mapdOut) at that frame.

Outputs:
  <out>_frames.csv   one row per processed frame: decision, reason, counts
  <out>_tracks.csv   one row per live track per processed frame, with its hindsight label
  <out>_wide.csv     one row per bright light the wide guard looked at, with its tests
  <out>_jpg/         annotated frames at every dip/restore and every --jpg seconds
and a summary on stdout.

Hindsight labels: far lights are ambiguous, near ones aren't. A track that ever resolves at short range (moving like
a vehicle, or static like the road, or a model lead) carries that label back over its whole history. Unresolved tracks
stay "unknown". The labels are only as good as the near-range rules, so look at the JPEGs before trusting a number.
"""
import argparse
import csv
import importlib.util
import inspect
import math
import os
import re
import subprocess
from collections import defaultdict

import cv2
import numpy as np

from cereal import log
from openpilot.starpilot.system.lightd.lightd import HBA_PERIOD_S, MODEL_MAX_FRAME_LAG, WIDE_PERIOD_S, mapd_speed_limit, model_dict
from openpilot.starpilot.system.lightd.solar import solar_position
from openpilot.tools.lib.logreader import LogReader

FRAME_COLS = ("seg", "idx", "t", "frame_id", "truth", "v", "dark", "high_beam", "permitted", "threat", "reason", "n_oncoming",
              "n_preceding", "n_candidates", "lead_prob", "town", "town_lights", "lamp", "n_wide", "ms")
WIDE_COLS = ("seg", "idx", "t", "u", "v", "area", "peak", "chroma", "ring", "reach", "lamp", "permitted")
TRACK_COLS = ("seg", "t", "frame_id", "tid", "u", "v", "pred_err", "area", "peak", "red", "paired", "post", "flux", "sat", "dist",
              "lateral", "kind", "why", "obj_speed", "age", "reflector", "threat", "label", "partner", "compact", "bend")
KIND_BGR = {"oncoming": (0, 0, 255), "preceding": (0, 165, 255), "candidate": (255, 0, 255), "static": (0, 200, 0)}


def load_hba(path):
  if path is None:
    from openpilot.starpilot.system.lightd import hba
    return hba
  spec = importlib.util.spec_from_file_location("hba_alt", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


def segments(route, segs):
  out = []
  for f in os.listdir(route):
    m = re.fullmatch(r"fcamera--(\d+)\.hevc", f)
    if m and (segs is None or int(m.group(1)) in segs):
      n = int(m.group(1))
      log = os.path.join(route, f"rlog--{n}.zst")
      wide = os.path.join(route, f"ecamera--{n}.hevc")
      if os.path.exists(log):
        out.append((n, log, os.path.join(route, f), wide if os.path.exists(wide) else None))
  return sorted(out)


def parse_segs(s):
  if s is None:
    return None
  r = set()
  for part in s.split(","):
    a, _, b = part.partition("-")
    r |= set(range(int(a), int(b or a) + 1))
  return r


class SegmentLog:
  """Everything the HBA needs from one rlog, indexed by camera frame or time."""

  def __init__(self, path):
    self.enc = {}          # segmentId -> (frameId, t)
    self.wide_enc = {}     # segmentId -> frameId, wide camera
    self.models = {}       # frameId -> model dict
    self.calib, self.car, self.pose, self.hb, self.limit = [], [], [], [], []
    self.gps = None
    for m in LogReader(path, sort_by_time=True, only_union_types=True):
      w, t = m.which(), m.logMonoTime * 1e-9
      if w == "roadEncodeIdx":
        e = m.roadEncodeIdx
        self.enc[e.segmentId] = (e.frameId, e.timestampEof * 1e-9)
      elif w == "wideRoadEncodeIdx":
        self.wide_enc[m.wideRoadEncodeIdx.segmentId] = m.wideRoadEncodeIdx.frameId
      elif w == "modelV2":
        self.models[m.modelV2.frameId] = model_dict(m.modelV2)
      elif w == "liveCalibration":
        c = m.liveCalibration
        calibrated = c.calStatus == log.LiveCalibrationData.Status.calibrated and len(c.rpyCalib) == 3
        wide = list(c.wideFromDeviceEuler) if calibrated and len(c.wideFromDeviceEuler) == 3 else None
        self.calib.append((t, list(c.rpyCalib) if calibrated else None, c.height[0] if len(c.height) else 1.22, wide))
      elif w == "carState":
        self.car.append((t, m.carState.vEgo))
      elif w == "livePose":
        a = m.livePose.angularVelocityDevice
        self.pose.append((t, a.x, a.y, a.z))
      elif w == "can":
        for f in m.can:
          if f.address == 0x390 and len(f.dat) == 8:
            self.hb.append((t, bool(f.dat[6] & 0x02)))  # GK1_Fernlicht, bit 49
      elif w == "mapdOut":
        self.limit.append((t, mapd_speed_limit(m.mapdOut)))
      elif w in ("gpsLocationExternal", "gpsLocation") and self.gps is None:
        g = getattr(m, w)
        if g.hasFix and g.unixTimestampMillis > 1.5e12:
          self.gps = (t, g.unixTimestampMillis / 1e3, g.latitude, g.longitude)
    self.model_ids = np.array(sorted(self.models))
    for name in ("calib", "car", "pose", "hb", "limit"):
      setattr(self, name + "_t", np.array([x[0] for x in getattr(self, name)]))

  def at(self, name, t):
    ts = getattr(self, name + "_t")
    i = np.searchsorted(ts, t, side="right") - 1
    return getattr(self, name)[i] if i >= 0 else None

  def model(self, frame_id):
    i = np.searchsorted(self.model_ids, frame_id, side="right") - 1
    if i < 0 or frame_id - self.model_ids[i] > MODEL_MAX_FRAME_LAG:
      return None
    return self.models[int(self.model_ids[i])]

  def sun(self, t):
    if self.gps is None:
      return None
    return solar_position(self.gps[2], self.gps[3], self.gps[1] + (t - self.gps[0]))[0]


def decode(path, w=1344, h=760):
  """Yields (y, uv) NV12 planes per frame."""
  proc = subprocess.Popen(["ffmpeg", "-v", "error", "-i", path, "-f", "rawvideo", "-pix_fmt", "nv12", "-"],
                          stdout=subprocess.PIPE, bufsize=w * h * 3)
  n = w * h * 3 // 2
  try:
    while True:
      buf = proc.stdout.read(n)
      if len(buf) < n:
        break
      a = np.frombuffer(buf, np.uint8)
      yield a[:w * h].reshape(h, w), a[w * h:].reshape(h // 2, w)
  finally:
    proc.kill()


def obvious_lamp(y, uv):
  """Area of the biggest saturated, near-white blob in the left two thirds of the band where lamps can be, ignoring the
  classifier entirely: a crude ground truth for "an oncoming car is right there". 0 if none."""
  h, w = y.shape
  band = slice(h // 3, h * 5 // 6)
  yy = y[band, :w * 2 // 3]
  m = cv2.threshold(yy, 229, 255, cv2.THRESH_BINARY)[1]  # NV12 Y is video range: 235 is white
  n, lab, st, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
  best = 0
  for i in range(1, n):
    x0, y0, bw, bh, area = st[i]
    if area < OBVIOUS_LAMP_AREA or area <= best or bh > 2.5 * bw:
      continue
    # chroma from the NV12 plane (2x2 blocks): far from neutral is a lit sign
    cu = uv[(band.start + y0) // 2:(band.start + y0 + bh) // 2 + 1, (x0 // 2) * 2:((x0 + bw) // 2 + 1) * 2].reshape(-1, 2)
    # and a lamp blooms: a white sign lit by our beams has a dark surround
    pad = max(bw, bh)
    ys, xs = slice(max(0, y0 - pad), y0 + bh + pad), slice(max(0, x0 - pad), x0 + bw + pad)
    ring = yy[ys, xs][lab[ys, xs] != i]
    if len(cu) and np.median(np.abs(cu.astype(int) - 128).sum(axis=1)) < 40 and len(ring) and np.median(ring) >= 40:
      best = int(area)
  return best


OBVIOUS_LAMP_AREA = 60


def annotate(path, y, uv, clf, out, info, wide=None):
  bgr = cv2.cvtColor(np.concatenate([y, uv]), cv2.COLOR_YUV2BGR_NV12)
  if wide is not None:
    # the wide frame on the right, with the lights the guard last looked at: red dips, grey failed a test
    wbgr = cv2.cvtColor(np.concatenate(wide), cv2.COLOR_YUV2BGR_NV12)
    for w in getattr(clf, "last_wide", []):
      cv2.circle(wbgr, (int(w.u), int(w.v)), 8 + int(math.sqrt(w.area)), (0, 0, 255) if w.lamp else (150, 150, 150), 2)
    bgr = np.concatenate([bgr, wbgr], axis=1)
  if clf.last_mask is not None:
    cs, _ = cv2.findContours(clf.last_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bgr, cs, -1, (90, 90, 90), 1)
  for tr in clf.tracks:
    if tr.missed:
      continue
    h = tr.hist[-1]
    col = KIND_BGR.get(tr.kind, (255, 255, 255))
    u, v = int(h[1]), int(h[2])
    r = 6 + int(math.sqrt(h[5]))
    cv2.rectangle(bgr, (u - r, v - r), (u + r, v + r), col, 2 if tr.is_threat(clf.p) else 1)
    sp = "" if not math.isfinite(tr.obj_speed) else f" {tr.obj_speed:+.0f}"
    d = "" if not math.isfinite(tr.dist) else f" {tr.dist:.0f}m"
    cv2.putText(bgr, f"{tr.tid}{d}{sp}", (u + r + 2, v), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
  cv2.putText(bgr, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255) if out.high_beam_permitted else (0, 0, 255), 2)
  cv2.imwrite(path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])


def label_tracks(rows):
  """Hindsight label per (seg, tid) from the track's near-range history."""
  by = defaultdict(list)
  for r in rows:
    by[(r["seg"], r["tid"])].append(r)
  labels = {}
  for key, rs in by.items():
    near = [r for r in rs if math.isfinite(r["dist"]) and r["dist"] < 60]
    moving = [r for r in near if math.isfinite(r["obj_speed"]) and abs(r["obj_speed"]) >= 6]
    still = [r for r in near if math.isfinite(r["obj_speed"]) and abs(r["obj_speed"]) < 3]
    red = sum(r["red"] for r in rs) * 2 > len(rs)
    if len(moving) >= 3 and len(moving) > len(still):
      labels[key] = "vehicle"
    elif len(still) >= 3 or sum(r["post"] for r in rs) * 2 > len(rs):
      labels[key] = "static"
    elif red and any(r["kind"] == "preceding" for r in near):
      labels[key] = "vehicle"
    else:
      labels[key] = "unknown"
  for r in rows:
    r["label"] = labels[(r["seg"], r["tid"])]
  return labels


def summarize(frames, tracks):
  fr = [f for f in frames if f["dark"]]
  if not fr:
    print("no dark frames")
    return
  dur = len(fr) * HBA_PERIOD_S
  threat = sum(f["threat"] for f in fr)
  dips = sum(1 for a, b in zip(fr, fr[1:], strict=False) if a["permitted"] and not b["permitted"])
  by_frame = defaultdict(list)
  for r in tracks:
    by_frame[(r["seg"], r["frame_id"])].append(r)
  why = defaultdict(int)
  for f in fr:
    if not f["threat"]:
      continue
    ts = [r for r in by_frame[(f["seg"], f["frame_id"])] if r["threat"]]
    labs = {r["label"] for r in ts}
    why["vehicle" if "vehicle" in labs else "unknown" if "unknown" in labs else "static" if labs else "model lead only"] += 1
  # glare: a hindsight vehicle visible while the decision (ignoring the speed gate) said clear
  glare = sum(1 for f in fr if not f["threat"] and any(r["label"] == "vehicle" for r in by_frame[(f["seg"], f["frame_id"])]))
  labels = defaultdict(set)
  for r in tracks:
    labels[r["label"]].add((r["seg"], r["tid"]))
  # how early vehicle tracks became threats: seconds between first threat and the track's last frame
  lead = []
  vt = defaultdict(list)
  for r in tracks:
    if r["label"] == "vehicle":
      vt[(r["seg"], r["tid"])].append(r)
  for rs in vt.values():
    first = next((r["t"] for r in rs if r["threat"]), None)
    lead.append(rs[-1]["t"] - first if first is not None else -1.0)
  print(f"dark: {dur:.0f} s ({len(fr)} frames), mean {np.mean([f['v'] for f in fr]) * 3.6:.0f} km/h, "
        f"high beam on {sum(f['high_beam'] == 1 for f in fr) * HBA_PERIOD_S:.0f} s")
  print(f"threat frames: {100 * threat / len(fr):.1f} %  dips/min: {dips / dur * 60:.1f}  "
        f"permitted: {100 * sum(f['permitted'] for f in fr) / len(fr):.1f} %")
  print("threat cause (frames): " + ", ".join(f"{k} {v}" for k, v in sorted(why.items())))
  print(f"glare frames (vehicle track live, no threat): {glare}")
  lamp = [f for f in fr if f.get("lamp", 0) >= OBVIOUS_LAMP_AREA]
  if lamp:
    print(f"OBVIOUS LAMP (big neutral blob, left 2/3, classifier-independent): {len(lamp) * HBA_PERIOD_S:.0f} s, "
          f"high beam permitted for {sum(f['permitted'] for f in lamp) * HBA_PERIOD_S:.1f} s of it")
  # town spans as segment seconds (video index / 20), split at gaps over a second
  spans = []
  for f in fr:
    if not f.get("town"):
      continue
    ts = f["idx"] / 20
    if spans and spans[-1][0] == f["seg"] and ts - spans[-1][2] <= 1.0:
      spans[-1][2] = ts
    else:
      spans.append([f["seg"], ts, ts])
  print(f"town: {sum(f.get('town', 0) for f in fr) * HBA_PERIOD_S:.0f} s  " +
        " ".join(f"{s}:{a:.0f}-{b:.0f}" for s, a, b in spans))
  print("tracks: " + ", ".join(f"{k} {len(v)}" for k, v in sorted(labels.items())))
  wide_only = [f for f in fr if f.get("n_wide") and not (f["n_oncoming"] or f["n_candidates"] or f["n_preceding"])]
  print(f"wide guard: lamps in {sum(1 for f in fr if f.get('n_wide')) * HBA_PERIOD_S:.1f} s, the only threat in "
        f"{len(wide_only) * HBA_PERIOD_S:.1f} s")
  hand = [f for f in fr if f["truth"] in ("vehicle", "none")]
  if hand:
    veh = [f for f in hand if f["truth"] == "vehicle"]
    none = [f for f in hand if f["truth"] == "none"]
    # misses: runs of >= 1 s inside vehicle intervals without a threat
    miss_s, run = 0.0, 0
    for a in veh + [None]:
      if a is not None and not a["threat"]:
        run += 1
        continue
      if run * HBA_PERIOD_S >= 1.0:
        miss_s += run * HBA_PERIOD_S
      run = 0
    print(f"HAND LABELS: vehicle {len(veh) * HBA_PERIOD_S:.0f} s, none {len(none) * HBA_PERIOD_S:.0f} s")
    if veh:
      print(f"  vehicle: threat {100 * np.mean([f['threat'] for f in veh]):.1f} %, permitted (glare) "
            f"{100 * np.mean([f['permitted'] for f in veh]):.1f} %, missed runs >= 1 s: {miss_s:.1f} s")
    if none:
      nd = sum(1 for a, b in zip(none, none[1:], strict=False) if a["permitted"] and not b["permitted"] and b["idx"] - a["idx"] <= 2)
      print(f"  none:    threat (nuisance) {100 * np.mean([f['threat'] for f in none]):.1f} %, permitted "
            f"{100 * np.mean([f['permitted'] for f in none]):.1f} %, dips {nd}, town "
            f"{100 * np.mean([f.get('town', 0) for f in none]):.1f} %")
  if lead:
    missed = sum(x < 0 for x in lead)
    ok = [x for x in lead if x >= 0]
    print(f"vehicle tracks: {len(lead)}, never a threat: {missed}, threat lead time median {np.median(ok) if ok else 0:.1f} s")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--segs")
  ap.add_argument("--out", default="hba")
  ap.add_argument("--jpg", type=float, default=0.0, help="also save an annotated frame every N seconds")
  ap.add_argument("--clf", help="alternative hba.py, e.g. a baseline copy")
  ap.add_argument("--speed-gate", action="store_true", help="keep the classifier's speed gate (default: off)")
  ap.add_argument("--dark", choices=("sun", "always"), default="sun")
  ap.add_argument("--no-gyro", action="store_true")
  ap.add_argument("--gyro-sign", type=float, default=1.0, help="debug: scale the gyro input")
  ap.add_argument("--no-beam", action="store_true")
  ap.add_argument("--no-wide", action="store_true", help="don't feed the wide camera even if the route has it")
  ap.add_argument("--labels", help="csv route,seg,t0,t1,label (vehicle/none/ambiguous), t in seconds into the segment")
  ap.add_argument("--at", help="also save these frames: seg:first-last seconds, e.g. 5:6-10,6:30-31")
  args = ap.parse_args()

  jpg_at = set()
  for part in (args.at or "").split(","):
    if part:
      s, _, rng = part.partition(":")
      a, _, b = rng.partition("-")
      jpg_at |= {(int(s), k) for k in range(int(float(a) * 10), int(float(b or a) * 10) + 1)}
  hba = load_hba(args.clf)
  intervals = []
  if args.labels:
    with open(args.labels) as f:
      intervals = [(r["route"], int(r["seg"]), float(r["t0"]), float(r["t1"]), r["label"]) for r in csv.DictReader(f)]

  def truth(rname, n, ts):
    return next((lab for r, s, a, b, lab in intervals if rname.startswith(r) and s == n and a <= ts < b), "")
  segs = parse_segs(args.segs)
  jpg_dir = args.out + "_jpg"
  os.makedirs(jpg_dir, exist_ok=True)
  frames, tracks, wides = [], [], []

  for route in args.routes:
    rname = os.path.basename(os.path.normpath(route))
    for n, log_path, video, wide_video in segments(route, segs):
      seg = f"{rname[:8]}/{n}"
      lg = SegmentLog(log_path)
      clf = hba.HbaClassifier()
      if not args.speed_gate:
        clf.p.enable_v = clf.p.disable_v = -1.0
      extras = set(inspect.signature(clf.update).parameters)
      last_run, last_jpg, last_permit, last_wide, wide_on = -math.inf, -math.inf, None, -math.inf, False
      wide_it = decode(wide_video) if wide_video and not args.no_wide and "wide" in extras else None
      wf = None
      for idx, (y, uv) in enumerate(decode(video)):
        # the wide video advances with the narrow one; wideRoadEncodeIdx says whether it's the same camera frame
        wf = next(wide_it, None) if wide_it is not None else None
        if idx not in lg.enc:
          continue
        fid, t = lg.enc[idx]
        if t - last_run < HBA_PERIOD_S - 1e-3:
          continue
        last_run = t
        cal, car, hb = lg.at("calib", t), lg.at("car", t), lg.at("hb", t)
        model = lg.model(fid)
        rpy = cal[1] if cal else None
        v = car[1] if car else 0.0
        sun = lg.sun(t)
        dark = args.dark == "always" or (sun is not None and sun < 0)
        kw = {}
        if "high_beam_on" in extras and not args.no_beam:
          kw["high_beam_on"] = None if hb is None else hb[1]
        if "ang_vel" in extras and not args.no_gyro:
          pose = lg.at("pose", t)
          kw["ang_vel"] = None if pose is None else [args.gyro_sign * w for w in pose[1:]]
        if "speed_limit" in extras:
          lim = lg.at("limit", t)  # pre-StarPilot logs have no mapdOut
          kw["speed_limit"] = lim[1] if lim is not None and t - lim[0] < 1.0 else None
        looked = (wf is not None and t - last_wide >= WIDE_PERIOD_S - 1e-3 and lg.wide_enc.get(idx) == fid
                  and cal is not None and cal[3] is not None)
        if looked:
          last_wide = t
          kw["wide"], kw["wide_euler"] = hba.Nv12Frame(*wf), cal[3]
        t0 = cv2.getTickCount()
        out = clf.update(t, hba.Nv12Frame(y, uv), model, rpy, cal[2] if cal else 1.22, v, dark=dark,
                         valid=model is not None and rpy is not None, **kw)
        ms = (cv2.getTickCount() - t0) / cv2.getTickFrequency() * 1e3
        frames.append({"seg": seg, "idx": idx, "truth": truth(rname, n, idx / 20), "t": t, "frame_id": fid, "v": v,
                       "dark": int(dark), "high_beam": -1 if hb is None else int(hb[1]), "permitted": int(out.high_beam_permitted),
                       "threat": int(out.threat), "reason": out.reason,
                       "n_oncoming": out.n_oncoming, "n_preceding": out.n_preceding, "n_candidates": out.n_candidates,
                       "lead_prob": out.lead_prob, "town": int(getattr(out, "town", False)),
                       "town_lights": round(getattr(out, "town_lights", 0.0), 2), "lamp": obvious_lamp(y, uv),
                       "n_wide": getattr(out, "n_wide", 0), "ms": ms})
        if looked and dark:
          wides += [{"seg": seg, "idx": idx, "t": t, **w._asdict(), "lamp": int(w.lamp), "permitted": int(out.high_beam_permitted)}
                    for w in clf.last_wide]
        for tr in clf.tracks:
          if tr.missed:
            continue
          h = tr.hist[-1]
          pe = math.hypot(h[1] - tr.pred[0], h[2] - tr.pred[1]) if getattr(tr, "pred", None) and len(tr.hist) > 1 else float("nan")
          tracks.append({"seg": seg, "t": t, "frame_id": fid, "tid": tr.tid, "u": h[1], "v": h[2], "pred_err": pe, "area": h[5], "peak": h[7],
                         "red": int(h[6]), "paired": int(h[9]), "post": int(h[10]), "flux": getattr(h, "flux", 0),
                         "sat": getattr(h, "sat", 0), "dist": tr.dist, "lateral": tr.lateral, "kind": tr.kind, "why": getattr(tr, "why", ""),
                         "obj_speed": tr.obj_speed, "age": getattr(tr, "age_s", getattr(tr, "age", 0)),
                         "reflector": int(getattr(tr, "reflector", False)), "threat": int(tr.is_threat(clf.p)),
                         "partner": int(getattr(h, "partner", False)), "compact": int(getattr(h, "compact", False)),
                         "bend": round(float(getattr(h, "bend", 0.0)), 2)})
        changed = last_permit is not None and out.high_beam_permitted != last_permit
        last_permit = out.high_beam_permitted
        wanted = (n, idx // 2) in jpg_at  # video index at 20 Hz, --at in tenths of a second
        wide_onset = bool(getattr(out, "n_wide", 0)) and not wide_on  # every time the wide guard starts a threat
        wide_on = bool(getattr(out, "n_wide", 0))
        if wanted or wide_onset or (dark and out.valid and (changed or (args.jpg and t - last_jpg >= args.jpg))):
          last_jpg = t
          info = (f"{seg} f{fid} {v * 3.6:.0f}kmh {'HIGH' if out.high_beam_permitted else 'low'} reason {out.reason} "
                  f"onc {out.n_oncoming} pre {out.n_preceding} cand {out.n_candidates} lead {out.lead_prob:.2f}")
          tag = ("_chg" if changed else "") + ("_wide" if wide_onset else "")
          annotate(os.path.join(jpg_dir, f"{rname[:8]}_{n:03d}_{idx:04d}{tag}.jpg"), y, uv, clf, out, info, wf)
      print(f"{seg}: {sum(1 for f in frames if f['seg'] == seg)} frames, "
            f"{np.mean([f['ms'] for f in frames if f['seg'] == seg]):.1f} ms/frame")

  label_tracks(tracks)
  for name, rows, cols in (("frames", frames, FRAME_COLS), ("tracks", tracks, TRACK_COLS), ("wide", wides, WIDE_COLS)):
    with open(f"{args.out}_{name}.csv", "w", newline="") as f:
      wr = csv.DictWriter(f, cols, extrasaction="ignore")
      wr.writeheader()
      wr.writerows(rows)
  summarize(frames, tracks)


if __name__ == "__main__":
  main()
