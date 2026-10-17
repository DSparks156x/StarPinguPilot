#!/usr/bin/env python3
"""Replay logged drives through the lightd ambient estimator. StarPinguPilot.

  replay_lightd.py <segment dir | rlog file> [...] --out out_prefix

Writes <out>.csv (one row per wide camera frame) and <out>.png. Needs only the rlogs: camera AE state,
GPS and the TT light frames on CAN (Gate_Komf_1 0x390, Gate_Komf_2 0x392).
"""
import argparse
import csv
import math
import os
import re

from openpilot.starpilot.system.lightd.ambient import AmbientEstimator, CameraSample, LUX_PER_BUCKET
from openpilot.starpilot.system.lightd.solar import solar_position
from openpilot.tools.lib.logreader import LogReader

COLUMNS = ("t", "wide_b", "narrow_b", "fast_b", "slow_b", "on_threshold", "dark_request", "dark_tunnel", "valid", "regime",
           "confidence", "bucket", "tunnel_cue", "solar_elev", "v", "car_low_beam", "car_high_beam", "factory_bucket", "factory_b")


def rlog_paths(inputs):
  """Segment dirs (<route>--<n>/rlog.zst), flat route dirs (<route>/rlog--<n>.zst) or rlog files."""
  paths = []
  for p in inputs:
    if not os.path.isdir(p):
      paths.append(p)
    elif os.path.exists(os.path.join(p, "rlog.zst")):
      paths.append(os.path.join(p, "rlog.zst"))
    else:
      paths += [os.path.join(p, f) for f in os.listdir(p) if re.fullmatch(r"rlog--\d+\.zst", f)]

  def seg_key(path):
    m = re.search(r"([0-9a-f]+--[0-9a-f]+)(?:--|/rlog--)(\d+)", path)
    return (m.group(1), int(m.group(2))) if m else (path, 0)
  return sorted(paths, key=seg_key)


def replay(paths):
  est = AmbientEstimator()
  wide = narrow = None
  gps_ref = None  # (logMonoTime s, unix s, lat, lon)
  v = 0.0
  low_beam = high_beam = factory_bucket = None
  rows = []
  t0 = None

  for path in paths:
    for m in LogReader(path, sort_by_time=True, only_union_types=True):
      w = m.which()
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t

      if w == "roadCameraState":
        c = m.roadCameraState
        narrow = CameraSample(t, c.measuredGreyFraction, c.exposureValPercent, c.frameId)
      elif w in ("gpsLocationExternal", "gpsLocation"):
        g = getattr(m, w)
        if g.hasFix and g.unixTimestampMillis > 1.5e12:
          gps_ref = (t, g.unixTimestampMillis / 1000.0, g.latitude, g.longitude)
      elif w == "carState":
        v = m.carState.vEgo
      elif w == "can":
        for f in m.can:
          if f.address == 0x390 and len(f.dat) == 8:
            low_beam = bool(f.dat[6] & 0x01)    # GK1_Abblendlicht, bit 48
            high_beam = bool(f.dat[6] & 0x02)   # GK1_Fernlicht, bit 49
          elif f.address == 0x392 and len(f.dat) >= 3:
            factory_bucket = f.dat[2] & 0x07    # GK2_Helligkeit, 16|3, x714.286 lux
      elif w == "wideRoadCameraState":
        c = m.wideRoadCameraState
        wide = CameraSample(t, c.measuredGreyFraction, c.exposureValPercent, c.frameId)

        elev = None
        if gps_ref is not None:
          elev = solar_position(gps_ref[2], gps_ref[3], gps_ref[1] + (t - gps_ref[0]))[0]
        out = est.update(t, wide, narrow, elev, v, bool(low_beam))
        factory_b = math.log2((factory_bucket + 0.5) * LUX_PER_BUCKET) if factory_bucket is not None else float("nan")
        rows.append((t - t0, out.wide_b, out.narrow_b, out.fast_b, out.slow_b, out.on_threshold, int(out.dark_request),
                     int(out.dark_tunnel), int(out.valid),
                     out.regime, out.confidence, out.bucket, out.tunnel_cue, float("nan") if elev is None else elev, v,
                     -1 if low_beam is None else int(low_beam), -1 if high_beam is None else int(high_beam),
                     -1 if factory_bucket is None else factory_bucket, factory_b))
  return rows


def plot(rows, png, title):
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  import numpy as np

  a = np.array(rows, dtype=float)
  col = {n: a[:, i] for i, n in enumerate(COLUMNS)}
  t = col["t"] / 60.0
  fig, (ax, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
  ax.plot(t, col["wide_b"], lw=0.5, alpha=0.5, label="wide raw")
  ax.plot(t, col["narrow_b"], lw=0.5, alpha=0.5, label="narrow raw")
  ax.plot(t, col["slow_b"], lw=1.2, color="k", label="slow channel")
  ax.plot(t, col["fast_b"], lw=0.8, color="tab:green", label="fast channel")
  on = np.where(np.isfinite(col["on_threshold"]), col["on_threshold"], np.nan)
  ax.plot(t, on, "--", color="tab:red", lw=1, label="on threshold")
  ax.plot(t, col["factory_b"], color="tab:purple", lw=1, label="G397 bucket (GK2_Helligkeit)")
  ax.set_ylabel("log2 lux (uncalibrated)")
  ax.legend(loc="upper right", fontsize=8)
  ax.grid(alpha=0.3)
  ax.set_title(title)

  ax2.step(t, col["dark_request"], where="post", label="lightd dark request")
  ax2.step(t, col["dark_tunnel"] * 0.95, where="post", label="lightd tunnel channel")
  ax2.step(t, np.clip(col["car_low_beam"], 0, 1) * 0.9, where="post", label="car low beam (GK1)")
  ax2.step(t, np.clip(col["car_high_beam"], 0, 1) * 0.8, where="post", label="car high beam (GK1)")
  ax2.set_yticks([0, 1])
  ax2.set_xlabel("minutes")
  ax2.legend(loc="upper left", fontsize=8)
  axs = ax2.twinx()
  axs.plot(t, col["solar_elev"], color="tab:orange", lw=1)
  axs.set_ylabel("sun elev [deg]", color="tab:orange")
  fig.tight_layout()
  fig.savefig(png, dpi=110)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("inputs", nargs="+")
  ap.add_argument("--out", required=True)
  args = ap.parse_args()

  paths = rlog_paths(args.inputs)
  rows = replay(paths)
  with open(args.out + ".csv", "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(COLUMNS)
    wr.writerows(rows)
  plot(rows, args.out + ".png", os.path.basename(args.out))
  print(f"{len(rows)} frames from {len(paths)} segments -> {args.out}.csv/.png")


if __name__ == "__main__":
  main()
