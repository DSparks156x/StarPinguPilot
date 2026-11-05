#!/usr/bin/env python3
"""short_peaks.py <rlog or route dir>... [--csv out.csv]: the short-exposure radiance of every light lightd tracked.

Reads starpilotLightState's per-light lists (filled once per HBA frame; shortExposureValid says the frame had its
staggered-HDR short exposure) with our own high beam from starpilotCarState, and prints short-peak percentiles by what
the HBA called each light and by our beam, for white lights with a long-frame core of at least short_sign_min_area px
and for smaller ones. That is the measurement HbaParams.short_lamp_peak / short_sign_peak wait for: lit signs and
delineators under our high beam vs oncoming lamps (label those in hindsight, hba_tools/score.py). StarPinguPilot.
"""
import argparse
import csv
import math
import os
from collections import defaultdict

import numpy as np

from openpilot.starpilot.system.lightd.hba import HbaParams
from openpilot.tools.lib.logreader import LogReader

COLUMNS = ["file", "t", "frame", "high_beam", "tid", "u", "v", "area", "red", "short_peak", "evidence", "kind", "lamp"]


def rlogs(paths):
  for p in paths:
    if os.path.isdir(p):
      for root, _, files in sorted(os.walk(p)):
        yield from (os.path.join(root, f) for f in sorted(files) if f.startswith("rlog"))
    else:
      yield p


def rows(path):
  high_beam = None
  for m in LogReader(path, sort_by_time=True, only_union_types=True):
    w = m.which()
    if w == "starpilotCarState":
      high_beam = bool(m.starpilotCarState.highBeamOn)
    elif w == "starpilotLightState":
      ls = m.starpilotLightState
      if not ls.shortExposureValid or not len(ls.lightTrackId):
        continue
      for i in range(len(ls.lightTrackId)):
        yield [os.path.basename(path), m.logMonoTime * 1e-9, ls.hbaFrameId, high_beam, ls.lightTrackId[i], ls.lightU[i],
               ls.lightV[i], ls.lightArea[i], ls.lightRed[i], ls.lightShortPeak[i], ls.lightEvidence[i], ls.lightKind[i],
               ls.lightLamp[i]]


def pct(a):
  return " ".join(f"{q}:{np.percentile(a, q):6.0f}" for q in (5, 25, 50, 75, 95)) if len(a) else "-"


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("paths", nargs="+")
  ap.add_argument("--csv")
  args = ap.parse_args()
  p = HbaParams()
  out = open(args.csv, "w", newline="") if args.csv else None
  writer = csv.writer(out) if out else None
  if writer:
    writer.writerow(COLUMNS)
  groups = defaultdict(list)
  n = 0
  for f in rlogs(args.paths):
    for r in rows(f):
      n += 1
      if writer:
        writer.writerow(r)
      _, _, _, hb, _, _, _, area, red, peak, _, kind, _ = r
      if red or not math.isfinite(peak):
        continue
      size = "big" if area >= p.short_sign_min_area else "small"
      groups[(kind, "hb" if hb else "lb", size)].append(peak)
  if out:
    out.close()
  print(f"{n} light rows with a short exposure; white lights, short peak (RAW10 counts above black) percentiles")
  print(f"thresholds: lamp >= {p.short_lamp_peak:.0f}, sign <= {p.short_sign_peak:.0f} (big: core >= {p.short_sign_min_area} px)")
  for key in sorted(groups, key=lambda k: -len(groups[k])):
    a = np.array(groups[key])
    print(f"{key[0]:>22} {key[1]} {key[2]:>5}  n={len(a):6d}  {pct(a)}")


if __name__ == "__main__":
  main()
