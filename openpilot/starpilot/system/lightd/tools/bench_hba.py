#!/usr/bin/env python3
"""Time the HBA per stage on a synthetic night frame. StarPinguPilot.

  bench_hba.py [--busy] [--frames N]

Run it on the device (onroad or off) to see where hbaProcessingMs goes. Each stage runs with OpenCV on one thread
(as lightd does) and on its default pool. The "old full-frame" rows are the ops the corridor crop removed.
"""
import argparse
import os
import time

import cv2
import numpy as np

from openpilot.starpilot.system.lightd.hba import (FCAM_H, FCAM_W, Geometry, HbaClassifier, HbaParams, Nv12Frame,
                                                    corridor_mask, find_blobs, road_centerline)

HEIGHT = 1.22
RPY = [0.0, 0.02, 0.01]
XS = np.linspace(0, 190, 33)
MODEL = {
  "position": np.column_stack([XS, 0.0004 * XS ** 2, np.zeros(33)]),
  "lanes": [np.column_stack([XS, 0.0004 * XS ** 2 + y, np.full(33, HEIGHT)]) for y in (-5.4, -1.8, 1.8, 5.4)],
  "lane_probs": [0.5, 0.9, 0.9, 0.5],
  "edges": [],
  "leads": [(0.0, 50.0, 0.0)],
}


def night_nv12(busy):
  rng = np.random.default_rng(0)
  bgr = np.clip(rng.normal(14, 6, (FCAM_H, FCAM_W, 3)), 0, 255).astype(np.uint8)
  geom = Geometry(RPY, HEIGHT)
  lights = [(120, -3.6, 0.7, 3, (255, 255, 255)), (125, -2.2, 0.7, 3, (255, 255, 255)), (60, 0.0, 0.8, 2, (30, 30, 250)),
            (60, 1.5, 0.8, 2, (30, 30, 250)), (40, 4.0, 0.9, 1, (240, 240, 240)), (80, 4.0, 0.9, 1, (240, 240, 240))]
  if busy:  # town: house, street and parked-car lights in the corridor
    lights += [(rng.uniform(20, 250), rng.uniform(-8, 3.5), rng.uniform(0.3, 1.5), int(rng.integers(1, 4)),
                (255, 255, 255) if rng.random() < 0.7 else (30, 30, 250)) for _ in range(50)]
  for x, y, z, r, c in lights:
    uv, ok = geom.project(np.array([[x, y + 0.0004 * x ** 2, HEIGHT - z]]))
    if ok[0]:
      cv2.circle(bgr, (int(round(uv[0, 0])), int(round(uv[0, 1]))), r, c, -1)
  yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
  u = yuv[FCAM_H:FCAM_H + FCAM_H // 4].reshape(FCAM_H // 2, FCAM_W // 2)
  v = yuv[FCAM_H + FCAM_H // 4:].reshape(FCAM_H // 2, FCAM_W // 2)
  return np.vstack([yuv[:FCAM_H], np.stack([u, v], axis=-1).reshape(FCAM_H // 2, FCAM_W)])


def timed(fn, n):
  fn()
  ts = []
  for _ in range(n):
    t0 = time.perf_counter()
    fn()
    ts.append((time.perf_counter() - t0) * 1e3)
  return np.median(ts), np.percentile(ts, 90)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--busy", action="store_true")
  ap.add_argument("--frames", type=int, default=100)
  args = ap.parse_args()

  nv12 = night_nv12(args.busy)
  frame = Nv12Frame(nv12[:FCAM_H], nv12[FCAM_H:])
  p = HbaParams()
  geom = Geometry(RPY, HEIGHT)
  center = road_centerline(MODEL, p, HEIGHT)
  mask = corridor_mask(geom, center, p)
  full_mask = mask.copy()
  k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * p.margin_px + 1, 2 * p.margin_px + 1))
  clf = HbaClassifier()
  t = [0.0]

  def whole():
    t[0] += 0.1
    clf.update(t[0], frame, MODEL, RPY, HEIGHT, 25.0)

  stages = [
    ("classifier update (all)", whole),
    ("  corridor mask", lambda: corridor_mask(geom, road_centerline(MODEL, p, HEIGHT), p)),
    ("  blobs (NV12 crop + label)", lambda: find_blobs(frame, mask, p)),
    ("old full-frame NV12->BGR", lambda: cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)),
    ("old full-frame dilate", lambda: cv2.dilate(full_mask, k)),
  ]
  print(f"cpus allowed {sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else '?'}, nice {os.nice(0)}, "
        f"corridor box {cv2.boundingRect(mask)}, blobs {len(find_blobs(frame, mask, p))}")
  default_threads = cv2.getNumThreads()
  print(f"{'stage':30s} {'1 thread':>18s} {f'{default_threads} threads':>18s}   (median / p90 ms)")
  for name, fn in stages:
    cv2.setNumThreads(1)
    one = timed(fn, args.frames)
    cv2.setNumThreads(default_threads)
    many = timed(fn, args.frames)
    print(f"{name:30s} {one[0]:8.2f} / {one[1]:6.2f}  {many[0]:8.2f} / {many[1]:6.2f}")


if __name__ == "__main__":
  main()
