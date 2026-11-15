#!/usr/bin/env python3
"""hbacap_view.py <capture dir> <out dir> [--tid N] [--min-ev E]: contact sheets of an HbaCapture route folder (see
capture.py): per light, the processed crop (gamma'd to show glow) beside the short-exposure crop (log-scaled raw counts
above black), labelled with track id, kind, evidence, look and short peak. Writes <out>/sheet_<n>.jpg of 24 lights each
and <out>/lights.csv (one row per light per frame, for fitting). StarPinguPilot."""
import argparse
import csv
import glob
import os

import cv2
import numpy as np

TILE = 128


def tile_long(y):
  g = (np.power(y.astype(np.float32) / 255.0, 0.4) * 255).astype(np.uint8)
  return cv2.resize(g, (TILE, TILE * g.shape[0] // max(g.shape[1], 1)), interpolation=cv2.INTER_NEAREST)


def tile_short(s, black):
  q = np.maximum(s.astype(np.float32) - black, 0)
  g = (np.log1p(q) / np.log1p(959) * 255).clip(0, 255).astype(np.uint8)
  return cv2.resize(g, (TILE, TILE * g.shape[0] // max(g.shape[1], 1)), interpolation=cv2.INTER_NEAREST)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("cap")
  ap.add_argument("out")
  ap.add_argument("--tid", type=int)
  ap.add_argument("--min-ev", type=float, default=0.0)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=True)
  rows, tiles = [], []
  for p in sorted(glob.glob(os.path.join(a.cap, "*.npz")), key=lambda p: int(os.path.basename(p)[:-4])):
    d = np.load(p)
    for i in range(len(d["tid"])):
      r = {k: d[k][i].item() for k in ("tid", "u", "v", "area", "red", "short_peak", "evidence", "lamp", "look", "kind")}
      r.update(frame_id=int(d["frame_id"]), t=float(d["t"]), permitted=bool(d["permitted"]))
      rows.append(r)
      if (a.tid is not None and r["tid"] != a.tid) or r["evidence"] < a.min_ev:
        continue
      lt = tile_long(d[f"y{i}"])
      st = tile_short(d[f"s{i}"], int(d["black"])) if f"s{i}" in d else np.zeros_like(lt)
      h = max(lt.shape[0], st.shape[0])
      pair = np.zeros((h + 30, 2 * TILE + 4), np.uint8)
      pair[30:30 + lt.shape[0], :TILE] = lt
      pair[30:30 + st.shape[0], TILE + 4:] = st
      cv2.putText(pair, f"{r['frame_id']} #{r['tid']} {r['kind'][:14]}", (2, 11), cv2.FONT_HERSHEY_SIMPLEX, 0.35, 255, 1)
      cv2.putText(pair, f"ev{r['evidence']:.2f} lk{r['look']:.2f} sp{r['short_peak']:.0f} a{r['area']}", (2, 25),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.35, 255, 1)
      tiles.append(cv2.copyMakeBorder(pair[:TILE + 30], 0, TILE + 30 - min(pair.shape[0], TILE + 30), 2, 2, cv2.BORDER_CONSTANT))
  for n in range(0, len(tiles), 24):
    chunk = tiles[n:n + 24]
    chunk += [np.zeros_like(chunk[0])] * (24 - len(chunk))
    sheet = np.vstack([np.hstack(chunk[r * 4:(r + 1) * 4]) for r in range(6)])
    cv2.imwrite(os.path.join(a.out, f"sheet_{n // 24:03d}.jpg"), sheet)
  if rows:
    with open(os.path.join(a.out, "lights.csv"), "w", newline="") as f:
      w = csv.DictWriter(f, fieldnames=list(rows[0]))
      w.writeheader()
      w.writerows(rows)
  print(f"{len(rows)} lights, {len(tiles)} shown, {(len(tiles) + 23) // 24} sheets")


if __name__ == "__main__":
  main()
