#!/usr/bin/env python3
"""short_exposure_dump.py [--wide] [--seconds S] [--save N] [--out DIR]: on the device, with camerad running in staggered HDR
mode (param CameraShortExposure), checks the short exposure stream and saves a few frames.

Prints once a second: frames received and their rate, the frame id lag behind the processed stream, raw percentiles,
the fraction of clipped pixels, and the short frame's mean next to the processed frame's Y mean (the short one should be
30-40x darker). Saves N binned frames as PNG, gamma-encoded for viewing, with the processed frame beside them. Also the
first raw buffer as .npy, for checking the black level and VC assignment offline. StarPinguPilot.
"""
import argparse
import os
import time

import numpy as np

from openpilot.starpilot.system.lightd.short_exposure import RAW_MAX, ShortExposureReader, bin_bayer


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--wide", action="store_true")
  ap.add_argument("--seconds", type=float, default=20.0)
  ap.add_argument("--save", type=int, default=5)
  ap.add_argument("--out", default="/data/tmp/short_exposure")
  args = ap.parse_args()

  import cv2
  from msgq.visionipc import VisionIpcClient
  from openpilot.cereal.visionipc import VisionStreamType
  os.makedirs(args.out, exist_ok=True)
  short = ShortExposureReader(wide=args.wide)
  main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if args.wide else VisionStreamType.VISION_STREAM_NARROW_ROAD
  proc = VisionIpcClient("camerad", main_stream, True)
  print("streams:", sorted(VisionIpcClient.available_streams("camerad", block=False)))

  t0 = last_print = time.monotonic()
  n = saved = 0
  last_id = None
  while time.monotonic() - t0 < args.seconds:
    got = short.latest()
    if got is None:
      time.sleep(0.01)
      continue
    fid, sof, raw = got
    if fid == last_id:
      continue
    last_id = fid
    n += 1
    if not proc.is_connected():
      proc.connect(False)
    pbuf = proc.recv(timeout_ms=0) if proc.is_connected() else None
    y = None
    if pbuf is not None:
      y = np.frombuffer(pbuf.data, dtype=np.uint8, count=pbuf.stride * pbuf.height).reshape(pbuf.height, pbuf.stride)[:, :pbuf.width]
    if n == 1:
      np.save(os.path.join(args.out, f"raw_{fid}.npy"), raw)
      print(f"raw buffer {raw.shape}, saved raw_{fid}.npy")

    now = time.monotonic()
    if now - last_print >= 1.0:
      p = np.percentile(raw[::4, ::4], [0.1, 1, 50, 99, 99.9])
      clip = np.mean(raw[::4, ::4] >= RAW_MAX)
      line = (f"{n / (now - t0):5.1f} Hz  frame {fid}  raw p0.1/1/50/99/99.9 {' '.join(f'{v:.0f}' for v in p)}  "
              f"clipped {clip * 100:.3f} %  short mean {raw.mean():.1f}")
      if y is not None:
        line += f"  processed Y mean {y.mean():.1f} (frame {proc.frame_id}, lag {proc.frame_id - fid})"
      print(line)
      last_print = now

    if saved < args.save and n % 10 == 0:
      b = bin_bayer(raw)
      view = (255.0 * np.sqrt(np.clip(b / (RAW_MAX - 12), 0.0, 1.0))).astype(np.uint8)
      if y is not None:
        view = np.hstack([view, cv2.resize(y, (view.shape[1], view.shape[0]))])
      cv2.imwrite(os.path.join(args.out, f"short_{fid}.png"), view)
      saved += 1
  print(f"{n} short frames in {args.seconds:.0f} s, saved {saved} to {args.out}")


if __name__ == "__main__":
  main()
