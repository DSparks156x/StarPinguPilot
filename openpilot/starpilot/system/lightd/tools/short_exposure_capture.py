#!/usr/bin/env python3
"""short_exposure_capture.py [--period S] [--out DIR] [--max-mb MB]: on the device, runs until killed and saves matched
pairs of the narrow camera's processed frame and its staggered-HDR short exposure while it is dark, for looking at what
lamps, signs and reflectors look like in each (lightd only logs one number per light, the short peak).

Waits for camerad (start it with nohup before a drive; it survives offroad/onroad switches). Every --period seconds, if
the processed frame is dark (mean Y < --dark-y), saves <out>/<frame id>_y.png (processed Y, 1344x760) and
<out>/<frame id>_s.png (the short exposure 2x2-binned onto the same pixel grid, counts above black, 16-bit) plus a line
in <out>/index.csv with the frame id, monotonic time and wall time; lightd's light lists in the rlog carry the same frame
id. Stops saving at --max-mb. StarPinguPilot.
"""
import argparse
import os
import time

import numpy as np

from openpilot.starpilot.system.lightd.short_exposure import ShortExposureReader, bin_bayer, unpack_raw10


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--period", type=float, default=4.0)
  ap.add_argument("--out", default="/data/media/0/shortcap")
  ap.add_argument("--max-mb", type=float, default=1500.0)
  ap.add_argument("--dark-y", type=float, default=45.0)
  args = ap.parse_args()

  import cv2
  from msgq.visionipc import VisionIpcClient
  from openpilot.cereal.visionipc import VisionStreamType
  os.makedirs(args.out, exist_ok=True)
  index = open(os.path.join(args.out, "index.csv"), "a")
  used = sum(os.path.getsize(os.path.join(args.out, f)) for f in os.listdir(args.out)) / 1e6
  proc = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
  short = ShortExposureReader()
  last = 0.0
  while used < args.max_mb:
    if not proc.is_connected() and not proc.connect(False):
      time.sleep(2.0)
      continue
    b = proc.recv(timeout_ms=1000)
    if b is None or time.monotonic() - last < args.period:
      continue
    fid = proc.frame_id
    y = np.frombuffer(b.data, np.uint8, count=b.stride * b.height).reshape(b.height, b.stride)[:, :b.width]
    if y.mean() >= args.dark_y:
      last = time.monotonic()
      continue
    s = short.matching(fid)
    if s is None:
      continue
    last = time.monotonic()
    q = bin_bayer(unpack_raw10(s.data, s.width, s.height, s.stride), s.black)
    for name, img in (("y", y), ("s", np.clip(q, 0, 65535).astype(np.uint16))):
      p = os.path.join(args.out, f"{fid}_{name}.png")
      cv2.imwrite(p, img)
      used += os.path.getsize(p) / 1e6
    index.write(f"{fid},{time.monotonic():.3f},{time.time():.3f}\n")  # noqa: TID251 wall time, to find the drive
    index.flush()
  print(f"stopped at {used:.0f} MB")


if __name__ == "__main__":
  main()
