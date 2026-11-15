import os
import time

import numpy as np

from openpilot.starpilot.system.lightd.capture import LightCapture
from openpilot.starpilot.system.lightd.hba import HbaLight, HbaOut, Nv12Frame
from openpilot.starpilot.system.lightd.short_exposure import ShortFrame


def _scene():
  y = np.full((760, 1344), 10, np.uint8)
  y[378:383, 698:703] = 255
  uv = np.full((380, 1344), 128, np.uint8)
  raw = np.full((1520, 2688), 64, np.uint16)
  raw[756:766, 1396:1406] = 1000  # the same light at the short exposure's 2x scale
  return Nv12Frame(y, uv), ShortFrame.from_raw(raw, 7)


def _wait(cap, n, timeout=5.0):
  t0 = time.monotonic()
  while cap.saved < n and time.monotonic() - t0 < timeout:
    time.sleep(0.01)
  return cap.saved >= n


def test_capture_saves_aligned_long_and_short_crops(tmp_path):
  frame, short = _scene()
  out = HbaOut(lights=[HbaLight(3, 700.0, 380.0, 25, False, 936.0, 0.8, "candidate/colour", True, 0.9),
                       HbaLight(4, 10.0, 5.0, 2, True, 3.0, 0.1, "preceding/colour", False)])
  cap = LightCapture(str(tmp_path), route_fn=lambda: b"00000099--abcdef0123")
  cap.maybe_save(1.0, 7, frame, short, out, dark=True)
  assert _wait(cap, 1)
  d = np.load(os.path.join(tmp_path, "00000099--abcdef0123", "7.npz"))
  assert list(d["tid"]) == [3, 4] and d["kind"][0] == "candidate/colour" and d["look"][0] == np.float32(0.9)
  x0, y0 = d["o0"]
  yc, sc = d["y0"], d["s0"]
  assert sc.shape == (2 * yc.shape[0], 2 * yc.shape[1])
  # the light is in the same place in both crops
  ly = np.argwhere(yc == 255).mean(axis=0) + (y0, x0)
  ls = np.argwhere(sc == 1000).mean(axis=0) / 2 + (y0, x0)
  assert np.allclose(ly, (380, 700), atol=1) and np.allclose(ls, ly, atol=1)
  assert "full_y" in d and d["full_y"].shape == (760, 1344)
  # the corner light's crops are clipped to the frame, not padded
  assert tuple(d["o1"]) == (0, 0) and d["y1"].shape == (37, 42)


def test_capture_rate_dark_and_no_short(tmp_path):
  frame, _ = _scene()
  out = HbaOut(lights=[HbaLight(3, 700.0, 380.0, 25, False, float("nan"), 0.8, "candidate/colour", False)])
  cap = LightCapture(str(tmp_path), route_fn=lambda: "")
  cap.maybe_save(1.0, 1, frame, None, out, dark=False)   # daylight: nothing
  cap.maybe_save(1.0, 2, frame, None, HbaOut(), dark=True)  # no lights: nothing
  cap.maybe_save(1.0, 3, frame, None, out, dark=True)
  cap.maybe_save(1.2, 4, frame, None, out, dark=True)    # too soon
  cap.maybe_save(1.6, 5, frame, None, out, dark=True)
  assert _wait(cap, 2)
  time.sleep(0.1)
  files = sorted(f for _, _, fs in os.walk(tmp_path) for f in fs)
  assert files == ["3.npz", "5.npz"]
  d = np.load(next(os.path.join(dp, "3.npz") for dp, _, fs in os.walk(tmp_path) if "3.npz" in fs))
  assert not d["short_valid"] and "s0" not in d


def test_capture_prunes_oldest_past_the_cap(tmp_path):
  old = tmp_path / "r1"
  old.mkdir()
  for i in range(5):
    p = old / f"{i}.npz"
    p.write_bytes(b"x" * 1000)
    os.utime(p, (i, i))
  cap = LightCapture(str(tmp_path), max_bytes=2500)
  cap.prune()
  assert sorted(os.listdir(old)) == ["3.npz", "4.npz"]
