"""HBA light capture (param HbaCapture): what the HBA saw of each light it tracked, long and short exposure side by side,
for tuning the look and short-exposure rules on real drives. StarPinguPilot.

lightd's HBA thread calls maybe_save after every classified frame. In the dark, at most every PERIOD_S, on frames with
tracked lights, it copies a (2 HALF)^2 crop of the processed Y and UV around each of the MAX_LIGHTS lights with the most
evidence, and the raw RAW10 short exposure under the same box (unpacked, black level not subtracted), plus the light's
track id, kind, evidence, look, short peak and lamp flag; every FULL_PERIOD_S also the whole processed Y frame for
context. A writer thread saves one compressed .npz per frame to <root>/<route>/<frame id>.npz (frame ids are the
starpilotLightState hbaFrameId, so every file lines up with the rlog), and deletes the oldest files under root past
max_bytes. The HBA thread only copies small arrays; if the writer falls behind, frames are dropped, never waited for.
Read them with tools/hbacap_view.py.
"""
import math
import os
import queue
import threading
import time

import numpy as np

from openpilot.common.swaglog import cloudlog

CAPTURE_DIR = "/data/media/0/hbacap"
PERIOD_S = 0.5
FULL_PERIOD_S = 10.0
MAX_LIGHTS = 12
HALF = 32            # processed px either side of a light
MAX_BYTES = 3e9
PRUNE_EVERY = 50     # saves between disk-use checks


def crops(frame, short, u: float, v: float, half: int = HALF):
  """(x0, y0, y crop, uv crop, raw short crop or None) around (u, v), clipped to the frame. uv rows/cols are the NV12
  interleaved U,V of the same box (x0 rounded down to even)."""
  h, w = frame.y.shape
  x0, y0 = max(int(round(u)) - half, 0) & ~1, max(int(round(v)) - half, 0) & ~1
  x1, y1 = min(int(round(u)) + half, w), min(int(round(v)) + half, h)
  yc = np.array(frame.y[y0:y1, x0:x1])
  uvc = np.array(frame.uv[y0 // 2:(y1 + 1) // 2, x0:x1 + (x1 - x0) % 2])
  sc = short.region((x0, y0, x1 - x0, y1 - y0), pad=0) if short is not None else None
  return x0, y0, yc, uvc, (np.array(sc, np.uint16) if sc is not None else None)


class LightCapture:
  def __init__(self, root: str = CAPTURE_DIR, route_fn=None, max_bytes: float = MAX_BYTES):
    self.root = root
    self.route_fn = route_fn or (lambda: "")
    self.max_bytes = max_bytes
    self.dir: str | None = None
    self.last = -math.inf
    self.last_full = -math.inf
    self.q: queue.Queue = queue.Queue(maxsize=8)
    self.thread: threading.Thread | None = None
    self.saved = 0
    self.dropped = 0

  def _route_dir(self):
    if self.dir is None or os.path.basename(self.dir).startswith("boot-"):
      route = self.route_fn() or ""
      route = route.decode() if isinstance(route, bytes) else str(route)
      name = route if route else time.strftime("boot-%Y%m%d-%H%M%S")  # a name to find the files by
      if self.dir is None or route:
        self.dir = os.path.join(self.root, name)
    return self.dir

  def maybe_save(self, t: float, frame_id: int, frame, short, out, dark: bool):
    if not dark or out is None or not out.lights or t - self.last < PERIOD_S:
      return
    self.last = t
    lights = sorted(out.lights, key=lambda x: -x.evidence)[:MAX_LIGHTS]
    rec = {"frame_id": np.int64(frame_id), "t": np.float64(t), "permitted": np.bool_(out.high_beam_permitted),
           "reason": np.int32(out.reason), "short_valid": np.bool_(short is not None),
           "black": np.int32(short.black if short is not None else -1),
           "tid": np.array([x.tid for x in lights], np.int64), "u": np.array([x.u for x in lights], np.float32),
           "v": np.array([x.v for x in lights], np.float32), "area": np.array([x.area for x in lights], np.int32),
           "red": np.array([x.red for x in lights], bool), "short_peak": np.array([x.short_peak for x in lights], np.float32),
           "evidence": np.array([x.evidence for x in lights], np.float32), "lamp": np.array([x.lamp for x in lights], bool),
           "look": np.array([x.look for x in lights], np.float32), "kind": np.array([x.kind for x in lights], dtype="U32")}
    for i, x in enumerate(lights):
      x0, y0, yc, uvc, sc = crops(frame, short, x.u, x.v)
      rec[f"o{i}"] = np.array([x0, y0], np.int32)
      rec[f"y{i}"] = yc
      rec[f"uv{i}"] = uvc
      if sc is not None:
        rec[f"s{i}"] = sc
    if t - self.last_full >= FULL_PERIOD_S:
      self.last_full = t
      rec["full_y"] = np.array(frame.y)
    try:
      self.q.put_nowait((self._route_dir(), int(frame_id), rec))
    except queue.Full:
      self.dropped += 1
      return
    if self.thread is None:
      self.thread = threading.Thread(target=self._run, name="lightd_capture", daemon=True)
      self.thread.start()

  def _run(self):
    while True:
      d, fid, rec = self.q.get()
      try:
        os.makedirs(d, exist_ok=True)
        np.savez_compressed(os.path.join(d, f"{fid}.npz"), **rec)
        self.saved += 1
        if self.saved % PRUNE_EVERY == 1:
          self.prune()
      except Exception:
        cloudlog.exception("lightd: capture write failed")

  def prune(self):
    files = []
    for dp, _, names in os.walk(self.root):
      for n in names:
        p = os.path.join(dp, n)
        try:
          st = os.stat(p)
        except OSError:
          continue
        files.append((st.st_mtime, st.st_size, p))
    total = sum(f[1] for f in files)
    for _, size, p in sorted(files):
      if total <= self.max_bytes:
        break
      try:
        os.remove(p)
        total -= size
      except OSError:
        pass
