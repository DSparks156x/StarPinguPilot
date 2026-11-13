"""Short exposures from camerad's staggered HDR mode (param CameraShortExposure, see CAMERA_HDR.md).

camerad sends each road camera's short exposure, raw, on VisionIPC streams VISION_STREAM_NARROW_SHORT / _WIDE_SHORT:
2688x1520 MIPI-packed RAW10 (4 pixels in 5 bytes), BGGR, with the same frame id and timestamps as the processed frame.
The processed frame is the same sensor area binned 2x2, so processed pixel (u, v) is raw pixels (2u..2u+1, 2v..2v+1).
The exposure and gain are fixed, so a pixel's value means the same scene radiance on every frame. StarPinguPilot.
"""
import math
import time

import numpy as np

SHORT_EXPOSURE_LINES = 0x40  # 0x3510-0x3512 in camerad's OS04C10 init table, at gain 1x (0x350c/d = 0x0080)
ROW_TIME_S = 50e-3 / 2523    # staggered HDR VTS: about 19.8 us per row
SHORT_EXPOSURE_S = SHORT_EXPOSURE_LINES * ROW_TIME_S
BLACK_LEVEL = 64             # RAW10 counts, measured: a dim room's short frames read 61-65 (camerad sets 0x366a = 0x54)
RAW_MAX = 1023
RAW_SCALE = 2                # raw pixels per processed pixel, each way (os04c10 out_scale)
MATCH_WAIT_MS = 5            # camerad sends the short frame right after the processed one
RECONNECT_S = 5.0            # camerad without staggered HDR serves the stream with no buffers; look again this often


def unpack_raw10(data, width: int, height: int, stride: int) -> np.ndarray:
  """MIPI RAW10 bytes (rows of `stride` bytes) -> (height, width) uint16. Each 5 bytes hold 4 pixels: their 8 high
  bits, then one byte of their 2 low bits, first pixel in the lowest bits."""
  assert width % 4 == 0 and stride >= width * 5 // 4
  return unpack_raw10_region(data, stride, 0, 0, width, height)


def unpack_raw10_region(data, stride: int, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
  """Raw pixels x0..x1, y0..y1 (x0 and x1 multiples of 4) of a MIPI RAW10 image, (y1 - y0, x1 - x0) uint16."""
  assert x0 % 4 == 0 and x1 % 4 == 0 and x1 > x0 and y1 > y0
  rows = np.frombuffer(data, dtype=np.uint8, count=y1 * stride).reshape(y1, stride)[y0:, x0 * 5 // 4:x1 * 5 // 4]
  g = rows.reshape(y1 - y0, (x1 - x0) // 4, 5).astype(np.uint16)
  out = np.empty((y1 - y0, (x1 - x0) // 4, 4), dtype=np.uint16)
  for k in range(4):
    out[:, :, k] = (g[:, :, k] << 2) | ((g[:, :, 4] >> (2 * k)) & 3)
  return out.reshape(y1 - y0, x1 - x0)


def bin_bayer(raw: np.ndarray, black: int = BLACK_LEVEL) -> np.ndarray:
  """Mean of each 2x2 Bayer quad minus black: a half-resolution luminance proxy, float32, in RAW10 counts."""
  h, w = raw.shape
  q = raw[:h // 2 * 2, :w // 2 * 2].astype(np.float32).reshape(h // 2, 2, w // 2, 2)
  return np.maximum(q.mean(axis=(1, 3)) - black, 0.0)


class ShortFrame:
  """One short exposure, left packed: only the boxes around lights are unpacked."""

  def __init__(self, data, width: int, height: int, stride: int, frame_id: int = 0, black: int = BLACK_LEVEL):
    assert width % 4 == 0 and stride >= width * 5 // 4 and len(data) >= height * stride
    self.data, self.width, self.height, self.stride = data, width, height, stride
    self.frame_id, self.black = frame_id, black

  @classmethod
  def from_raw(cls, raw: np.ndarray, frame_id: int = 0, black: int = BLACK_LEVEL):
    """Packs a (h, w) array of RAW10 values: for tests and offline tools."""
    h, w = raw.shape
    r = np.clip(raw, 0, RAW_MAX).astype(np.uint16).reshape(h, w // 4, 4)
    packed = np.empty((h, w // 4, 5), np.uint8)
    packed[:, :, :4] = (r >> 2).astype(np.uint8)
    packed[:, :, 4] = sum(((r[:, :, k] & 3) << (2 * k)) for k in range(4)).astype(np.uint8)
    return cls(packed.reshape(h, w * 5 // 4).tobytes(), w, h, w * 5 // 4, frame_id, black)

  def region(self, bbox, pad: int = 1) -> np.ndarray | None:
    """Raw pixels under a processed-frame box (x, y, w, h) grown by pad processed px, clipped to the frame; None if
    nothing is left. Columns are widened to whole 5-byte groups."""
    x, y, w, h = bbox
    s = RAW_SCALE
    x0, y0 = max((x - pad) * s, 0) & ~3, max((y - pad) * s, 0)
    x1, y1 = min(((x + w + pad) * s + 3) & ~3, self.width), min((y + h + pad) * s, self.height)
    if x1 <= x0 or y1 <= y0:
      return None
    return unpack_raw10_region(self.data, self.stride, x0, y0, x1, y1)

  def peak(self, bbox, pad: int = 1) -> float:
    """The brightest raw count above black under the box, on any colour channel: a light's radiance in fixed units
    (a lamp's core is its brightest point; a sign's face is uniform). nan if the box is outside the frame."""
    r = self.region(bbox, pad)
    if r is None:
      return math.nan
    return float(max(int(r.max()) - self.black, 0))


class ShortExposureReader:
  """The short exposure of one road camera that goes with a processed frame."""

  def __init__(self, wide: bool = False):
    self.wide = wide
    self.client = None
    self._checked = -math.inf

  def _connected(self) -> bool:
    from msgq.visionipc import VisionIpcClient
    from openpilot.cereal.visionipc import VisionStreamType
    if self.client is None:
      stream = VisionStreamType.VISION_STREAM_WIDE_SHORT if self.wide else VisionStreamType.VISION_STREAM_NARROW_SHORT
      self.client = VisionIpcClient("camerad", stream, True)
    if self.client.is_connected() and self.client.num_buffers > 0:
      return True
    now = time.monotonic()
    if now - self._checked < RECONNECT_S:
      return False
    self._checked = now
    return self.client.connect(False) and self.client.num_buffers > 0

  def _newest(self, timeout_ms=0):
    buf = None
    while (b := self.client.recv(timeout_ms=timeout_ms)) is not None:
      buf, timeout_ms = b, 0
    return buf

  def matching(self, frame_id: int) -> ShortFrame | None:
    """The short exposure of processed frame frame_id, or None (no staggered HDR, or it isn't the same frame)."""
    if not self._connected():
      return None
    buf = self._newest()
    fid = self.client.frame_id if buf is not None else -1
    if buf is None or fid < frame_id:
      buf = self._newest(MATCH_WAIT_MS)
      fid = self.client.frame_id if buf is not None else -1
    if buf is None or fid != frame_id:
      return None
    return ShortFrame(buf.data, buf.width, buf.height, buf.stride, fid)

  def latest(self):
    """(frame_id, timestamp_sof, raw uint16 HxW) of the newest short exposure, or None. For tools."""
    if not self._connected():
      return None
    buf = self._newest()
    if buf is None:
      return None
    raw = unpack_raw10(buf.data, buf.width, buf.height, buf.stride)
    return self.client.frame_id, self.client.timestamp_sof, raw
