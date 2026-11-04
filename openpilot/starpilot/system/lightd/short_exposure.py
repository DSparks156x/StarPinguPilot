"""Short exposures from camerad's staggered HDR mode (param CameraShortExposure, see CAMERA_HDR.md).

camerad sends each road camera's short exposure, raw, on VisionIPC streams VISION_STREAM_NARROW_SHORT / _WIDE_SHORT:
2688x1520 MIPI-packed RAW10 (4 pixels in 5 bytes), BGGR, with the same frame id and timestamps as the processed frame.
The exposure and gain are fixed, so a pixel's value means the same scene brightness on every frame. StarPinguPilot.
"""
import numpy as np

SHORT_EXPOSURE_LINES = 0x40  # 0x3510-0x3512 in camerad's OS04C10 init table
ROW_TIME_S = 50e-3 / 2523    # staggered HDR VTS: about 19.8 us per row
SHORT_EXPOSURE_S = SHORT_EXPOSURE_LINES * ROW_TIME_S
BLACK_LEVEL = 16             # RAW10 counts, measured: a dark room's short frames read 15-16 at every percentile
RAW_MAX = 1023


def unpack_raw10(data, width: int, height: int, stride: int) -> np.ndarray:
  """MIPI RAW10 bytes (rows of `stride` bytes) -> (height, width) uint16. Each 5 bytes hold 4 pixels: their 8 high
  bits, then one byte of their 2 low bits, first pixel in the lowest bits."""
  assert width % 4 == 0 and stride >= width * 5 // 4
  rows = np.frombuffer(data, dtype=np.uint8, count=height * stride).reshape(height, stride)[:, :width * 5 // 4]
  g = rows.reshape(height, width // 4, 5).astype(np.uint16)
  out = np.empty((height, width // 4, 4), dtype=np.uint16)
  for k in range(4):
    out[:, :, k] = (g[:, :, k] << 2) | ((g[:, :, 4] >> (2 * k)) & 3)
  return out.reshape(height, width)


def bin_bayer(raw: np.ndarray, black: int = BLACK_LEVEL) -> np.ndarray:
  """Mean of each 2x2 Bayer quad minus black: a half-resolution luminance proxy, float32, in RAW10 counts."""
  h, w = raw.shape
  q = raw[:h // 2 * 2, :w // 2 * 2].astype(np.float32).reshape(h // 2, 2, w // 2, 2)
  return np.maximum(q.mean(axis=(1, 3)) - black, 0.0)


class ShortExposureReader:
  """Latest short exposure of one road camera. latest() -> (frame_id, timestamp_sof, raw uint16 HxW) or None."""

  def __init__(self, wide: bool = False):
    self.wide = wide
    self.client = None

  def latest(self):
    from msgq.visionipc import VisionIpcClient
    from openpilot.cereal.visionipc import VisionStreamType
    if self.client is None:
      stream = VisionStreamType.VISION_STREAM_WIDE_SHORT if self.wide else VisionStreamType.VISION_STREAM_NARROW_SHORT
      self.client = VisionIpcClient("camerad", stream, True)
    if not self.client.is_connected() and not self.client.connect(False):
      return None
    buf = None
    while (b := self.client.recv(timeout_ms=0)) is not None:
      buf = b
    if buf is None:
      return None
    raw = unpack_raw10(buf.data, buf.width, buf.height, buf.stride)
    return self.client.frame_id, self.client.timestamp_sof, raw
