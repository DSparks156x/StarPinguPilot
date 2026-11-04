import numpy as np

from openpilot.starpilot.system.lightd.short_exposure import BLACK_LEVEL, bin_bayer, unpack_raw10


def pack_raw10(px: np.ndarray, stride: int) -> bytes:
  """The MIPI CSI-2 RAW10 packing the sensor sends: per 4 pixels their 8 high bits, then a byte of the 2 low bits each."""
  h, w = px.shape
  g = px.reshape(h, w // 4, 4).astype(np.uint16)
  packed = np.zeros((h, w // 4, 5), dtype=np.uint8)
  packed[:, :, :4] = g >> 2
  packed[:, :, 4] = (g[:, :, 0] & 3) | (g[:, :, 1] & 3) << 2 | (g[:, :, 2] & 3) << 4 | (g[:, :, 3] & 3) << 6
  rows = np.zeros((h, stride), dtype=np.uint8)
  rows[:, :w * 5 // 4] = packed.reshape(h, -1)
  return rows.tobytes()


def test_raw10_round_trips_every_value_and_ignores_row_padding():
  rng = np.random.default_rng(0)
  px = rng.integers(0, 1024, size=(6, 16), dtype=np.uint16)
  px[0, :4] = [0, 1023, 1, 1022]  # extremes and both low-bit patterns at the same spot of a group
  data = pack_raw10(px, stride=24)  # 20 bytes of pixels + 4 of padding per row
  assert np.array_equal(unpack_raw10(data, 16, 6, 24), px)


def test_raw10_at_camerad_size():
  w, h, stride = 2688, 1520, 2688 * 10 // 8
  px = (np.arange(w * h, dtype=np.uint32) % 1024).astype(np.uint16).reshape(h, w)
  assert np.array_equal(unpack_raw10(pack_raw10(px, stride), w, h, stride), px)


def test_bayer_bin_means_each_quad_and_removes_black():
  raw = np.full((4, 4), BLACK_LEVEL, dtype=np.uint16)
  raw[0:2, 0:2] = [[BLACK_LEVEL + 40, BLACK_LEVEL + 80], [BLACK_LEVEL + 80, BLACK_LEVEL + 120]]
  raw[2:4, 2:4] = BLACK_LEVEL - 5  # read noise below black clamps at 0
  out = bin_bayer(raw)
  assert out.shape == (2, 2)
  assert out[0, 0] == 80.0 and out[0, 1] == 0.0 and out[1, 1] == 0.0
