import math

import numpy as np

from openpilot.starpilot.system.lightd.hba import FCAM_H, FCAM_W, Geometry, HbaClassifier, HbaParams, KIND_STATIC
from openpilot.starpilot.system.lightd.short_exposure import (BLACK_LEVEL, RAW_SCALE, ShortFrame, bin_bayer, unpack_raw10,
                                                              unpack_raw10_region)
from openpilot.starpilot.system.tests.test_lightd_hba import (DT, HEIGHT, _first_dip, _lit_sign_frame, add_light, dark_frame,
                                                              frame_with_light, run)


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


# ---- ShortFrame: the raw box under a processed-frame box ----


def test_short_frame_packs_and_unpacks_regions():
  rng = np.random.default_rng(1)
  px = rng.integers(0, 1024, size=(40, 64), dtype=np.uint16)
  sf = ShortFrame.from_raw(px)
  assert np.array_equal(unpack_raw10(sf.data, 64, 40, sf.stride), px)
  assert np.array_equal(unpack_raw10_region(sf.data, sf.stride, 8, 6, 24, 20), px[6:20, 8:24])
  # processed box (x, y, w, h) = (5, 4, 2, 3) grown by 1 px: raw columns 8..16 (whole groups of 4), rows 6..16
  assert np.array_equal(sf.region((5, 4, 2, 3)), px[6:16, 8:16])
  # clipped at the frame's edges, None when nothing is left
  assert np.array_equal(sf.region((0, 0, 2, 2)), px[0:6, 0:8])
  assert sf.region((40, 30, 4, 4)) is None


def test_short_peak_is_the_brightest_raw_count_above_black_on_any_channel():
  px = np.full((40, 64), BLACK_LEVEL, np.uint16)
  px[2 * 10 + 1, 2 * 12] = BLACK_LEVEL + 300   # one red/blue site of processed pixel (12, 10)
  px[0, 0] = 1023                              # far away: not under the box
  sf = ShortFrame.from_raw(px)
  assert sf.peak((12, 10, 1, 1)) == 300.0
  assert sf.peak((20, 2, 2, 2)) == 0.0
  assert math.isnan(sf.peak((100, 100, 2, 2)))
  assert RAW_SCALE == 2


# ---- what the HBA does with it ----


GEOM = Geometry([0.0, 0.0, 0.0], HEIGHT)


def short_with(lights, black=BLACK_LEVEL):
  """A short exposure of the night: black everywhere except a lamp-sized spot reading `counts` above black at each
  (x, y, z_above_road, counts), the same place add_light draws it in the processed frame."""
  raw = np.full((FCAM_H * RAW_SCALE, FCAM_W * RAW_SCALE), black, np.uint16)
  for x, y, z, counts in lights:
    uv, ok = GEOM.project(np.array([[x, y, HEIGHT - z]]))
    assert ok[0]
    u, v = (int(round(c)) * RAW_SCALE for c in uv[0])
    raw[v - 4:v + 5, u - 4:u + 5] = black + counts
  return ShortFrame.from_raw(raw)


def lit_bg():
  return np.full((FCAM_H, FCAM_W, 3), 45, np.uint8)


def test_a_blooming_light_that_is_dim_in_the_short_exposure_is_a_sign():
  # without the short exposure this light dips: it blooms like a lamp (test_a_lit_sign_is_static_and_a_blooming_lamp_is_not)
  lamp = add_light(lit_bg(), GEOM, 150.0, -3.6, 0.7, radius=5)
  clf = HbaClassifier()
  run(clf, [lit_bg()] * 120, high_beam_on=True)
  outs = run(clf, [lamp] * 40, t0=6.0, high_beam_on=True, short=short_with([(150.0, -3.6, 0.7, 30)]))
  assert all(o.high_beam_permitted for o in outs) and all(o.short_valid for o in outs)
  assert {tr.why for tr in clf.tracks if tr.missed == 0} == {"short"}
  # the same light bright in the short exposure is a lamp and dips
  clf = HbaClassifier()
  run(clf, [lit_bg()] * 120, high_beam_on=True)
  outs = run(clf, [lamp] * 40, t0=6.0, high_beam_on=True, short=short_with([(150.0, -3.6, 0.7, 600)]))
  assert not outs[-1].high_beam_permitted
  assert all(x.lamp for x in outs[-1].lights)


def test_a_lamp_that_looks_compact_or_was_born_at_our_switch_on_is_still_a_lamp():
  # the compact-glow rule calls this a sign; the short exposure says it's a source
  clf = HbaClassifier()
  run(clf, [lit_bg()] * 120, high_beam_on=True)
  outs = run(clf, [_lit_sign_frame(GEOM, 150.0, 0.0)] * 40, t0=6.0, high_beam_on=True,
             short=short_with([(150.0, 0.0, 0.7, 500)]))
  assert not outs[-1].high_beam_permitted
  assert "sign" not in {tr.why for tr in clf.tracks if tr.missed == 0}
  # a far car that shows up just as our high beam comes on: the switch-on rule would call it the sign we lit
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 50, high_beam_on=False)
  car = frame_with_light(GEOM, 150.0, -3.6, 0.7, radius=6)
  outs = run(clf, [car] * 20, t0=2.5, high_beam_on=True, short=short_with([(150.0, -3.6, 0.7, 400)]))
  i = _first_dip(outs)
  # short_min_obs frames to measure it, then ev_lamp
  p = HbaParams()
  assert i is not None and (i + 1) * DT <= p.short_min_obs * DT + 1.0 / p.ev_lamp + 2 * DT + 1e-9


def test_a_small_dim_dot_is_left_to_the_rules():
  # a far car's lamp can be as dim as a lit delineator: too small to call a sign
  dot = frame_with_light(GEOM, 160.0, -3.6, 0.7, radius=1)
  need = 1.0 / HbaParams().ev_lone
  for short in (None, short_with([(160.0, -3.6, 0.7, 5)])):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * 120)
    outs = run(clf, [dot] * round(1.2 * need / DT), t0=6.0, short=short)
    assert _first_dip(outs) is not None
    assert all(tr.kind != KIND_STATIC for tr in clf.tracks if tr.missed == 0)


def test_lights_are_listed_for_the_log():
  clf = HbaClassifier()
  outs = run(clf, [frame_with_light(GEOM, 60.0, -3.6, 0.7, radius=4)] * 3, short=short_with([(60.0, -3.6, 0.7, 250)]))
  (light,) = outs[-1].lights
  assert light.short_peak == 250.0 and not light.red and light.area > 0 and "/" in light.kind
  outs = run(clf, [frame_with_light(GEOM, 60.0, -3.6, 0.7, radius=4)], t0=1.0)
  assert not outs[-1].short_valid and math.isnan(outs[-1].lights[0].short_peak)
