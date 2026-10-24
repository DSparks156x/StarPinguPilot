import math

import cv2
import numpy as np
import pytest

from openpilot.starpilot.system.lightd.hba import (ECAM_H, ECAM_W, FCAM_H, FCAM_W, Geometry, HbaClassifier, Nv12Frame,
                                                    REASON_DELAY, REASON_FAULT, REASON_ONCOMING, REASON_SPEED, REASON_TOWN,
                                                    corridor_mask, find_blobs, HbaParams, KIND_STATIC, road_centerline,
                                                    rot_from_euler, rotate_pixel, wide_lights, wide_params)
from openpilot.starpilot.system.lightd.hba import _geomspace, _slopes, candidate_ymin, colour_masks, median, town_lights, union_box

HEIGHT = 1.22
STRAIGHT = {
  "position": np.column_stack([np.linspace(0, 190, 33), np.zeros(33), np.zeros(33)]),
  # lane lines are on the road (z = camera height below the camera); the plan position is the device's own path
  "lanes": [np.column_stack([np.linspace(0, 190, 33), np.full(33, y), np.full(33, HEIGHT)]) for y in (-5.4, -1.8, 1.8, 5.4)],
  "lane_probs": [0.5, 0.9, 0.9, 0.5],
  "edges": [],
  "leads": [(0.0, 50.0, 0.0)],
}
FAST = 25.0
DT = 0.05


def dark_frame():
  return np.full((FCAM_H, FCAM_W, 3), 12, np.uint8)


def draw_lamp(img, u, v, radius=3, color=(255, 255, 255), glow=True):
  """A lamp blooms on the sensor: a dim halo around the core. A retroreflector lit by our beams doesn't (glow=False)."""
  if glow:
    g = 3 * radius + 2
    img[max(0, v - g):v + g + 1, max(0, u - g):u + g + 1] = np.maximum(
      img[max(0, v - g):v + g + 1, max(0, u - g):u + g + 1], (np.array(color) * 0.4).astype(np.uint8))
  img[v - radius:v + radius + 1, u - radius:u + radius + 1] = color
  return img


def frame_with_light(geom, x, y, z_above_road, radius=3, color=(255, 255, 255), glow=True):
  return add_light(dark_frame(), geom, x, y, z_above_road, radius, color, glow)


def run(clf, frames, v=FAST, t0=0.0, model=STRAIGHT, dt=DT, **kw):
  return [clf.update(t0 + i * dt, f, model, [0.0, 0.0, 0.0], HEIGHT, v, **kw) for i, f in enumerate(frames)]


def add_light(img, geom, x, y, z_above_road, radius=3, color=(255, 255, 255), glow=True):
  uv, ok = geom.project(np.array([[x, y, HEIGHT - z_above_road]]))
  assert ok[0]
  u, v = (int(round(c)) for c in uv[0])
  return draw_lamp(img, u, v, radius, color, glow)


def test_rot_from_euler_is_zyx():
  assert np.allclose(rot_from_euler([0, 0, 0]), np.eye(3))
  r = rot_from_euler([0.1, 0.2, 0.3])
  assert np.allclose(r @ r.T, np.eye(3))
  # yaw only rotates x towards y
  assert np.allclose(rot_from_euler([0, 0, np.pi / 2]) @ [1, 0, 0], [0, 1, 0])


def test_projection_and_ground_ray_roundtrip():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  uv, ok = geom.project(np.array([[100.0, 0.0, HEIGHT - 0.7]]))
  assert ok[0] and abs(uv[0, 0] - FCAM_W / 2) < 1 and uv[0, 1] > FCAM_H / 2  # straight ahead, below the horizon
  x, y = geom.ray_to_plane(uv[0, 0], uv[0, 1], HEIGHT - 0.7)
  assert x == pytest.approx(100.0, rel=1e-6) and y == pytest.approx(0.0, abs=1e-6)


def test_finds_white_and_red_but_not_amber():
  mask = np.full((FCAM_H, FCAM_W), 255, np.uint8)
  img = dark_frame()
  draw_lamp(img, 301, 301, 2)                     # headlight
  img[300:304, 500:504] = (20, 20, 255)          # taillight halo (BGR)...
  img[301:303, 501:503] = (220, 220, 255)        # ...with a bright core
  img[300:304, 700:704] = (0, 200, 255)          # amber reflector / yellow sign
  blobs = find_blobs(img, mask, HbaParams())
  assert sorted(b.red for b in blobs) == [False, True]
  assert all(abs(b.u - 701.5) > 10 for b in blobs)


def test_nv12_crop_matches_full_conversion():
  rng = np.random.default_rng(0)
  nv12 = rng.integers(0, 256, (FCAM_H * 3 // 2, FCAM_W), dtype=np.uint8)
  full = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
  frame = Nv12Frame(nv12[:FCAM_H], nv12[FCAM_H:])
  for x, y, w, h in ((0, 0, FCAM_W, FCAM_H), (187, 318, 781, 127), (1, 1, 1, 1), (FCAM_W - 3, FCAM_H - 5, 3, 5), (10, 11, 2, 3)):
    assert np.array_equal(frame.crop_bgr(x, y, w, h), full[y:y + h, x:x + w])


@pytest.mark.parametrize("rpy", [[0.0, 0.0, 0.0], [0.0, 0.05, 0.02], [0.02, -0.04, -0.03]])
def test_corridor_mask_matches_full_frame_dilation(rpy):
  p = HbaParams()
  geom = Geometry(rpy, HEIGHT)
  mask = corridor_mask(geom, road_centerline(STRAIGHT, p, HEIGHT), p)
  undilated = corridor_mask(geom, road_centerline(STRAIGHT, p, HEIGHT), HbaParams(margin_px=0))
  k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * p.margin_px + 1, 2 * p.margin_px + 1))
  dilated = cv2.dilate(undilated, k)
  # the outline trick matches a disc dilation to within the line rasterisation: a pixel or two along the edges
  assert undilated.any() and np.all(mask[undilated > 0] > 0)
  diff = np.count_nonzero(mask != dilated)
  assert diff < 0.03 * np.count_nonzero(dilated), diff


def test_dark_empty_road_restores_high_beam_after_clear_time():
  clf = HbaClassifier()
  outs = run(clf, [dark_frame()] * 80)
  first = next(i for i, o in enumerate(outs) if o.high_beam_permitted)
  assert (first + 1) * DT == pytest.approx(clf.p.clear_s + DT, abs=2 * DT)
  assert outs[5].reason & REASON_DELAY


def test_oncoming_headlight_dips_fast_and_restores_slow():
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  run(clf, [dark_frame()] * 120)  # permitted since 2 s; a lamp within short_permit_s of that would back off
  # big saturated headlight in the oncoming lane at 150 m, approaching
  frames = [frame_with_light(geom, 150 - i * 2.0, -3.6, 0.7, radius=4) for i in range(20)]
  outs = run(clf, frames, t0=6.0)
  dip = next(i for i, o in enumerate(outs) if not o.high_beam_permitted)
  assert (dip + 1) * DT <= 1.0 / clf.p.ev_strong_far + DT + 1e-9  # far (no range at 150 m), our beam off: two 10 Hz frames
  assert outs[-1].reason & REASON_ONCOMING
  after = run(clf, [dark_frame()] * 60, t0=7.0)
  assert not any(o.high_beam_permitted for o in after[:int(clf.p.clear_s / DT) - 1])
  assert after[-1].high_beam_permitted


def test_speed_gate_and_invalid_inputs():
  clf = HbaClassifier()
  outs = run(clf, [dark_frame()] * 80, v=10.0)
  assert not any(o.high_beam_permitted for o in outs) and outs[-1].reason & REASON_SPEED
  out = clf.update(10.0, dark_frame(), None, [0.0, 0.0, 0.0], HEIGHT, FAST)
  assert not out.high_beam_permitted and out.reason == REASON_FAULT


def test_model_lead_dips():
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 60)
  lead = dict(STRAIGHT, leads=[(0.9, 80.0, 0.0)])
  outs = run(clf, [dark_frame()] * 3, t0=3.0, model=lead)
  assert not outs[-1].high_beam_permitted


def test_lane_lines_and_plan_path_give_the_same_road():
  # lanes sit on the road; the plan path is the device's own trajectory, a camera height above it
  p = HbaParams()
  no_lanes = dict(STRAIGHT, lane_probs=[0.0, 0.0, 0.0, 0.0])
  a = road_centerline(STRAIGHT, p, HEIGHT)
  b = road_centerline(no_lanes, p, HEIGHT)
  assert np.allclose(a[:, 2], HEIGHT) and np.allclose(a, b)
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  assert np.array_equal(corridor_mask(geom, a, p), corridor_mask(geom, b, p))


def test_ranges_lamps_above_the_horizon_on_an_uphill_road():
  p = HbaParams()
  xs = np.linspace(0, 190, 33)
  uphill = dict(STRAIGHT, lanes=[np.column_stack([xs, np.full(33, y), HEIGHT - 0.04 * xs]) for y in (-5.4, -1.8, 1.8, 5.4)])
  center = road_centerline(uphill, p, HEIGHT)
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  # a lamp 0.7 m over the road 100 m out, where the road is 4 m higher than here: above the camera horizon
  uv, _ = geom.project(np.array([[100.0, 0.0, HEIGHT - 4.0 - 0.7]]))
  assert uv[0, 1] < geom.horizon_row()
  hit = geom.ray_to_road(uv[0, 0], uv[0, 1], center, 0.7)
  assert hit is not None and hit[0] == pytest.approx(100.0, rel=0.02)
  # and the usual downhill-looking case on a flat road
  flat = road_centerline(STRAIGHT, p, HEIGHT)
  uv, _ = geom.project(np.array([[60.0, 1.0, HEIGHT - 0.7]]))
  x, y = geom.ray_to_road(uv[0, 0], uv[0, 1], flat, 0.7)
  assert x == pytest.approx(60.0, rel=1e-3) and y == pytest.approx(1.0, abs=0.01)


def test_tall_lamps_side_by_side_are_a_pair_not_posts():
  # far headlights smeared vertically (glare, wet road) look like delineator posts one by one
  mask = np.full((FCAM_H, FCAM_W), 255, np.uint8)
  img = dark_frame()
  img[400:405, 600:602] = 255
  img[400:405, 610:612] = 255
  img[400:405, 900:902] = 255          # a lone post
  blobs = sorted(find_blobs(img, mask, HbaParams()), key=lambda b: b.u)
  assert [(b.paired, b.post) for b in blobs] == [(True, False), (True, False), (False, True)]


def test_lit_yellow_signs_are_not_lamps():
  mask = np.full((FCAM_H, FCAM_W), 255, np.uint8)
  img = dark_frame()
  img[300:312, 300:312] = (0, 230, 255)   # yellow sign, bright enough in luma to pass the white test
  img[304:308, 304:308] = 255             # ...with an overexposed middle
  draw_lamp(img, 501, 301, 2)             # a headlight
  blobs = find_blobs(img, mask, HbaParams())
  assert len(blobs) == 1 and abs(blobs[0].u - 501.5) < 1


def _pair_frames(geom, n, radius, value, glow=True):
  frames = []
  for _ in range(n):
    img = dark_frame()
    for y in (-3.0, -4.5):
      add_light(img, geom, 120.0, y, 0.7, radius=radius, color=(value, value, value), glow=glow)
    frames.append(img)
  return frames


@pytest.mark.parametrize("reflector", [True, False])
def test_lights_that_follow_our_high_beam_are_reflectors(reflector):
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  # a far lamp pair in the oncoming lane: dips
  outs = run(clf, _pair_frames(geom, 30, 1, 235), high_beam_on=False)
  assert not outs[-1].high_beam_permitted and outs[-1].reason & REASON_ONCOMING
  # our high beam comes on (driver or J519): a reflector lights up, a lamp doesn't care
  after = _pair_frames(geom, 100, 2 if reflector else 1, 255 if reflector else 235, glow=not reflector)
  outs = run(clf, after, t0=1.5, high_beam_on=True)
  kinds = {tr.kind for tr in clf.tracks if tr.missed == 0}
  if reflector:
    assert kinds <= {KIND_STATIC} and outs[-1].high_beam_permitted
  else:
    assert KIND_STATIC not in kinds and not any(o.high_beam_permitted for o in outs)


def test_restores_cut_short_by_weak_lights_back_off():
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  weak = _pair_frames(geom, 12, 1, 235)    # far, small: dips after confirm_s
  t = 0.0
  for expected in (2.0, 4.0, 8.0, 8.0):
    outs = run(clf, [dark_frame()] * int((expected + 0.5) / DT), t0=t)
    t += len(outs) * DT
    assert outs[-1].high_beam_permitted and outs[-1].clear_needed_s == expected
    outs = run(clf, weak, t0=t)            # cut short within a second of the restore
    assert not outs[-1].high_beam_permitted
    t += len(weak) * DT
  # a long clear stretch resets it
  outs = run(clf, [dark_frame()] * int((8.5 + clf.p.backoff_reset_s) / DT), t0=t)
  assert outs[-1].clear_needed_s == clf.p.clear_s


def test_a_passing_car_resets_the_restore_backoff():
  # route 90: cars cut two restores short, and the backoff then waited 8 s after every car that went past
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 50)
  outs = run(clf, _pair_frames(geom, 12, 1, 235), t0=2.5)
  assert not outs[-1].high_beam_permitted and outs[-1].clear_needed_s == 2 * clf.p.clear_s
  passing = [frame_with_light(geom, 150.0 - i * 130.0 / 80, -3.6, 0.7, radius=4) for i in range(80)]
  run(clf, passing, t0=3.1)
  assert _restore_after(clf, 7.1) == pytest.approx(clf.p.clear_pass_s, abs=2 * DT)
  assert clf.policy.clear_needed == clf.p.clear_s


def test_only_a_lead_right_after_a_restore_skips_the_backoff():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  lamp = [frame_with_light(geom, 60, -3.6, 0.7, radius=4)] * 4
  # a strong lone lamp right after a restore is most often the sign our beam just lit: it dips, and backs off
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 50)
  outs = run(clf, lamp, t0=2.5)
  assert not outs[-1].high_beam_permitted and outs[-1].clear_needed_s == 2 * clf.p.clear_s
  # a model lead is a car whatever the timing
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 50)
  outs = run(clf, lamp, t0=2.5, model=dict(STRAIGHT, leads=[(0.9, 60.0, 0.0)]))
  assert not outs[-1].high_beam_permitted and outs[-1].clear_needed_s == clf.p.clear_s


def test_a_light_born_at_our_switch_on_is_a_reflector_until_it_pairs():
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  outs = run(clf, [dark_frame()] * 50, high_beam_on=False)
  assert outs[-1].high_beam_permitted
  # the shutter opens and a big lone lamp-like light is there at once: the sign we lit, however bright it blooms
  sign = frame_with_light(geom, 150.0, -3.6, 0.7, radius=6, glow=True)
  outs = run(clf, [sign] * 20, t0=2.5, high_beam_on=True)
  assert all(o.high_beam_permitted for o in outs)
  assert {tr.why for tr in clf.tracks if tr.missed == 0} == {"beam"}
  # it was a car after all: a lamp pair dips
  outs = run(clf, _pair_frames(geom, 10, 3, 255), t0=3.5, high_beam_on=True)
  assert not outs[-1].high_beam_permitted


def _first_dip(outs):
  return next((i for i, o in enumerate(outs) if not o.high_beam_permitted), None)


def test_a_far_bright_light_builds_evidence_slowly_under_our_high_beam():
  # the signs our high beam lights are far, big and saturated: a far bright light needs 1 / ev_strong_hb s under it,
  # 1 / ev_strong_far without it
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  lamp = [frame_with_light(geom, 150.0, -3.6, 0.7, radius=4)] * 40
  times = {}
  for hb in (False, True):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * 120, high_beam_on=hb)
    outs = run(clf, lamp, t0=6.0, high_beam_on=hb)
    times[hb] = (_first_dip(outs) + 1) * DT
  assert times[False] <= 1.0 / clf.p.ev_strong_far + DT + 1e-9
  assert times[True] == pytest.approx(1.0 / clf.p.ev_strong_hb, abs=2 * DT)


def test_a_near_bright_light_dips_at_once_whatever_our_beam():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  for hb in (False, True):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * 120, high_beam_on=hb)
    near = [frame_with_light(geom, 40.0 - i * 1.5, -3.6, 0.7, radius=6) for i in range(10)]
    outs = run(clf, near, t0=6.0, high_beam_on=hb)
    assert (_first_dip(outs) + 1) * DT <= 1.0 / clf.p.ev_strong + DT + 1e-9


def test_a_far_lone_dot_must_persist_and_keeps_its_evidence_through_a_flicker():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  dot = frame_with_light(geom, 160.0, -3.6, 0.7, radius=1)  # a weak far lone light: what a delineator or a far car is
  need = 1.0 / HbaParams().ev_lone
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 120)
  n = round(0.6 * need / DT)
  outs = run(clf, [dot] * n, t0=6.0)
  assert _first_dip(outs) is None  # not long enough
  # it flickers out for 0.1 s: a re-detection inherits what it had built (less ev_missing * 0.1 s), so 0.8 of the
  # time it needs on its own is enough now
  t = 6.0 + n * DT
  run(clf, [dark_frame()] * 2, t0=t)
  outs = run(clf, [dot] * round(0.8 * need / DT), t0=t + 2 * DT)
  assert _first_dip(outs) is not None
  assert outs[-1].evidence >= 1.0 and outs[-1].reason & REASON_ONCOMING


def _restore_after(clf, t0):
  outs = run(clf, [dark_frame()] * 120, t0=t0)
  return (next(i for i, o in enumerate(outs) if o.high_beam_permitted) + 1) * DT


def test_restore_is_fast_after_a_pass_and_slower_when_a_light_vanishes_far():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  # an oncoming car closing from 150 m to 20 m over 4 s (longer than the minimum low time), then it's past us
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 120)
  passing = [frame_with_light(geom, 150.0 - i * 130.0 / 80, -3.6, 0.7, radius=4) for i in range(80)]
  assert not run(clf, passing, t0=6.0)[-1].high_beam_permitted
  assert _restore_after(clf, 10.0) == pytest.approx(clf.p.clear_pass_s, abs=2 * DT)
  # the same light vanishing at 150 m (a bend, a crest): it usually comes back
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 120)
  assert not run(clf, [frame_with_light(geom, 150.0, -3.6, 0.7, radius=4)] * 80, t0=6.0)[-1].high_beam_permitted
  assert _restore_after(clf, 10.0) == pytest.approx(clf.p.clear_far_s, abs=2 * DT)


def _bend(k, horizon):
  """A road bending right, y = k x^2, that the model sees out to horizon m."""
  xs = np.linspace(0, horizon, 17)
  return {**STRAIGHT, "position": np.column_stack([xs, k * xs ** 2, np.zeros(17)]),
          "lanes": [np.column_stack([xs, y + k * xs ** 2, np.full(17, HEIGHT)]) for y in (-5.4, -1.8, 1.8, 5.4)]}


def test_a_light_beside_a_straight_road_is_roadside_but_not_one_round_a_bend_towards_our_side():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  clf = HbaClassifier()
  run(clf, [frame_with_light(geom, 40.0, 6.0, 0.7, radius=1)] * 6)
  assert {tr.why for tr in clf.tracks if tr.missed == 0} == {"roadside"}
  # route 90 seg 21: a car in the oncoming lane 110 m round a right bend the model sees only out to 60 m, low lamps. Past
  # 60 m the path goes straight on and the one-row range says 70 m, so the car is 6 degrees right of the road there; its
  # ray is over our lanes at lamp height 8x farther out
  k, x = 3e-3, 110.0
  clf = HbaClassifier()
  run(clf, [frame_with_light(geom, x, k * x * x - 3.6, 0.4, radius=1)] * 6, model=_bend(k, 60.0))
  live = [tr for tr in clf.tracks if tr.missed == 0]
  assert live and all(tr.hist[-1].bend > clf.p.roadside_bend_k and tr.why != "roadside" for tr in live)


def test_a_short_dip_stays_low_for_the_minimum_time():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  clf = HbaClassifier()
  run(clf, [dark_frame()] * 120)
  near = [frame_with_light(geom, 40.0 - i * 1.5, -3.6, 0.7, radius=6) for i in range(6)]  # 0.3 s, then past us
  outs = run(clf, near, t0=6.0)
  dip = _first_dip(outs)
  low = dip * DT + _restore_after(clf, 6.0 + len(near) * DT) + (len(near) - dip - 1) * DT
  assert low == pytest.approx(clf.p.min_low_s[0], abs=2 * DT)  # FAST is above min_low_v[1]


def _lit_sign_frame(geom, x, y, core=5, face=3, bg=45):
  """A sign lit by our high beam: a burnt-out white middle in a yellow face that ends at its edge, on the faintly lit
  night scene our high beam makes (route 6f: Y ~55 around signs)."""
  img = np.full((FCAM_H, FCAM_W, 3), bg, np.uint8)
  uv, ok = geom.project(np.array([[x, y, HEIGHT - 0.7]]))
  u, v = (int(round(c)) for c in uv[0])
  r = core + face
  img[v - r:v + r + 1, u - r:u + r + 1] = (0, 200, 230)
  img[v - core:v + core + 1, u - core:u + core + 1] = 255
  return img


def test_a_lit_sign_is_static_and_a_blooming_lamp_is_not():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  # the sign: compact glow, under our high beam, straight ahead where the road bends away
  clf = HbaClassifier()
  run(clf, [np.full((FCAM_H, FCAM_W, 3), 45, np.uint8)] * 120, high_beam_on=True)
  outs = run(clf, [_lit_sign_frame(geom, 150.0, 0.0)] * 40, t0=6.0, high_beam_on=True)
  assert all(o.high_beam_permitted for o in outs)
  assert {tr.why for tr in clf.tracks if tr.missed == 0} == {"sign"}
  # a lamp the same size whose flare spreads far past its core dips
  clf = HbaClassifier()
  run(clf, [np.full((FCAM_H, FCAM_W, 3), 45, np.uint8)] * 120, high_beam_on=True)
  lamp = add_light(np.full((FCAM_H, FCAM_W, 3), 45, np.uint8), geom, 150.0, -3.6, 0.7, radius=5)
  outs = run(clf, [lamp] * 40, t0=6.0, high_beam_on=True)
  assert not outs[-1].high_beam_permitted


def test_a_compact_lamp_pair_is_not_a_sign():
  # smeared or side-on headlights can look compact; a sign doesn't have a twin on its row
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  pair = np.maximum(_lit_sign_frame(geom, 150.0, -2.8, core=3, face=1), _lit_sign_frame(geom, 150.0, -4.8, core=3, face=1))
  clf = HbaClassifier()
  run(clf, [np.full((FCAM_H, FCAM_W, 3), 45, np.uint8)] * 120, high_beam_on=True)
  outs = run(clf, [pair] * 40, t0=6.0, high_beam_on=True)
  assert not outs[-1].high_beam_permitted
  assert "sign" not in {tr.why for tr in clf.tracks if tr.missed == 0}


def test_a_faint_near_light_waits_for_the_motion_fit_under_our_high_beam():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  dot = frame_with_light(geom, 45.0, -3.6, 0.7, radius=1)  # what our high beam lights 45 m out: a delineator
  t_dip = {}
  for hb in (False, True):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * 120, high_beam_on=hb)
    outs = run(clf, [dot] * 20, t0=6.0, high_beam_on=hb)
    i = _first_dip(outs)
    t_dip[hb] = math.inf if i is None else (i + 1) * DT
  assert t_dip[False] <= 1.0 / HbaParams().ev_near + DT + 1e-9
  assert t_dip[True] >= min(1.0 / HbaParams().ev_near_hb, HbaParams().motion_s) - DT


def test_legacy_policy_is_still_there():
  # evidence=False is the pre-evidence decision (tools/policy_sim.py compares against it)
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  clf = HbaClassifier(HbaParams(evidence=False))
  run(clf, [dark_frame()] * 120)
  outs = run(clf, [frame_with_light(geom, 150.0, -3.6, 0.7, radius=4)] * 5, t0=6.0)
  assert _first_dip(outs) == 0


def test_timing_does_not_depend_on_the_frame_rate():
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  dips = []
  for dt in (0.05, 0.1):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * int(3.0 / dt), dt=dt)
    outs = run(clf, _pair_frames(geom, int(2.0 / dt), 1, 235), t0=3.0, dt=dt)
    dips.append(next(i for i, o in enumerate(outs) if not o.high_beam_permitted) * dt)
  assert abs(dips[0] - dips[1]) <= 0.1 + 1e-9


def test_gyro_rotation_moves_distant_points_the_right_way():
  # livePose angular velocity is in the device frame (x fwd, y right, z down): +z is a yaw to the right, so the
  # scene moves left; +y pitches the nose up, so the scene moves down. Checked against logged drives (replay_hba).
  u0, v0 = FCAM_W / 2, FCAM_H / 2
  yaw = rotate_pixel(u0, v0, cv2.Rodrigues(np.array([0.0, 0.0, 0.01]))[0])
  pitch = rotate_pixel(u0, v0, cv2.Rodrigues(np.array([0.0, 0.01, 0.0]))[0])
  assert yaw[0] < u0 - 10 and abs(yaw[1] - v0) < 1e-6
  assert pitch[1] > v0 + 10 and abs(pitch[0] - u0) < 1e-6


WARM = (200, 230, 255)    # BGR: a sodium/warm-white streetlight or window, bright and barely saturated
GREEN = (200, 240, 150)   # a lit green guide sign: no more saturated than a warm lamp, but green


def _street_frame(geom, n, color=WARM):
  # n lights 7 m over the road, 60-135 m out, well off the road on both sides
  img = dark_frame()
  for i in range(n):
    add_light(img, geom, 60.0 + 25 * i, (-12.0, 12.0)[i % 2], 7.0, radius=2, color=color)
  return img


@pytest.mark.parametrize("n, color, town", [(4, WARM, True), (1, WARM, False), (4, GREEN, False)])
def test_streetlight_density_is_town(n, color, town):
  # a row of streetlights is town; one lone yard light on the open road isn't, nor are lit green signs
  clf = HbaClassifier()
  outs = run(clf, [_street_frame(Geometry([0.0, 0.0, 0.0], HEIGHT), n, color)] * int(5.0 / DT))
  assert outs[-1].town == town and bool(outs[-1].reason & REASON_TOWN) == town
  assert outs[-1].high_beam_permitted != town


def test_town_holds_through_gaps_and_then_restores():
  clf = HbaClassifier()
  p = clf.p
  outs = run(clf, [_street_frame(Geometry([0.0, 0.0, 0.0], HEIGHT), 4)] * int(5.0 / DT))
  assert outs[-1].town
  # past the last streetlight: still town for town_exit_s once the density has dropped, then the usual clear time
  after = run(clf, [dark_frame()] * int((p.town_window_s + p.town_exit_s + p.clear_s + 1.0) / DT), t0=5.0)
  assert all(o.town and not o.high_beam_permitted for o in after[:int(p.town_exit_s / DT)])
  left = next(i for i, o in enumerate(after) if not o.town)
  assert left * DT <= p.town_window_s + p.town_exit_s
  assert not after[left].high_beam_permitted and after[-1].high_beam_permitted and not after[-1].reason


@pytest.mark.parametrize("limit, town", [(50 / 3.6, True), (30 / 3.6, True), (80 / 3.6, False), (0.0, False), (None, False)])
def test_posted_limit_is_town(limit, town):
  clf = HbaClassifier()
  outs = run(clf, [dark_frame()] * int(3.0 / DT), speed_limit=limit)
  assert bool(outs[-1].reason & REASON_TOWN) == town and outs[-1].high_beam_permitted != town


def test_crisp_white_blobs_are_signs_not_lamps():
  # route 46: white signs and post reflectors have a dark surround (median V 13-24), headlights bloom (41-128)
  mask = np.full((FCAM_H, FCAM_W), 255, np.uint8)
  img = dark_frame()
  draw_lamp(img, 300, 300, 4, glow=False)   # speed limit sign lit by our beams
  draw_lamp(img, 600, 300, 4)               # headlight
  draw_lamp(img, 900, 300, 1, glow=False)   # small and far: too small to judge, kept
  blobs = sorted(find_blobs(img, mask, HbaParams()), key=lambda b: b.u)
  assert [round(b.u) for b in blobs] == [600, 900]


WIDE_EULER = [0.0, 0.0, 0.0]
NEUTRAL = (128, 128)
YELLOW = (60, 170)  # NV12 (U, V) of a lit yellow sign


def dark_wide():
  return Nv12Frame(np.full((ECAM_H, ECAM_W), 16, np.uint8), np.full((ECAM_H // 2, ECAM_W), 128, np.uint8))


def wide_lamp(frame, x, y, z_above_road=0.7, radius=4, glow=True, uv=NEUTRAL):
  """A light at (x, y) m drawn into a wide NV12 frame: a saturated core, a dim bloom around it if glow."""
  g = Geometry([0.0, 0.0, 0.0], HEIGHT, WIDE_EULER)
  pt, ok = g.project(np.array([[x, y, HEIGHT - z_above_road]]))
  assert ok[0]
  u, v = (int(round(c)) for c in pt[0])
  if glow:
    b = 3 * radius + 2
    frame.y[v - b:v + b + 1, u - b:u + b + 1] = 90
  frame.y[v - radius:v + radius + 1, u - radius:u + radius + 1] = 240
  frame.uv[(v - radius) // 2:(v + radius) // 2 + 1, (u - radius) & ~1:(u + radius + 2) & ~1] = np.tile(uv, radius + 1)
  return frame, (u, v)


def look_wide(frame):
  p = HbaParams()
  wp = wide_params(p)
  return wide_lights(frame, Geometry([0.0, 0.0, 0.0], HEIGHT, WIDE_EULER), road_centerline(STRAIGHT, wp, HEIGHT), wp,
                     Geometry([0.0, 0.0, 0.0], HEIGHT))


def test_wide_geometry_sees_past_the_narrow_view():
  narrow, wide = Geometry([0.0, 0.0, 0.0], HEIGHT), Geometry([0.0, 0.0, 0.0], HEIGHT, WIDE_EULER)
  ahead = np.array([[100.0, 0.0, 0.0]])
  assert np.allclose(wide.project(ahead)[0], [[ECAM_W / 2, ECAM_H / 2]])
  # 37 degrees to the left, 8 m out: past the narrow camera's edge, well inside the wide one
  side = np.array([[8.0, -6.0, HEIGHT - 0.7]])
  (un, _), = narrow.project(side)[0]
  (uw, vw), = wide.project(side)[0]
  assert un < 0 and 0 < uw < ECAM_W / 2 and ECAM_H / 2 < vw < ECAM_H
  # the device-to-wide rotation is applied: yawing the wide camera moves the point in its image
  yawed = Geometry([0.0, 0.0, 0.0], HEIGHT, [0.0, 0.0, 0.05])
  assert abs(yawed.project(side)[0][0, 0] - uw) > 10
  # the strided projection is the full one scaled
  assert np.allclose(wide.strided(2).project(side)[0], wide.project(side)[0] / 2)


def test_wide_guard_finds_blooming_lamps_left_of_the_narrow_view():
  frame, (u, v) = wide_lamp(dark_wide(), 8.0, -6.0)
  lights = look_wide(frame)
  assert len(lights) == 1 and lights[0].lamp
  assert abs(lights[0].u - u) <= 2 and abs(lights[0].v - v) <= 2


@pytest.mark.parametrize("x, y, kw", [
  (8.0, -6.0, {"glow": False}),       # a sign or reflector lit by our beams: crisp edges
  (8.0, -6.0, {"uv": YELLOW}),        # a lit yellow sign
  (8.0, -6.0, {"radius": 1}),         # too small to be sure
  (6.5, -6.5, {"radius": 8}),         # a car passing beside us: our beam doesn't reach its driver
])
def test_wide_guard_ignores_signs_and_small_lights(x, y, kw):
  assert not any(w.lamp for w in look_wide(wide_lamp(dark_wide(), x, y, **kw)[0]))


@pytest.mark.parametrize("x, y", [
  (60.0, -3.6),   # well inside the narrow view: that pipeline knows better
  (8.0, 6.0),     # on our side of the path: parked cars, reflectors, driveways
  (8.0, -12.0),   # beyond the oncoming lane
])
def test_wide_guard_looks_only_outside_the_narrow_view_on_the_oncoming_side(x, y):
  assert look_wide(wide_lamp(dark_wide(), x, y)[0]) == []


def test_wide_lamp_dips_at_once_and_holds_between_looks():
  clf = HbaClassifier()
  outs = run(clf, [dark_frame()] * 60)
  assert outs[-1].high_beam_permitted
  car = wide_lamp(dark_wide(), 8.0, -6.0)[0]
  # the narrow frame is empty; the wide guard looks every fourth frame (0.2 s)
  outs = [clf.update(3.0 + i * DT, dark_frame(), STRAIGHT, [0.0, 0.0, 0.0], HEIGHT, FAST,
                     **({"wide": car, "wide_euler": WIDE_EULER} if i % 4 == 0 else {})) for i in range(8)]
  assert not outs[0].high_beam_permitted and outs[0].reason & REASON_ONCOMING and outs[0].n_wide == 1
  assert all(o.threat and not o.high_beam_permitted for o in outs)
  # it's gone: the hit holds wide_hold_s, then the usual clear time
  after = [clf.update(3.4 + i * DT, dark_frame(), STRAIGHT, [0.0, 0.0, 0.0], HEIGHT, FAST,
                      **({"wide": dark_wide(), "wide_euler": WIDE_EULER} if i % 4 == 0 else {})) for i in range(60)]
  clear = next(i for i, o in enumerate(after) if not o.threat)
  assert clear * DT <= clf.p.wide_hold_s + DT
  assert not after[clear].high_beam_permitted and after[-1].high_beam_permitted


def test_wide_guard_needs_its_calibration_and_the_dark():
  car = wide_lamp(dark_wide(), 8.0, -6.0)[0]
  for kw in ({"wide": car}, {"wide": car, "wide_euler": WIDE_EULER, "dark": False}):
    clf = HbaClassifier()
    run(clf, [dark_frame()] * 60)
    out = clf.update(3.0, dark_frame(), STRAIGHT, [0.0, 0.0, 0.0], HEIGHT, FAST, **kw)
    assert not out.threat and out.n_wide == 0


# --- the CPU shortcuts: same results as the plain computations they replace ---

CURVE = {**STRAIGHT,
         "lanes": [np.column_stack([np.linspace(0, 190, 33), y + 4e-4 * np.linspace(0, 190, 33) ** 2,
                                    np.full(33, HEIGHT) - 0.01 * np.linspace(0, 190, 33)]) for y in (-5.4, -1.8, 1.8, 5.4)]}


def corridor_mask_reference(geom, center, p, far_extra=0.0, lateral=None):
  """corridor_mask as it was before vectorizing: one piece at a time."""
  mask = np.zeros((geom.h, geom.w), np.uint8)
  far_side = p.far_side_width + far_extra
  left = p.ego_half_width + (p.oncoming_width if p.oncoming_side < 0 else far_side)
  right = p.ego_half_width + (p.oncoming_width if p.oncoming_side > 0 else far_side)
  if lateral is not None:
    left, right = lateral
  corners = []
  for x, y, z, beyond in center:
    wl, wz = p.beyond_lateral * beyond, p.beyond_vertical * beyond
    corners.append([(x, y - left - wl, z - p.lamp_low + wz), (x, y + right + wl, z - p.lamp_low + wz),
                    (x, y - left - wl, z - p.lamp_high - wz), (x, y + right + wl, z - p.lamp_high - wz)])
  corners = np.array(corners)
  uv, valid = geom.project(corners.reshape(-1, 3))
  uv, valid = uv.reshape(-1, 4, 2), valid.reshape(-1, 4)
  for i in range(len(corners) - 1):
    if not (valid[i].all() and valid[i + 1].all()):
      continue
    poly = np.concatenate([uv[i], uv[i + 1]])
    if np.abs(poly).max() > 1e5:
      continue
    hull = cv2.convexHull(poly.astype(np.float32)).astype(np.int32)
    cv2.fillConvexPoly(mask, hull, 255)
    if p.margin_px > 0:
      cv2.polylines(mask, [hull], True, 255, thickness=2 * p.margin_px + 1, lineType=cv2.LINE_8)
  return mask


@pytest.mark.parametrize("model", [STRAIGHT, CURVE])
@pytest.mark.parametrize("rpy", [[0.0, 0.0, 0.0], [0.02, -0.04, -0.03], [0.0, 0.3, 0.0]])
def test_corridor_mask_matches_reference_and_its_box(model, rpy):
  p = HbaParams()
  center = road_centerline(model, p, HEIGHT)
  narrow = Geometry(rpy, HEIGHT)
  wp = wide_params(p)
  cases = [(narrow, center, p, {}), (narrow, center, p, {"far_extra": p.adjacent_lane_width}),
           (narrow, center, HbaParams(margin_px=0), {}),
           (Geometry(rpy, HEIGHT, WIDE_EULER).strided(2), road_centerline(model, wp, HEIGHT), wp, {"lateral": (7.5, 0.0)})]
  for geom, c, q, kw in cases:
    ref = corridor_mask_reference(geom, c, q, **kw)
    mask, box = corridor_mask(geom, c, q, with_box=True, **kw)
    assert np.array_equal(mask, ref) and np.array_equal(corridor_mask(geom, c, q, **kw), ref)
    assert tuple(box) == tuple(cv2.boundingRect(ref))


def test_union_box():
  assert union_box((0, 0, 0, 0), (3, 4, 5, 6)) == (3, 4, 5, 6)
  assert union_box((3, 4, 5, 6), (0, 0, 0, 0)) == (3, 4, 5, 6)
  a, b = np.zeros((50, 60), np.uint8), np.zeros((50, 60), np.uint8)
  a[5:9, 40:44], b[20:30, 3:7] = 255, 255
  assert union_box(cv2.boundingRect(a), cv2.boundingRect(b)) == cv2.boundingRect(a | b)


def test_median_matches_numpy():
  rng = np.random.default_rng(1)
  for n in (1, 2, 3, 4, 7, 10, 51):
    for a in (rng.integers(0, 256, n).astype(np.uint8), np.full(n, 255, np.uint8), rng.integers(-300, 300, n).astype(np.int16),
              rng.normal(size=n), rng.integers(0, 256, (n, 3)).astype(np.uint8)[:, 1], list(rng.normal(size=n))):
      assert median(a) == np.median(a)
    angles = rng.normal(size=n)
    angles[::3] = np.nan
    if np.isfinite(angles).any():
      assert median(angles[np.isfinite(angles)]) == np.nanmedian(angles)
  assert np.isnan(median(np.zeros(0)))


def test_road_centerline_helpers_match_numpy():
  for args in ((15.0, np.float32(192.0), 24), (np.float32(197.0), 600.0, 10), (15.0, 16.0, 24), (6.0, 300.0, 200)):
    for _ in range(2):  # memoized the second time
      r = _geomspace(*args)
      assert r.dtype == np.geomspace(*args).dtype and np.array_equal(r, np.geomspace(*args))
  rng = np.random.default_rng(2)
  for dtype in (np.float32, np.float64):
    tail = np.column_stack([np.linspace(120, 192, 6), rng.normal(0, 3, 6), rng.normal(1.2, 0.5, 6)]).astype(dtype)
    sy, sz = _slopes(tail)
    assert sy == pytest.approx(np.polyfit(tail[:, 0], tail[:, 1], 1)[0], rel=1e-9, abs=1e-12)
    assert sz == pytest.approx(np.polyfit(tail[:, 0], tail[:, 2], 1)[0], rel=1e-9, abs=1e-12)


def test_candidate_table_covers_every_passing_pixel():
  """Every (Y, U, V) whose pixel passes the white or red test is at or above the table's Y for its chroma pair."""
  p = HbaParams()
  table = candidate_ymin(p).reshape(256, 256)  # [V, U]
  uv = np.empty((256, 512), np.uint8)
  uv[:, 0::2], uv[:, 1::2] = np.arange(256, dtype=np.uint8)[None, :], np.arange(256, dtype=np.uint8)[:, None]
  for y in range(256):
    bgr = cv2.cvtColor(np.concatenate([np.full((512, 512), y, np.uint8), uv]), cv2.COLOR_YUV2BGR_NV12)[::2, ::2]
    white, red = colour_masks(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), p)
    assert np.all(table[(white > 0) | (red > 0)] <= y), y
  # and it is selective: a dark or neutral block never is a candidate
  assert table[128, 128] >= 190 and np.mean(table > 60) > 0.75


def nv12_scene(seed, n):
  """A dark frame with n lights (lamps, reflectors, taillights, yellow signs) and sensor noise, as NV12."""
  rng = np.random.default_rng(seed)
  bgr = np.clip(rng.normal(14, 4, (FCAM_H, FCAM_W, 3)), 0, 255).astype(np.uint8)
  for _ in range(n):
    u, v, r, kind = int(rng.integers(150, FCAM_W - 150)), int(rng.integers(330, 430)), int(rng.integers(1, 6)), rng.integers(4)
    if kind == 0:
      draw_lamp(bgr, u, v, r)
    elif kind == 1:
      draw_lamp(bgr, u, v, r, glow=False)
    elif kind == 2:  # taillight: red halo, near-white core
      bgr[v - r - 2:v + r + 3, u - r - 2:u + r + 3] = (30, 30, 230)
      bgr[v - r // 2:v + r // 2 + 1, u - r // 2:u + r // 2 + 1] = (200, 200, 255)
    else:
      bgr[v - 2 * r:v + 2 * r, u - 2 * r:u + 2 * r] = (0, 200, 255)
  i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420)
  h, w = FCAM_H, FCAM_W
  uv = np.empty((h // 2, w), np.uint8)
  uv[:, 0::2] = i420[h:h + h // 4].reshape(h // 2, w // 2)
  uv[:, 1::2] = i420[h + h // 4:].reshape(h // 2, w // 2)
  return Nv12Frame(i420[:h], uv), cv2.cvtColor(np.concatenate([i420[:h], uv]), cv2.COLOR_YUV2BGR_NV12)


@pytest.mark.parametrize("seed,n", [(0, 12), (1, 12), (2, 25), (3, 80)])
def test_nv12_blobs_match_full_conversion(seed, n):
  """Converting only around the candidate pixels finds the same blobs, in the same order, as converting it all."""
  p = HbaParams()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  center = road_centerline(STRAIGHT, p, HEIGHT)
  mask, box = corridor_mask(geom, center, p, with_box=True)
  red_mask, red_box = corridor_mask(geom, center, p, p.adjacent_lane_width, with_box=True)
  frame, bgr = nv12_scene(seed, n)
  for rm, b in ((None, box), (red_mask, union_box(box, red_box))):
    ref = find_blobs(bgr, mask, p, rm)
    assert len(ref) > 0
    assert find_blobs(frame, mask, p, rm) == ref
    assert find_blobs(frame, mask, p, rm, b) == ref


def test_town_lights_box_and_pooling():
  rng = np.random.default_rng(4)
  band = rng.integers(0, 256, (200, 1344)).astype(np.uint8)
  pooled = cv2.resize(cv2.threshold(band, 179, 255, cv2.THRESH_BINARY)[1], (672, 100), interpolation=cv2.INTER_AREA) > 0
  maxpool = np.maximum(np.maximum(band[0::2, 0::2], band[1::2, 0::2]), np.maximum(band[0::2, 1::2], band[1::2, 1::2])) >= 180
  assert np.array_equal(pooled, maxpool)
  p = HbaParams()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  center = road_centerline(STRAIGHT, p, HEIGHT)
  mask, box = corridor_mask(geom, center, p, with_box=True)
  img = dark_frame()
  for u, v in rng.integers((20, 20), (FCAM_W - 20, 330), (40, 2)):
    draw_lamp(img, int(u), int(v), int(rng.integers(1, 4)), color=(240, 240, 240))
  n = town_lights(img, geom, center, mask, p)
  assert n > 10 and town_lights(img, geom, center, mask, p, box) == n
