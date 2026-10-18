import cv2
import numpy as np
import pytest

from openpilot.starpilot.system.lightd.hba import (FCAM_H, FCAM_W, Geometry, HbaClassifier, Nv12Frame, REASON_DELAY,
                                                    REASON_FAULT, REASON_ONCOMING, REASON_SPEED, REASON_TOWN, corridor_mask,
                                                    find_blobs, HbaParams, KIND_STATIC, road_centerline, rot_from_euler,
                                                    rotate_pixel)

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


def frame_with_light(geom, x, y, z_above_road, radius=3, color=(255, 255, 255)):
  img = dark_frame()
  uv, ok = geom.project(np.array([[x, y, HEIGHT - z_above_road]]))
  assert ok[0]
  u, v = (int(round(c)) for c in uv[0])
  img[v - radius:v + radius + 1, u - radius:u + radius + 1] = color
  return img


def run(clf, frames, v=FAST, t0=0.0, model=STRAIGHT, dt=DT, **kw):
  return [clf.update(t0 + i * dt, f, model, [0.0, 0.0, 0.0], HEIGHT, v, **kw) for i, f in enumerate(frames)]


def add_light(img, geom, x, y, z_above_road, radius=3, color=(255, 255, 255)):
  uv, ok = geom.project(np.array([[x, y, HEIGHT - z_above_road]]))
  assert ok[0]
  u, v = (int(round(c)) for c in uv[0])
  img[v - radius:v + radius + 1, u - radius:u + radius + 1] = color
  return img


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
  img[300:304, 300:304] = (255, 255, 255)        # headlight
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
  assert undilated.any() and np.array_equal(mask, cv2.dilate(undilated, k))


def test_dark_empty_road_restores_high_beam_after_clear_time():
  clf = HbaClassifier()
  outs = run(clf, [dark_frame()] * 80)
  first = next(i for i, o in enumerate(outs) if o.high_beam_permitted)
  assert (first + 1) * DT == pytest.approx(clf.p.clear_s + DT, abs=2 * DT)
  assert outs[5].reason & REASON_DELAY


def test_oncoming_headlight_dips_fast_and_restores_slow():
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  run(clf, [dark_frame()] * 60)
  # big saturated headlight in the oncoming lane at 150 m, approaching
  frames = [frame_with_light(geom, 150 - i * 2.0, -3.6, 0.7, radius=4) for i in range(20)]
  outs = run(clf, frames, t0=3.0)
  dip = next(i for i, o in enumerate(outs) if not o.high_beam_permitted)
  assert dip <= 2
  assert outs[-1].reason & REASON_ONCOMING
  after = run(clf, [dark_frame()] * 60, t0=4.0)
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
  img[300:304, 500:504] = 255             # a headlight
  blobs = find_blobs(img, mask, HbaParams())
  assert len(blobs) == 1 and abs(blobs[0].u - 501.5) < 1


def _pair_frames(geom, n, radius, value):
  frames = []
  for _ in range(n):
    img = dark_frame()
    for y in (-3.0, -4.5):
      add_light(img, geom, 120.0, y, 0.7, radius=radius, color=(value, value, value))
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
  after = _pair_frames(geom, 100, 2 if reflector else 1, 255 if reflector else 235)
  outs = run(clf, after, t0=1.5, high_beam_on=True)
  kinds = {tr.kind for tr in clf.tracks if tr.missed == 0}
  if reflector:
    assert kinds == {KIND_STATIC} and outs[-1].high_beam_permitted
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


def test_a_real_car_right_after_a_restore_does_not_back_off():
  clf = HbaClassifier()
  geom = Geometry([0.0, 0.0, 0.0], HEIGHT)
  run(clf, [dark_frame()] * 50)
  outs = run(clf, [frame_with_light(geom, 60, -3.6, 0.7, radius=4)] * 4, t0=2.5)
  assert not outs[-1].high_beam_permitted and outs[-1].clear_needed_s == clf.p.clear_s


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
