import cv2
import numpy as np
import pytest

from openpilot.starpilot.system.lightd.hba import (FCAM_H, FCAM_W, Geometry, HbaClassifier, Nv12Frame, REASON_DELAY,
                                                    REASON_FAULT, REASON_ONCOMING, REASON_SPEED, corridor_mask, find_blobs,
                                                    HbaParams, road_centerline, rot_from_euler)

HEIGHT = 1.22
STRAIGHT = {
  "position": np.column_stack([np.linspace(0, 190, 33), np.zeros(33), np.zeros(33)]),
  "lanes": [np.column_stack([np.linspace(0, 190, 33), np.full(33, y), np.zeros(33)]) for y in (-5.4, -1.8, 1.8, 5.4)],
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


def run(clf, frames, v=FAST, t0=0.0, model=STRAIGHT):
  return [clf.update(t0 + i * DT, f, model, [0.0, 0.0, 0.0], HEIGHT, v) for i, f in enumerate(frames)]


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
  mask = corridor_mask(geom, road_centerline(STRAIGHT, p), p)
  undilated = corridor_mask(geom, road_centerline(STRAIGHT, p), HbaParams(margin_px=0))
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
  assert dip <= clf.p.dip_frames
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
