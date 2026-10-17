"""High beam assist, prototype. StarPinguPilot.

Answers one question per narrow-camera frame: may the high beam be on? Everything uncertain means no.

Pipeline:
  1. corridor: project the model's road (lane lines / path, 0..~600 m) into the image, from road level up to
     lamp height, wide enough for our lane and the oncoming lane. Lights outside it (streetlights, windows)
     are ignored.
  2. blobs: bright white (headlights) and red (taillights) connected components inside the corridor.
  3. tracks: nearest-neighbour association. Distance comes from a lamp-height ground-plane assumption; the
     object's own speed comes from how fast inverse depth changes vs our speed. Roadside reflectors and other
     static lights move like the road (speed ~0), oncoming lamps approach faster than we drive.
  4. decision: any plausible vehicle dips at once; restoring needs a clear corridor for clear_s.
     Reasons use the FL1_Abblendursache bits from the PQ35 K-matrix (mFLA_1, 0x65A).
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

# comma four (mici, OS04C10) narrow camera, from common/transformations/camera.py. Kept local so this module
# doesn't pull in the compiled transformations extension and runs anywhere for offline replay.
FCAM_W, FCAM_H, FCAM_F = 1344, 760, 1522.0 * 3 / 4
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def rot_from_euler(rpy):
  """Same convention as common/transformations/orientation.py: R = Rz(yaw) Ry(pitch) Rx(roll)."""
  r, p, y = rpy
  cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  return rz @ ry @ rx

# FL1_Abblendursache bitmask
REASON_ONCOMING = 1
REASON_PRECEDING = 2
REASON_SPEED = 4
REASON_TOWN = 8
REASON_TOO_BRIGHT = 16
REASON_FOG = 32
REASON_OVERTEMP = 64
REASON_DELAY = 128
REASON_FAULT = 255

KIND_CANDIDATE, KIND_ONCOMING, KIND_PRECEDING, KIND_STATIC = "candidate", "oncoming", "preceding", "static"


@dataclass
class HbaParams:
  # corridor, model frame metres (x forward, y right)
  near_m: float = 15.0              # nearer vehicles are long since dipped for; near samples make the corridor huge
  far_m: float = 600.0
  oncoming_side: float = -1.0       # -1: oncoming traffic on the left (right-hand traffic)
  ego_half_width: float = 2.0
  oncoming_width: float = 4.5       # beyond the ego lane on the oncoming side
  far_side_width: float = 1.5
  lamp_low: float = 0.2             # lamp heights above the road
  lamp_high: float = 1.6
  margin_px: int = 10               # calibration / pitch slop
  lamp_height: float = 0.7          # for ground-plane distance
  # blobs
  white_y: int = 225
  red_hue: int = 8                  # OpenCV hue 0..180: red is <= red_hue or >= 180 - red_hue; amber/yellow is 12..35
  red_sat: int = 110
  red_v: int = 70
  red_peak_v: int = 190             # emissive: a taillight's core is bright; red paint lit by our beams is not
  min_area: int = 2
  max_area: int = 20000
  # tracking
  gate_px: float = 45.0
  max_missed: int = 3
  motion_frames: int = 8
  confirm_frames: int = 8            # weak blobs must persist this long before they dip (= motion_frames, so
                                     # roadside reflectors are already classified static by then)
  strong_area: int = 25              # ...unless big or saturated: those dip at once
  strong_peak: int = 250
  strong_peak_area: int = 8
  roadside_angle_deg: float = 3.3    # off-path angle on the non-oncoming side beyond which a light is roadside
                                     # (in-lane vehicles only get there inside ~40 m, where the model lead has them).
                                     # Angle = lateral / distance, so it survives the unknown lamp height.
  motion_max_dist: float = 70.0      # beyond this a lamp is a few px below the horizon and 1 px of row is tens of
                                     # metres, so motion can't classify it; far lights stay candidates (= dip)
  static_speed: float = 4.0          # |object speed| below this (m/s) after motion_frames: static light
  oncoming_speed: float = 5.0
  min_motion_v_ego: float = 8.0      # below this, motion can't separate static lights from vehicles
  # decision
  lead_prob: float = 0.5
  lead_max_x: float = 400.0
  dip_frames: int = 2
  clear_s: float = 2.0
  enable_v: float = 60 / 3.6
  disable_v: float = 50 / 3.6


@dataclass
class Blob:
  u: float
  v: float
  area: int
  peak_y: int
  red: bool
  bbox: tuple


@dataclass
class Track:
  tid: int
  hist: list = field(default_factory=list)  # (t, u, v, dist, lateral, area, red, peak, angle_deg)
  missed: int = 0
  kind: str = KIND_CANDIDATE
  obj_speed: float = float("nan")
  lateral: float = float("nan")
  dist: float = float("nan")
  age: int = 0

  @property
  def red_frac(self):
    return sum(h[6] for h in self.hist) / len(self.hist)

  def strong(self, p):
    area, peak = self.hist[-1][5], self.hist[-1][7]
    return area >= p.strong_area or (peak >= p.strong_peak and area >= p.strong_peak_area)

  def is_threat(self, p):
    return self.kind != KIND_STATIC and (self.age >= p.confirm_frames or self.strong(p))


@dataclass
class HbaOut:
  valid: bool = False
  high_beam_permitted: bool = False
  reason: int = REASON_FAULT
  threat: bool = False
  clear_for_s: float = 0.0
  n_oncoming: int = 0
  n_preceding: int = 0
  n_candidates: int = 0
  lead_prob: float = 0.0


class Geometry:
  """Projection between the calibrated model frame and narrow-camera pixels."""

  def __init__(self, rpy_calib, height):
    self.w, self.h = FCAM_W, FCAM_H
    self.K = np.array([[FCAM_F, 0.0, FCAM_W / 2], [0.0, FCAM_F, FCAM_H / 2], [0.0, 0.0, 1.0]])
    self.view_from_calib = VIEW_FROM_DEVICE @ rot_from_euler(rpy_calib)
    self.calib_from_view = self.view_from_calib.T
    self.K_inv = np.linalg.inv(self.K)
    self.height = height

  def project(self, pts):
    """pts: Nx3 in calib frame (x fwd, y right, z down, origin at camera). Returns Nx2 pixels, valid mask."""
    v = self.view_from_calib @ np.asarray(pts, dtype=np.float64).T
    valid = v[2] > 0.5
    uv = (self.K @ v)[:2] / np.where(valid, v[2], 1.0)
    return uv.T, valid

  def ray_to_plane(self, u, v, z_plane):
    """Intersect pixel ray with the horizontal plane z = z_plane (calib frame). Returns (x, y) or None."""
    d = self.calib_from_view @ (self.K_inv @ np.array([u, v, 1.0]))
    if d[2] <= 1e-4:
      return None
    s = z_plane / d[2]
    return s * d[0], s * d[1]

  def horizon_row(self):
    uv, _ = self.project(np.array([[1000.0, 0.0, 0.0]]))
    return uv[0, 1]


def road_centerline(model, p: HbaParams):
  """(x, y, z_road) samples of our lane centre from near_m to far_m, z_road relative to model z (add height later)."""
  lanes, probs = model["lanes"], model["lane_probs"]
  if len(lanes) >= 3 and min(probs[1], probs[2]) > 0.3:
    l, r = lanes[1], lanes[2]
    pts = np.column_stack([l[:, 0], (l[:, 1] + r[:, 1]) / 2, (l[:, 2] + r[:, 2]) / 2])
  else:
    pts = model["position"]
  pts = pts[np.argsort(pts[:, 0])]
  pts = pts[pts[:, 0] > 1.0]
  if len(pts) < 3:
    pts = np.array([[5.0, 0.0, 0.0], [50.0, 0.0, 0.0]])
  xs = np.concatenate([np.geomspace(p.near_m, max(p.near_m + 1, pts[-1, 0]), 24),
                       np.geomspace(pts[-1, 0] + 5, p.far_m, 10)])
  ys = np.interp(xs, pts[:, 0], pts[:, 1])
  zs = np.interp(xs, pts[:, 0], pts[:, 2])
  # beyond the model horizon continue straight along the last heading
  tail = pts[-min(6, len(pts)):]
  slope_y = np.polyfit(tail[:, 0], tail[:, 1], 1)[0] if len(tail) > 1 else 0.0
  slope_z = np.polyfit(tail[:, 0], tail[:, 2], 1)[0] if len(tail) > 1 else 0.0
  beyond = xs > pts[-1, 0]
  ys[beyond] = pts[-1, 1] + slope_y * (xs[beyond] - pts[-1, 0])
  zs[beyond] = pts[-1, 2] + slope_z * (xs[beyond] - pts[-1, 0])
  return np.column_stack([xs, ys, zs])


def corridor_mask(geom: Geometry, center, p: HbaParams):
  mask = np.zeros((geom.h, geom.w), np.uint8)
  left = p.ego_half_width + (p.oncoming_width if p.oncoming_side < 0 else p.far_side_width)
  right = p.ego_half_width + (p.oncoming_width if p.oncoming_side > 0 else p.far_side_width)
  corners = []
  for x, y, z in center:
    zr = z + geom.height
    corners.append([(x, y - left, zr - p.lamp_low), (x, y + right, zr - p.lamp_low),
                    (x, y - left, zr - p.lamp_high), (x, y + right, zr - p.lamp_high)])
  corners = np.array(corners)  # N x 4 x 3
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
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * p.margin_px + 1, 2 * p.margin_px + 1)))
  return mask


def find_blobs(bgr, mask, p: HbaParams):
  # only the corridor's bounding box is converted and labelled; it is a band of ~100-200 rows, not the whole frame
  ox, oy, cw, ch = cv2.boundingRect(mask)
  if cw == 0 or ch == 0:
    return []
  blobs = find_blobs_in(bgr[oy:oy + ch, ox:ox + cw], mask[oy:oy + ch, ox:ox + cw], p)
  for b in blobs:
    b.u += ox
    b.v += oy
    b.bbox = (b.bbox[0] + ox, b.bbox[1] + oy, b.bbox[2], b.bbox[3])
  return blobs


def find_blobs_in(bgr, mask, p: HbaParams):
  y = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
  hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
  hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
  inside = mask > 0
  white = ((y >= p.white_y) & inside).astype(np.uint8)
  # taillights: saturated red; yellow signs, amber reflectors and warm windows have hue ~12..35
  red = (((hue <= p.red_hue) | (hue >= 180 - p.red_hue)) & (sat >= p.red_sat) & (val >= p.red_v) & inside).astype(np.uint8)
  blobs = []
  for m, is_red in ((white, False), (red, True)):
    n, _, stats, cents = cv2.connectedComponentsWithStats(m, connectivity=8)
    for i in range(1, n):
      area = int(stats[i, cv2.CC_STAT_AREA])
      if not (p.min_area <= area <= p.max_area):
        continue
      x0, y0, w, h = (int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
      peak = int(y[y0:y0 + h, x0:x0 + w].max())
      if is_red:
        # look one blob-size around: a taillight's red halo surrounds a near-white core
        pad = max(w, h)
        if int(val[max(0, y0 - pad):y0 + h + pad, max(0, x0 - pad):x0 + w + pad].max()) < p.red_peak_v:
          continue
      blobs.append(Blob(float(cents[i, 0]), float(cents[i, 1]), area, peak, is_red, (x0, y0, w, h)))

  # a taillight's saturated core also passes the white test; keep only the red blob around it
  def inside_red(b):
    for r in blobs:
      if r.red:
        x0, y0, w, h = r.bbox
        pad = max(w, h)
        if x0 - pad <= b.u <= x0 + w + pad and y0 - pad <= b.v <= y0 + h + pad:
          return True
    return False
  return [b for b in blobs if b.red or not inside_red(b)]


class HbaClassifier:
  def __init__(self, p: HbaParams | None = None):
    self.p = p or HbaParams()
    self.tracks: list[Track] = []
    self._next_id = 0
    self._threat_frames = 0
    self._clear_since: float | None = None
    self._speed_ok = False
    self._permitted = False
    self.last_mask = None
    self.last_blobs: list[Blob] = []

  def _associate(self, t, blobs, geom, center):
    p = self.p
    unmatched = list(range(len(blobs)))
    for tr in self.tracks:
      _, pu, pv, *_ = tr.hist[-1]
      best, best_d = None, p.gate_px
      for j in unmatched:
        b = blobs[j]
        d = np.hypot(b.u - pu, b.v - pv)
        if d < best_d:
          best, best_d = j, d
      if best is None:
        tr.missed += 1
        continue
      unmatched.remove(best)
      tr.missed = 0
      tr.age += 1
      tr.hist.append(self._obs(t, blobs[best], geom, center))
      tr.hist = tr.hist[-30:]
    for j in unmatched:
      tr = Track(self._next_id, [self._obs(t, blobs[j], geom, center)], age=1)
      self._next_id += 1
      self.tracks.append(tr)
    self.tracks = [tr for tr in self.tracks if tr.missed <= p.max_missed]

  def _obs(self, t, b: Blob, geom: Geometry, center):
    hit = geom.ray_to_plane(b.u, b.v, geom.height - self.p.lamp_height)
    if hit is None or hit[0] <= 0 or hit[0] > 2000:
      dist = lateral = angle = float("nan")  # at or above the horizon: far away, or not a lamp at all
    else:
      dist = hit[0]
      lateral = hit[1] - np.interp(dist, center[:, 0], center[:, 1])
      angle = np.degrees(np.arctan2(lateral, dist))
    return (t, b.u, b.v, dist, lateral, b.area, b.red, b.peak_y, angle)

  def _classify(self, tr: Track, v_ego):
    p = self.p
    h = np.array([x[:6] for x in tr.hist], dtype=float)
    tr.dist, tr.lateral = h[-1, 3], h[-1, 4]
    red = tr.red_frac > 0.5
    angles = np.array([x[8] for x in tr.hist[-5:]], dtype=float)
    big_white = not red and tr.hist[-1][5] >= p.strong_area  # real headlights pointing at the road (driveway exit)
    if not big_white and np.isfinite(angles).any() and p.oncoming_side * np.nanmedian(angles) < -p.roadside_angle_deg:
      tr.obj_speed = float("nan")
      tr.kind = KIND_STATIC  # roadside on our side: reflectors, parked cars, driveway markers
      return
    seg = h[-p.motion_frames:]
    if (len(h) >= p.motion_frames and v_ego >= p.min_motion_v_ego and np.isfinite(seg[:, 3]).all()
        and seg[:, 3].max() < p.motion_max_dist):
      inv = 1.0 / np.clip(seg[:, 3], 1.0, 2000.0)
      slope = np.polyfit(seg[:, 0] - seg[0, 0], inv, 1)[0]   # d(1/d)/dt = closing / d^2
      closing = slope * tr.dist ** 2
      tr.obj_speed = closing - v_ego                           # >0: coming at us, ~0: static, <0: moving away
      if abs(tr.obj_speed) < p.static_speed:
        tr.kind = KIND_STATIC      # moves with the road: reflector, sign, parked car, house light
      elif tr.obj_speed > p.oncoming_speed and not red:
        tr.kind = KIND_ONCOMING
      elif tr.obj_speed < 0 or red:
        tr.kind = KIND_PRECEDING
      else:
        tr.kind = KIND_CANDIDATE
    else:
      tr.obj_speed = float("nan")
      tr.kind = KIND_PRECEDING if red else KIND_CANDIDATE

  def update(self, t, bgr, model, rpy_calib, height, v_ego, dark=True, valid=True) -> HbaOut:
    p, out = self.p, HbaOut()
    if not valid or model is None or rpy_calib is None:
      self._reset()
      out.reason = REASON_FAULT
      return out
    geom = Geometry(rpy_calib, height)
    center = road_centerline(model, p)
    mask = corridor_mask(geom, center, p)
    blobs = find_blobs(bgr, mask, p)
    self.last_mask, self.last_blobs, self.last_center, self.last_geom = mask, blobs, center, geom
    self._associate(t, blobs, geom, center)
    for tr in self.tracks:
      if tr.missed == 0:
        self._classify(tr, v_ego)

    live = [tr for tr in self.tracks if tr.missed == 0 and tr.is_threat(p)]
    out.n_oncoming = sum(tr.kind == KIND_ONCOMING for tr in live)
    out.n_preceding = sum(tr.kind == KIND_PRECEDING for tr in live)
    out.n_candidates = sum(tr.kind == KIND_CANDIDATE for tr in live)
    lead = max(((pr, x) for pr, x, _ in model["leads"]), default=(0.0, 0.0))
    out.lead_prob = lead[0]
    model_lead = lead[0] > p.lead_prob and lead[1] < p.lead_max_x

    reason = 0
    if out.n_oncoming or out.n_candidates:
      reason |= REASON_ONCOMING
    if out.n_preceding or model_lead:
      reason |= REASON_PRECEDING
    out.threat = reason != 0
    self._threat_frames = self._threat_frames + 1 if out.threat else 0

    if v_ego > p.enable_v:
      self._speed_ok = True
    elif v_ego < p.disable_v:
      self._speed_ok = False
    if not self._speed_ok:
      reason |= REASON_SPEED
    if not dark:
      reason |= REASON_TOO_BRIGHT

    if self._threat_frames >= p.dip_frames or not self._speed_ok or not dark:
      self._permitted = False
      self._clear_since = None
    elif not out.threat:
      if self._clear_since is None:
        self._clear_since = t
      out.clear_for_s = t - self._clear_since
      if out.clear_for_s >= p.clear_s:
        self._permitted = True
      elif not self._permitted:
        reason |= REASON_DELAY
    out.high_beam_permitted = self._permitted
    out.reason = 0 if self._permitted else reason
    out.valid = True
    return out

  def _reset(self):
    self.tracks = []
    self._threat_frames = 0
    self._clear_since = None
    self._permitted = False
