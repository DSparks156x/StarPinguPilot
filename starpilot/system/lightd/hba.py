"""High beam assist, prototype. StarPinguPilot.

Answers one question per narrow-camera frame: may the high beam be on? Everything uncertain means no.

Pipeline:
  1. corridor: project the model's road (lane lines / path, 0..~600 m) into the image, from road level up to
     lamp height, wide enough for our lane and the oncoming lane. Past the model's horizon it widens with distance,
     because the road there is a guess. Taillights get one more lane on the far side when the model sees one.
     Lights outside it (streetlights, windows) are ignored.
  2. blobs: bright white (headlights) and red (taillights) connected components inside the corridor. Lit yellow
     and green signs are rejected by colour.
  3. tracks: nearest-neighbour association, predicted through the gyro's rotation between frames. Distance comes
     from a lamp-height assumption on the model's road profile, with the camera pitch corrected by the gyro; the
     object's own speed comes from how fast inverse depth changes vs our speed. Roadside reflectors and other
     static lights move like the road (speed ~0), oncoming lamps approach faster than we drive.
  4. our own high beam: retroreflectors (delineators, signs) brighten when it switches on and dim when it switches
     off; lamps don't care. Tracks that follow the switch, and lone lights born within a second of it switching on,
     are reflectors until they show what a sign can't: a lamp pair or oncoming motion. The high beam lights a far
     sign 5-10x brighter, so without this the restore itself creates the next "strong lamp" (route 5a flashed at
     signs every 2.6 s).
  5. decision (HbaPolicy): each track builds up evidence at a rate set by what it shows (after Gentex SmartBeam's
     per-light index): a model lead, a wide-camera lamp, measured oncoming motion or a bright near lamp dip on their
     first frame, a far bright light in two frames, but under our own high beam (which lights far signs to the same
     look) in 0.4 s, a far lone dot only after 2 s. Lost tracks hand their evidence to a re-detection nearby.
     Restoring needs a clear corridor for clear_s, which backs off when restores keep getting cut short by anything a
     sign could be (not a model lead, a wide-camera lamp or measured oncoming motion). Reasons use the
     FL1_Abblendursache bits from the PQ35 K-matrix (mFLA_1, 0x65A).
  6. town: streetlights and lit windows (bright, unsaturated, high above the road, outside the corridor) averaged over
     a few seconds with hysteresis, or a posted limit <= 50 km/h, block the high beam like the speed gate.
All timing is in seconds, so the result doesn't depend on the rate the worker manages.
"""
import copy
import math
from dataclasses import dataclass, field, replace
from typing import NamedTuple

import cv2
import numpy as np

# comma four (mici, OS04C10) narrow camera, from common/transformations/camera.py. Kept local so this module
# doesn't pull in the compiled transformations extension and runs anywhere for offline replay.
FCAM_W, FCAM_H, FCAM_F = 1344, 760, 1522.0 * 3 / 4
VIEW_FROM_DEVICE = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])
FCAM_K = np.array([[FCAM_F, 0.0, FCAM_W / 2], [0.0, FCAM_F, FCAM_H / 2], [0.0, 0.0, 1.0]])
FCAM_K_INV = np.linalg.inv(FCAM_K)
# the wide (fisheye) camera, treated as a pinhole like openpilot does (_os_fisheye): ~115 degrees across
ECAM_W, ECAM_H, ECAM_F = 1344, 760, 567.0 * 3 / 4
ECAM_K = np.array([[ECAM_F, 0.0, ECAM_W / 2], [0.0, ECAM_F, ECAM_H / 2], [0.0, 0.0, 1.0]])
ECAM_K_INV = np.linalg.inv(ECAM_K)


def rot_from_euler(rpy):
  """Same convention as common/transformations/orientation.py: R = Rz(yaw) Ry(pitch) Rx(roll)."""
  r, p, y = rpy
  cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
  rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  return rz @ ry @ rx


def median(a):
  """np.median of a small array, without its overhead (~100 us a call on the A55). Same value: the middle element, or
  the mean of the two middle ones as float (added as floats, so uint8 doesn't wrap). nan if empty."""
  s = np.sort(a, axis=None)
  n = s.size
  if n == 0:
    return float("nan")
  h = n >> 1
  return float(s[h]) if n & 1 else (float(s[h - 1]) + float(s[h])) / 2

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
  adjacent_lane_width: float = 4.0  # extra far-side width for red lights when the model sees a lane there
  adjacent_lane_prob: float = 0.5
  lamp_low: float = 0.2             # lamp heights above the road
  lamp_high: float = 1.6
  margin_px: int = 10               # calibration / pitch slop
  beyond_lateral: float = 0.02      # past the model horizon the corridor widens by this many m per m, each side...
  beyond_vertical: float = 0.01     # ...and grows this many m per m up and down (crests, dips)
  lamp_height: float = 0.7          # for ground-plane distance
  # blobs
  white_y: int = 225
  white_max_sat: int = 110          # a lit yellow or green sign is bright but saturated; lamp cores are near white
  sign_hue: tuple = (10, 100)       # OpenCV hue range of those signs (yellow ~25, green ~60)
  red_hue: int = 8                  # OpenCV hue 0..180: red is <= red_hue or >= 180 - red_hue; amber/yellow is 12..35
  red_sat: int = 110
  red_v: int = 70
  red_peak_v: int = 190             # emissive: a taillight's core is bright; red paint lit by our beams is not
  min_area: int = 2
  max_area: int = 20000
  # tracking
  gate_px: float = 45.0
  max_missed_s: float = 0.3
  motion_s: float = 0.7             # window for the motion fit
  motion_min_obs: int = 5
  confirm_s: float = 0.4            # weak blobs must persist this long before they dip (roadside reflectors are
                                    # mostly classified static by then)
  strong_area: int = 25             # ...unless big or saturated: those dip at once
  strong_peak: int = 250
  strong_peak_area: int = 16        # small saturated dots are often delineators lit by our beams
  # far-light vehicle evidence: delineators and porch lights are single dots, vehicles show two lamps on one row
  pair_max_dx: int = 60             # px between partner lamps
  pair_dy_frac: float = 0.3         # allowed row offset as a fraction of the separation (+2 px)
  pair_area_ratio: float = 4.0
  post_pair_k: float = 5.0          # a tall blob pairs only this close (x its height): far lamps smeared by glare or
                                    # a wet road look like posts, two posts across the road are much farther apart
  wide_ratio: float = 1.8           # w/h of one blob that already is two merged lamps
  post_ratio: float = 1.5           # h/w of a white blob that is a delineator reflector
  pair_window_s: float = 0.5
  pair_min: int = 2
  big_white_max_lateral: float = 8.0  # a big white light this close to our lane centre may be a car pointing at the
                                    # road; farther out it's a sign or reflector lit by our beams
  band_slack: float = 0.5           # a light whose ray is never within lamp_low..lamp_high (+- this) of the road between
                                    # near_m and far_m is not a lamp: it only got in through the corridor margin
  sign_frac: float = 1.0            # a red blob with more lit yellow/orange pixels around it than its own area is a sign:
  sign_frac_white: float = 4.0      # a white one only when yellow dominates: a halogen headlight's halo is a yellow ring
                                    # of ~1.7x its core (route 46), a sign's overexposed middle sits in a solid yellow field
  sign_sat: int = 150               # the red edge of a chevron, or the overexposed white middle of a yellow sign
  crisp_min_area: int = 16          # lamps bloom, retroreflectors don't: a white blob at least this big whose
  crisp_ring_v: int = 32            # surroundings have a median V under this is a sign or reflector lit by our beams.
                                    # Route 46: signs and post reflectors 13-24, headlights 41-128. Small far blobs overlap.
  # lamps bloom, lit signs don't, measured one level lower: a lamp is a tiny source of huge radiance whose flare spreads
  # far past its saturated core; a sign lit by our high beam is a surface just over saturation, and its glow ends at the
  # sign's edge (its own bright face is what defeats crisp_ring_v). Route 6f: the area at Y >= glow_y around a
  # saturated core is 2.2-4.9x the core on lit signs (p10-p90, yellow diamonds and white squares), 7.8-93x on oncoming
  # headlights (6f, 5a, 46; < 5x on 0.3-0.5 % of their far frames, nearly all lamp pairs or motion-smeared).
  glow_y: int = 70
  glow_ratio: float = 6.0           # a core at least glow_min_core big whose glow is under this many times its area...
                                    # (lit signs hover at 4.6-5.0 for seconds as they come in, 6f seg 12; cars under our
                                    # beam start at 7.8; < 6 on 0.6-1.1 % of car frames, nearly all lamp pairs)
  glow_min_core: int = 30           # ...is compact; small far cores are too few pixels to tell
  glow_max_core: int = 600          # ...and a core bigger than this is a lamp right in front of us, not a far sign
  near_band_m: float = 30.0         # a light whose ray is only at lamp height closer than this...
  near_band_area: int = 80          # ...and is smaller than this is not a lamp: a car that close would be huge
  roadside_angle_deg: float = 3.3   # off-path angle on the non-oncoming side beyond which a light is roadside...
  roadside_bend_k: float = 2.2      # ...unless its ray is over our lanes at lamp height past this x its range: on a bend
                                    # toward our side the range (one row, one lamp height) puts a far car beside the road
                                    # (route 90 seg 21: a car round a right bend was "roadside" for 1.2 s at full beam,
                                    # ratio 2.3-5.7). A lamp at lamp_low instead of lamp_height alone is up to
                                    # (1.22 - 0.2) / (1.22 - 0.7) = 2x farther; shoulder reflectors on gentle bends: 1.1-2.1
                                    # (in-lane vehicles only get there inside ~40 m, where the model lead has them).
                                    # Angle = lateral / distance, so it survives the unknown lamp height.
  motion_max_dist: float = 70.0     # beyond this a lamp is a few px below the horizon and 1 px of row is tens of
                                    # metres, so motion can't classify it; far lights stay candidates (= dip)
  glare_side_deg: float = 1.0       # a strong white lamp no more than this far to our side of the path is a car, whatever
                                    # motion and votes say (they're noisy close up: lamps swap, wet roads reflect)
  static_vote_s: float = 1.0       # a track that looked static for at least half of this window stays static...
  static_vote_frac: float = 0.5     # ...unless it shows vehicle evidence (oncoming motion, a strong lamp pair)
  static_speed: float = 4.0         # |object speed| below this (m/s) after motion_s: static light
  oncoming_speed: float = 5.0
  min_motion_v_ego: float = 8.0     # below this, motion can't separate static lights from vehicles
  # gyro
  # our own high beam
  beam_before_s: float = 0.4        # flux before a switch...
  beam_after_s: tuple = (0.15, 0.6)  # ...vs after it (the bi-xenon flap and CAN delay)
  beam_ratio: float = 1.8           # a reflector brightens at least this much when the high beam comes on
  beam_born_s: float = 1.2          # an unpaired light that first shows up this soon after our high beam came on is the
                                    # sign it lit, whatever its size (the permit -> CAN -> shutter -> camera -> 10 Hz chain
                                    # is up to ~1 s; on route 5a the lit signs were born 0.3-1.0 s after GK1_Fernlicht)
  # decision
  lead_prob: float = 0.5
  lead_max_x: float = 400.0
  dip_s: float = 0.08               # threat on two frames in a row
  clear_s: float = 2.0
  clear_max_s: float = 8.0          # restore backoff: each restore cut short within short_permit_s doubles clear_s
  short_permit_s: float = 3.0
  backoff_reset_s: float = 10.0     # this long permitted resets the backoff
  pass_resets_backoff: bool = True  # ...and so does a vehicle going past: the cut-short restores were traffic, not signs
                                    # our beam lit (route 90 sat at 8 s through a run of cars)
  enable_v: float = 60 / 3.6
  disable_v: float = 50 / 3.6
  # evidence (after Gentex SmartBeam, see DESIGN.md section 12): instead of "any threat on two frames dips", each track
  # builds up evidence at a rate (per second, 1 = dip) set by what it shows, and the beam dips when one track's evidence
  # reaches 1. Weak, far, lone lights must persist; a lamp we are sure of dips on its first frame (rate > 1 / frame dt, so
  # 12 is one frame at 10 Hz and 6 two). Under our own high
  # beam a bright blob is the weakest evidence there is (the sign we just lit is bright too), so it builds slowly.
  evidence: bool = True
  ev_motion: float = 12.0           # measured oncoming motion
  ev_strong: float = 12.0           # big or saturated lamp inside motion_max_dist
  ev_strong_far: float = 6.0        # ...farther (or no range), our high beam off: two frames
  ev_strong_hb: float = 2.5         # ...farther, our high beam on: the signs it lights are far, big and saturated
  ev_near: float = 6.0              # weak light inside motion_max_dist
  ev_near_hb: float = 1.5           # ...white, with our high beam on: what it lights up there are delineators and post
                                    # reflectors (a car's lamps that close are bright), so give the motion fit (motion_s)
                                    # time. Not red (a far car's dim taillights get a range that short too) and not a lamp
                                    # pair (a far car's headlights, route 6f seg 21)
  ev_pair: float = 4.0              # weak far light with a lamp partner (a second blob on its row) over pair_window_s
  ev_wide: float = 1.0              # weak far light that is one wide blob: two lamps merged, or (route 6f, most of the
                                    # dips left under our high beam) a small white plate or reflector far ahead: 1 s
  ev_lone: float = 0.5              # weak far lone dot: 2 s (the rules never dipped for these; a far car is one)
  ev_static: float = -2.0           # classified static: evidence drains
  ev_missing: float = -3.0          # a lost track's evidence drains this fast; a re-detection near it inherits it...
  ev_ghost_s: float = 1.0           # ...for this long (far lamps flicker in and out at this exposure)
  ev_hold: float = 0.5              # a track at or above this keeps the corridor from counting as clear
  ev_gain_v: tuple = (64 / 3.6, 110 / 3.6)  # evidence builds faster at speed (less time to the pass)...
  ev_gain_max: float = 1.0          # ...up to this factor at ev_gain_v[1]
  # restore by cause (SmartBeam's fast return to bright, holds and minimum low-beam time): an oncoming car that passed
  # us is gone for good, a light that vanished far out (a bend, a crest) usually comes back
  restore_by_cause: bool = True
  clear_pass_s: float = 0.5         # clear time when every light we dipped for left by passing us
  clear_far_s: float = 2.0          # ...when one vanished while still far
  cause_window_s: float = 1.0       # what ended this long before the corridor went clear is what cleared it
  pass_area: int = 100              # a lost light this big, or inside motion_max_dist, on the oncoming half of the image
                                    # (and not red) went by us; the wide guard's hit right after a pass is the same car
  min_low_s: tuple = (3.0, 5.0)     # every dip stays low at least this long: the first at min_low_v[1] and up, the second
  min_low_v: tuple = (28 / 3.6, 70 / 3.6)  # at min_low_v[0] and below. SmartBeam uses 2 s; on 5a and 46, 3 s cut glare by a
                                    # third (restores between two cars) for ~5 s of nuisance per drive, 4 s cost 20 s more
  # town: streetlights and lit windows, i.e. bright unsaturated lights high above the road outside the corridor.
  # Above the row of a point town_height over the road town_ref_m out, a light is higher than that or nearer.
  town_height: float = 3.0
  town_ref_m: float = 150.0
  town_y: int = 180
  town_max_sat: int = 110           # lit yellow signs and red lamps are saturated...
  town_sign_hue: tuple = (38, 100)  # ...green guide signs less so, but they're green
  town_sign_sat: int = 35
  town_window_s: float = 4.0        # mean lights per frame over this window...
  town_on: float = 2.0              # ...enters town (c9: a mountain town peaks at 6.6, the open road at 1.0, one yard light)
  town_off: float = 0.5             # ...and leaves it after town_exit_s below this
  town_exit_s: float = 10.0
  town_limit: float = 50 / 3.6 + 0.5  # a posted limit at or under 50 km/h is town too
  # wide-camera glare guard: in a tight curve an oncoming car is outside the narrow view, or cut by its edge, until
  # it's close. The wide camera sees it first. Any big, bright, near-neutral light that blooms on the oncoming side of
  # the path and outside the narrow view (inside it the narrow pipeline knows better) dips at once.
  wide_near_m: float = 6.0
  wide_far_m: float = 300.0
  wide_width: float = 7.5           # m from our lane centre towards oncoming traffic; the path lags in tight curves
  wide_stride: int = 2              # the wide frame is looked at every other row and column; px below are at that scale
  wide_margin_px: int = 4
  wide_narrow_inset_px: int = 40    # narrow px: a light this far inside the narrow view is left to the narrow pipeline
                                    # (all of it: leaving only its corridor, DESIGN.md section 15, let in the far house
                                    # lights the narrow corridor rejects and added 60-80 s of dips per drive)
  wide_y: int = 225
  wide_min_area: int = 6
  wide_max_chroma: int = 40         # median |U - 128| + |V - 128|: a lit yellow sign is far from neutral
  wide_ring_y: int = 32             # median Y around it: lamps bloom, signs and reflectors lit by our beams don't
  wide_max_aspect: float = 2.5      # h / w: taller is a post reflector
  wide_min_reach_m: float = 10.0    # a light that can't be a lamp in the corridor farther out than this is beside us: a car
                                    # passing (our beam doesn't reach its driver), or a house light at the horizon 40 deg
                                    # off, where the corridor's near end reaches up to. Route 46: those 7-10 m, cars in
                                    # curves 12-18 m (the model's road stops short of where they are).
  wide_hold_s: float = 0.3          # a hit holds the threat this long: the wide frame is only looked at every WIDE_PERIOD_S


@dataclass
class Blob:
  u: float
  v: float
  area: int
  peak_y: int
  red: bool
  bbox: tuple
  flux: float = 0.0     # sum of Y above the dark background over the blob's box
  sat: int = 0          # median HSV saturation / hue of its pixels
  hue: int = 0
  paired: bool = False  # has a same-colour partner on its row, or is a wide blob (two lamps merged)
  partner: bool = False # the first of those only: a wide blob is also what a sign lit by our beam looks like
  glow: float = math.inf  # area at Y >= glow_y connected to the core / core area (inf: not measured or spreading wide)
  post: bool = False    # tall white rectangle without a partner: delineator reflector, not a lamp


class Obs(NamedTuple):
  t: float
  u: float
  v: float
  dist: float
  lateral: float
  area: int
  red: bool
  peak: int
  angle: float
  paired: bool
  post: bool
  flux: float
  sat: int
  high: bool    # its ray never passes through the lamp band over the road (or only right next to us, and the blob
                # is small): building, sign, streetlight
  partner: bool = False  # paired by a second blob on its row, not by being wide
  compact: bool = False  # a saturated core whose glow ends close around it: a lit sign (glow_ratio)
  bend: float = 0.0      # off to our side at its range: how much farther out its ray is still at lamp height over our
                         # two lanes (x its range). Past roadside_bend_k the road bends that way and it can be a car on it.
                         # Only looked at for would-be roadside lights, 0 otherwise.


@dataclass
class Track:
  tid: int
  hist: list = field(default_factory=list)  # Obs
  missed: int = 0
  kind: str = KIND_CANDIDATE
  obj_speed: float = float("nan")
  lateral: float = float("nan")
  dist: float = float("nan")
  first_t: float = 0.0
  reflector: bool = False       # followed our high beam switching: retroreflector
  why: str = ""                 # the rule that set kind, for replay and debugging
  votes: list = field(default_factory=list)  # (t, a static rule fired)
  pred: tuple | None = None     # (u, v) predicted for the next frame

  @property
  def age_s(self):
    return self.hist[-1].t - self.first_t

  @property
  def red_frac(self):
    return sum(h.red for h in self.hist) / len(self.hist)

  def strong(self, p):
    o = self.hist[-1]
    return o.area >= p.strong_area or (o.peak >= p.strong_peak and o.area >= p.strong_peak_area)

  def paired_recent(self, p):
    recent = [h for h in self.hist if h.t >= self.hist[-1].t - p.pair_window_s]
    return sum(h.paired for h in recent) >= p.pair_min

  def partner_recent(self, p):
    recent = [h for h in self.hist if h.t >= self.hist[-1].t - p.pair_window_s]
    return sum(h.partner for h in recent) >= p.pair_min

  def is_threat(self, p):
    if self.kind == KIND_STATIC:
      return False
    if self.strong(p):
      return True
    if self.age_s < p.confirm_s:
      return False
    # far lights (no usable motion) must look like a vehicle: a lamp pair, not a lone delineator or porch light
    far = not np.isfinite(self.dist) or self.dist > p.motion_max_dist
    return not far or self.paired_recent(p)

  def view(self, p):
    o = self.hist[-1]
    far = not np.isfinite(self.dist) or self.dist > p.motion_max_dist
    return TrackView(self.tid, o.u, o.v, self.kind, self.why, self.strong(p), self.is_threat(p), not far,
                     self.paired_recent(p), self.reflector, o.area, self.partner_recent(p))


class TrackView(NamedTuple):
  """What the decision policy needs of a live track; replay_hba's tracks csv carries the same, so the policy can be
  rerun offline (hba_tools/policy_sim.py) without decoding video."""
  tid: int
  u: float
  v: float
  kind: str
  why: str
  strong: bool
  threat: bool     # Track.is_threat: the rules' own yes/no
  near: bool       # inside motion_max_dist
  paired: bool     # a lamp pair on pair_min frames of pair_window_s
  reflector: bool
  area: float = 0.0
  partner: bool = False  # paired by a second blob on its row (not by being one wide blob) on pair_min frames


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
  clear_needed_s: float = 0.0
  town: bool = False
  town_lights: float = 0.0  # mean town lights per frame over town_window_s
  n_wide: int = 0           # lamps the wide-camera guard saw (its last look, while it holds)
  evidence: float = 0.0     # the highest track evidence (1 = dip)


class Geometry:
  """Projection between the calibrated model frame and narrow-camera pixels, or wide-camera pixels when given
  liveCalibration.wideFromDeviceEuler."""

  def __init__(self, rpy_calib, height, wide_from_device_euler=None):
    if wide_from_device_euler is None:
      self.w, self.h, self.K, self.K_inv = FCAM_W, FCAM_H, FCAM_K, FCAM_K_INV
      self.view_from_calib = VIEW_FROM_DEVICE @ rot_from_euler(rpy_calib)
    else:
      self.w, self.h, self.K, self.K_inv = ECAM_W, ECAM_H, ECAM_K, ECAM_K_INV
      self.view_from_calib = VIEW_FROM_DEVICE @ rot_from_euler(wide_from_device_euler) @ rot_from_euler(rpy_calib)
    self.calib_from_view = self.view_from_calib.T
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

  def ray_heights(self, u, v, center):
    """Height above the model's road profile (center: x, y, z rows) of the pixel's ray at x = 1 m and at each
    center x, plus the ray direction. None if the ray doesn't point forward."""
    d = self.calib_from_view @ (self.K_inv @ np.array([u, v, 1.0]))
    if d[0] <= 1e-3:
      return None
    # the road under the car is a camera height down; center starts at near_m
    xs = np.concatenate([[1.0], center[:, 0]])
    z = np.interp(xs, np.concatenate([[0.0], center[:, 0]]), np.concatenate([[self.height], center[:, 2]]))
    return xs, z - xs * d[2] / d[0], d

  def ray_to_road(self, u, v, center, above, heights=None):
    """Where the pixel's ray first passes `above` m over the road profile, from below or from above (an uphill road
    puts lamps above the camera horizon). Returns (x, y), or None if it never does out to the profile's end."""
    rh = heights or self.ray_heights(u, v, center)
    if rh is None:
      return None
    xs, h, d = rh
    f = h - above
    hit = np.nonzero(np.signbit(f[:-1]) != np.signbit(f[1:]))[0]
    if not len(hit):
      return None
    i = hit[0]
    x = xs[i] + (xs[i + 1] - xs[i]) * f[i] / (f[i] - f[i + 1])
    return x, x * d[1] / d[0]

  def strided(self, s):
    """The same projection onto every s-th row and column."""
    g = copy.copy(self)
    g.w, g.h = self.w // s, self.h // s
    g.K = np.diag([1 / s, 1 / s, 1.0]) @ self.K
    g.K_inv = np.linalg.inv(g.K)
    return g

  def horizon_row(self):
    uv, _ = self.project(np.array([[1000.0, 0.0, 0.0]]))
    return uv[0, 1]


def rotate_pixel(u, v, r_rel):
  """Where a very distant point at pixel (u, v) appears after the device rotated by r_rel (device frame)."""
  ray = VIEW_FROM_DEVICE @ (r_rel.T @ (VIEW_FROM_DEVICE.T @ (FCAM_K_INV @ np.array([u, v, 1.0]))))
  if ray[2] <= 1e-3:
    return u, v
  uv = FCAM_K @ (ray / ray[2])
  return uv[0], uv[1]


_GEOMSPACE: dict = {}


def _geomspace(start, stop, num):
  """np.geomspace, memoized: the lane lines' x samples are a fixed grid, so the same few calls repeat every frame.
  Keyed by type too: a float32 endpoint gives other values than the same float64 one."""
  key = (start, stop, num, type(start), type(stop))
  r = _GEOMSPACE.get(key)
  if r is None:
    if len(_GEOMSPACE) > 256:  # the plan path's x varies with speed
      _GEOMSPACE.clear()
    r = _GEOMSPACE[key] = np.geomspace(start, stop, num)
    r.flags.writeable = False
  return r


def _slopes(tail):
  """Least-squares slopes of y and z over x (tail: rows x, y, z), the closed form of np.polyfit(x, ., 1)[0]. Agrees
  with it to ~1e-13 relative; polyfit's lstsq was 0.5 ms a frame on the A55."""
  x = tail[:, 0].astype(np.float64)
  dx = x - x.mean()
  d = tail[:, 1:3].astype(np.float64)
  s = dx @ (d - d.mean(axis=0)) / (dx @ dx)
  return float(s[0]), float(s[1])


def road_centerline(model, p: HbaParams, height):
  """(x, y, z_road, beyond) samples of our lane centre from near_m to far_m. z_road is the road surface in the calib
  frame (down from the camera): lane lines are on the road already, the plan position is the device's own path, so it
  needs the camera height. beyond: metres past the model's horizon, 0 inside it."""
  lanes, probs = model["lanes"], model["lane_probs"]
  if len(lanes) >= 3 and min(probs[1], probs[2]) > 0.3:
    l, r = lanes[1], lanes[2]
    pts = np.column_stack([l[:, 0], (l[:, 1] + r[:, 1]) / 2, (l[:, 2] + r[:, 2]) / 2])
  else:
    pts = model["position"] + np.array([0.0, 0.0, height])
  pts = pts[np.argsort(pts[:, 0])]
  pts = pts[pts[:, 0] > 1.0]
  if len(pts) < 3:
    pts = np.array([[5.0, 0.0, height], [50.0, 0.0, height]])
  xs = np.concatenate([_geomspace(p.near_m, max(p.near_m + 1, pts[-1, 0]), 24),
                       _geomspace(pts[-1, 0] + 5, p.far_m, 10)])
  ys = np.interp(xs, pts[:, 0], pts[:, 1])
  zs = np.interp(xs, pts[:, 0], pts[:, 2])
  # beyond the model horizon continue straight along the last heading
  tail = pts[-min(6, len(pts)):]
  slope_y, slope_z = _slopes(tail) if len(tail) > 1 else (0.0, 0.0)
  beyond = np.maximum(xs - pts[-1, 0], 0.0)
  out = beyond > 0
  ys[out] = pts[-1, 1] + slope_y * beyond[out]
  zs[out] = pts[-1, 2] + slope_z * beyond[out]
  return np.column_stack([xs, ys, zs, beyond])


def adjacent_far_lane(model, p: HbaParams):
  """The model sees a lane line beyond ours on the far (non-oncoming) side: there's a lane for same-direction traffic."""
  probs = model["lane_probs"]
  i = 3 if p.oncoming_side < 0 else 0
  return len(probs) == 4 and probs[i] > p.adjacent_lane_prob


def corridor_mask(geom: Geometry, center, p: HbaParams, far_extra=0.0, lateral=None, with_box=False):
  """lateral: (left, right) extent in m from the lane centre, each positive towards its side; None: our lane plus the
  oncoming lane, and far_side_width + far_extra on the other side. with_box: returns (mask, box), box the bounding
  rectangle of the lit pixels as cv2.boundingRect gives it, found without scanning the whole frame."""
  mask = np.zeros((geom.h, geom.w), np.uint8)
  far_side = p.far_side_width + far_extra
  left = p.ego_half_width + (p.oncoming_width if p.oncoming_side < 0 else far_side)
  right = p.ego_half_width + (p.oncoming_width if p.oncoming_side > 0 else far_side)
  if lateral is not None:
    left, right = lateral
  # per cross-section: road-level left, right, then lamp-height left, right
  x, y, z, beyond = (center[:, k] for k in range(4))
  wl, wz = p.beyond_lateral * beyond, p.beyond_vertical * beyond
  yl, yr, zl, zh = y - left - wl, y + right + wl, z - p.lamp_low + wz, z - p.lamp_high - wz
  corners = np.empty((len(center), 4, 3))
  corners[:, :, 0] = x[:, None]
  corners[:, 0, 1], corners[:, 1, 1], corners[:, 2, 1], corners[:, 3, 1] = yl, yr, yl, yr
  corners[:, 0, 2], corners[:, 1, 2], corners[:, 2, 2], corners[:, 3, 2] = zl, zl, zh, zh
  uv, valid = geom.project(corners.reshape(-1, 3))
  uv, valid = uv.reshape(-1, 4, 2), valid.reshape(-1, 4).all(axis=1)
  big = np.abs(uv).max(axis=(1, 2))
  draw = valid[:-1] & valid[1:] & ~(np.maximum(big[:-1], big[1:]) > 1e5)  # each piece: section i to i + 1
  uv32 = uv.astype(np.float32).reshape(-1, 8)
  hulls = [cv2.convexHull(q).astype(np.int32) for q in np.concatenate([uv32[:-1], uv32[1:]], axis=1)[draw].reshape(-1, 8, 2)]
  for hull in hulls:
    cv2.fillConvexPoly(mask, hull, 255)
  if p.margin_px > 0 and hulls:
    # the margin: a convex piece grown by a disc is the piece plus its outline drawn 2m+1 thick with round joins.
    # Same as dilating with an elliptical kernel to a pixel or two, at a fraction of the cost (14 ms on the device).
    cv2.polylines(mask, hulls, True, 255, thickness=2 * p.margin_px + 1, lineType=cv2.LINE_8)
  if not with_box:
    return mask
  if not hulls:
    return mask, (0, 0, 0, 0)
  # everything drawn is within the hulls' extent plus the margin: scan only that (2 px slack for the thick lines)
  pts = np.concatenate(hulls).reshape(-1, 2)
  pad = max(p.margin_px, 0) + 2
  x0, y0 = max(int(pts[:, 0].min()) - pad, 0), max(int(pts[:, 1].min()) - pad, 0)
  x1, y1 = min(int(pts[:, 0].max()) + pad + 1, geom.w), min(int(pts[:, 1].max()) + pad + 1, geom.h)
  if x1 <= x0 or y1 <= y0:
    return mask, (0, 0, 0, 0)
  bx, by, bw, bh = cv2.boundingRect(mask[y0:y1, x0:x1])
  return mask, ((bx + x0, by + y0, bw, bh) if bw else (0, 0, 0, 0))


class Nv12Frame:
  """A camera frame left in NV12 (y: HxW, uv: H/2xW interleaved), converted to BGR only where it's looked at."""

  def __init__(self, y, uv):
    self.y, self.uv = y, uv
    self.shape = (*y.shape, 3)

  def crop_bgr(self, x, y, w, h):
    # chroma is shared per 2x2 block: convert an even-aligned box, then trim. Same pixels as converting it all.
    x0, y0 = x & ~1, y & ~1
    x1, y1 = min(self.shape[1], (x + w + 1) & ~1), min(self.shape[0], (y + h + 1) & ~1)
    nv12 = np.concatenate([self.y[y0:y1, x0:x1], self.uv[y0 // 2:y1 // 2, x0:x1]])
    bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
    return bgr[y - y0:y - y0 + h, x - x0:x - x0 + w]


def union_box(a, b):
  """The bounding rectangle of two cv2.boundingRect results (w == 0: empty)."""
  if not a[2]:
    return b
  if not b[2]:
    return a
  x0, y0 = min(a[0], b[0]), min(a[1], b[1])
  return x0, y0, max(a[0] + a[2], b[0] + b[2]) - x0, max(a[1] + a[3], b[1] + b[3]) - y0


_CAND_YMIN: dict = {}
_BLOCK_PX = ((0, 0), (0, 1), (1, 0), (1, 1))


def candidate_ymin(p: HbaParams):
  """For each NV12 chroma pair, indexed by the uv plane read as uint16 (U + 256 V), the lowest Y at which a pixel can
  pass the white or the red test (colour_masks), minus a little slack. Built by running this OpenCV's own conversion
  over every (Y, U, V) once (~0.7 s on the A55, on the first frame): its NV12 conversion gives each pixel from its own Y and its 2x2
  block's chroma pair, and the clipping makes a closed form for red loose (a dark pixel with a strong blue-red chroma
  clips blue to 0 and comes out red), which would cost more candidates than the table."""
  key = (p.white_y, p.red_v, p.red_sat, p.red_hue)
  if key not in _CAND_YMIN:
    uv = np.empty((256, 512), np.uint8)
    uv[:, 0::2] = np.arange(256, dtype=np.uint8)[None, :]  # block (row V, column U)
    uv[:, 1::2] = np.arange(256, dtype=np.uint8)[:, None]
    ymin = np.full((256, 256), 255, np.uint8)  # [V, U]; 255 also for never (a candidate only at Y 255, harmless)
    yp = np.empty((512, 512), np.uint8)
    for y0 in range(0, 256, 4):
      for k, (r, c) in enumerate(_BLOCK_PX):  # the four pixels of each block get Y y0..y0+3
        yp[r::2, c::2] = y0 + k
      bgr = cv2.cvtColor(np.concatenate([yp, uv]), cv2.COLOR_YUV2BGR_NV12)
      white, red = colour_masks(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), p)
      hit = cv2.bitwise_or(white, red)
      for k, (r, c) in enumerate(_BLOCK_PX):
        ymin[(hit[r::2, c::2] > 0) & (ymin == 255)] = y0 + k
    # slack: a level or two of Y, and the neighbouring chroma pairs, in case the conversion rounds differently elsewhere
    ymin = cv2.erode(ymin, np.ones((3, 3), np.uint8), borderType=cv2.BORDER_REPLICATE)
    _CAND_YMIN[key] = cv2.subtract(ymin, 2).ravel()
  return _CAND_YMIN[key]


_K2 = np.ones((2, 2), np.uint8)


def candidate_blocks(y, uv, mask, p: HbaParams):
  """y, mask: an even-aligned box of the luma plane and the corridor (0/255); uv: its chroma rows. One value per 2x2
  block, nonzero where a pixel of the block inside mask might pass the white or red test (a superset)."""
  ymax = cv2.dilate(cv2.min(y, mask), _K2, anchor=(0, 0))[::2, ::2]  # brightest pixel inside the mask, per block
  return cv2.compare(np.ascontiguousarray(ymax), candidate_ymin(p).take(uv.view(np.uint16)), cv2.CMP_GE)


def nv12_regions(frame, ox, oy, cw, ch, mask, p: HbaParams, max_regions=32):
  """Boxes (x, y, w, h) in the crop ox, oy, cw, ch of an Nv12Frame that hold every pixel passing the white or red test
  inside mask (the crop's corridor, both colours'), each with room around it for find_blobs_in's tests: those look
  at most max(w, h, 2) px around a blob. The boxes are disjoint. None: too many, convert the whole crop."""
  # candidates are found per 2x2 block and grouped on a grid of 8x8 px cells: labelling is slow in OpenCV (~1 ms for
  # the half-resolution box on the A55, mostly per pixel), the cells are 16x fewer
  x0, y0 = ox & ~7, oy & ~7
  x1, y1 = min(frame.shape[1], (ox + cw + 7) & ~7), min(frame.shape[0], (oy + ch + 7) & ~7)
  assert not (x1 | y1) & 7, "frame size must be a multiple of 8"
  m = np.zeros((y1 - y0, x1 - x0), np.uint8)
  m[oy - y0:oy - y0 + ch, ox - x0:ox - x0 + cw] = mask
  cand = candidate_blocks(frame.y[y0:y1, x0:x1], frame.uv[y0 // 2:y1 // 2, x0:x1], m, p)
  gw, gh = (x1 - x0) // 8, (y1 - y0) // 8
  cells = cv2.resize(cand, (gw, gh), interpolation=cv2.INTER_AREA)  # 4x4 blocks averaged: > 0 if any is a candidate
  n, _, stats, _ = cv2.connectedComponentsWithStats(cells, connectivity=8)
  if n == 1:
    return []
  if n - 1 > max_regions:
    return None
  # each group's cells grown by its own size (no blob in it is bigger), in crop pixels
  rects = []
  for a, b, c, d in stats[1:, :4].tolist():
    pad = max(c, d)
    rects.append([max(x0 + 8 * (a - pad) - ox, 0), max(y0 + 8 * (b - pad) - oy, 0),
                  min(x0 + 8 * (a + c + pad) - ox, cw), min(y0 + 8 * (b + d + pad) - oy, ch)])
  # merge overlapping ones until they're disjoint
  merged = True
  while merged:
    merged = False
    for i in range(len(rects)):
      for j in range(i + 1, len(rects)):
        a, b = rects[i], rects[j]
        if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
          rects[i] = [min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])]
          del rects[j]
          merged = True
          break
      if merged:
        break
  return [(a, b, c - a, d - b) for a, b, c, d in rects]


def find_blobs(frame, mask, p: HbaParams, red_mask=None, box=None):
  """frame: BGR array or Nv12Frame. red_mask: a wider corridor for taillights (None: the same one). box: the bounding
  rectangle of mask | red_mask's lit pixels, if the caller has it (corridor_mask(with_box=True))."""
  # only the corridor's bounding box is converted and labelled; it is a band of ~100-200 rows, not the whole frame
  if box is None:
    box = cv2.boundingRect(mask if red_mask is None else cv2.bitwise_or(mask, red_mask))
  ox, oy, cw, ch = box
  if cw == 0 or ch == 0:
    return []
  m = mask[oy:oy + ch, ox:ox + cw]
  rm = None if red_mask is None else red_mask[oy:oy + ch, ox:ox + cw]
  if isinstance(frame, Nv12Frame):
    # at night the box is nearly all black: convert only around the pixels that can pass a test. The rest stays 0,
    # which passes nothing, and no test looks there, so the blobs are the same as converting the whole box.
    regions = nv12_regions(frame, ox, oy, cw, ch, m if rm is None else cv2.bitwise_or(m, rm), p)
    if regions is None or sum(w * h for _, _, w, h in regions) > cw * ch // 2:
      regions = [(0, 0, cw, ch)]
    gray, hsv = np.zeros((ch, cw), np.uint8), np.zeros((ch, cw, 3), np.uint8)
    white, red = np.zeros((ch, cw), np.uint8), np.zeros((ch, cw), np.uint8)
    for x, y, w, h in regions:
      bgr = frame.crop_bgr(ox + x, oy + y, w, h)
      ys, xs = slice(y, y + h), slice(x, x + w)
      g, s = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
      gray[ys, xs], hsv[ys, xs] = g, s
      wr, rr = colour_masks(g, s, p)
      white[ys, xs] = cv2.bitwise_and(wr, m[ys, xs])
      red[ys, xs] = cv2.bitwise_and(rr, (m if rm is None else rm)[ys, xs])
    blobs = blobs_in(gray, hsv, white, red, p)
    lum = frame.y
  else:
    blobs = find_blobs_in(frame[oy:oy + ch, ox:ox + cw], m, p, rm)
    lum = None
  for b in blobs:
    b.u += ox
    b.v += oy
    b.bbox = (b.bbox[0] + ox, b.bbox[1] + oy, b.bbox[2], b.bbox[3])
    if not b.red and p.glow_min_core <= b.area <= p.glow_max_core:
      if lum is None:
        lum = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      b.glow = glow_ratio(lum, b, p)
  return blobs


def components(m):
  """connectedComponentsWithStats(m), labelled only inside the box around m's lit pixels (a night frame is mostly
  dark). The box is 2-px aligned so the labelling sees the same blocks; stats and centroids are in m's pixels."""
  x, y, w, h = cv2.boundingRect(m)
  if w == 0:
    return 1, np.zeros((1, 5), np.int32), np.zeros((1, 2))
  x0, y0 = x & ~1, y & ~1
  n, _, stats, cents = cv2.connectedComponentsWithStats(m[y0:y + h, x0:x + w], connectivity=8)
  stats[:, cv2.CC_STAT_LEFT] += x0
  stats[:, cv2.CC_STAT_TOP] += y0
  return n, stats, cents + (x0, y0)


def colour_masks(y, hsv, p: HbaParams):
  """Lamp-bright pixels (headlights) and saturated red ones (taillights), 0/255, from the gray and HSV images."""
  # cv2 thresholds instead of numpy boolean algebra: same masks, a fraction of the passes
  white = cv2.threshold(y, p.white_y - 1, 255, cv2.THRESH_BINARY)[1]
  # taillights: saturated red; yellow signs, amber reflectors and warm windows have hue ~12..35
  red = (cv2.inRange(hsv, (0, p.red_sat, p.red_v), (p.red_hue, 255, 255)) |
         cv2.inRange(hsv, (180 - p.red_hue, p.red_sat, p.red_v), (255, 255, 255)))
  return white, red


def find_blobs_in(bgr, mask, p: HbaParams, red_mask=None):
  y, hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
  white, red = colour_masks(y, hsv, p)
  return blobs_in(y, hsv, cv2.bitwise_and(white, mask), cv2.bitwise_and(red, mask if red_mask is None else red_mask), p)


def blobs_in(y, hsv, white, red, p: HbaParams):
  """find_blobs_in on the gray and HSV images, with colour_masks already cut to the corridors."""
  val = hsv[..., 2]
  blobs = []
  for m, is_red in ((white, False), (red, True)):
    n, stats, cents = components(m)
    cents = cents.tolist()
    for i, (x0, y0, w, h, area) in enumerate(stats.tolist()):  # LEFT, TOP, WIDTH, HEIGHT, AREA
      if i == 0 or not (p.min_area <= area <= p.max_area):
        continue
      box_y = y[y0:y0 + h, x0:x0 + w]
      peak = int(box_y.max())
      lit = m[y0:y0 + h, x0:x0 + w] > 0
      box_hsv = hsv[y0:y0 + h, x0:x0 + w][lit]
      sat, hue = int(median(box_hsv[:, 1])), int(median(box_hsv[:, 0]))
      if is_red:
        # look one blob-size around: a taillight's red halo surrounds a near-white core
        pad = max(w, h)
        if int(val[max(0, y0 - pad):y0 + h + pad, max(0, x0 - pad):x0 + w + pad].max()) < p.red_peak_v:
          continue
      elif sat > p.white_max_sat and p.sign_hue[0] <= hue <= p.sign_hue[1]:
        continue  # a yellow or green sign lit by our beams
      elif area >= p.crisp_min_area:
        pad = max(w, h, 2)
        ys, xs = slice(max(0, y0 - pad), y0 + h + pad), slice(max(0, x0 - pad), x0 + w + pad)
        ring = val[ys, xs][m[ys, xs] == 0]
        if len(ring) and median(ring) < p.crisp_ring_v:
          continue  # crisp edges against the dark: a sign or reflector, not a lamp
      # lots of saturated yellow right next to it: the red edge of a chevron, the overexposed middle of a yellow sign
      pad = max(w, h, 2)
      around = hsv[max(0, y0 - pad):y0 + h + pad, max(0, x0 - pad):x0 + w + pad]
      if cv2.countNonZero(cv2.inRange(around, (p.red_hue + 3, p.sign_sat, 150), (35, 255, 255))) > \
         (p.sign_frac if is_red else p.sign_frac_white) * area:
        continue
      win = y[max(0, y0 - 1):y0 + h + 1, max(0, x0 - 1):x0 + w + 1]
      flux = float(int(np.maximum(win, 16).sum()) - 16 * win.size)  # sum of max(Y - 16, 0)
      blobs.append(Blob(cents[i][0], cents[i][1], area, peak, is_red, (x0, y0, w, h), flux, sat, hue))

  # a taillight's saturated core also passes the white test; keep only the red blob around it
  def inside_red(b):
    for r in blobs:
      if r.red:
        x0, y0, w, h = r.bbox
        pad = max(w, h)
        if x0 - pad <= b.u <= x0 + w + pad and y0 - pad <= b.v <= y0 + h + pad:
          return True
    return False
  blobs = [b for b in blobs if b.red or not inside_red(b)]

  for a in blobs:
    a.post = not a.red and a.bbox[3] >= 3 and a.bbox[3] >= p.post_ratio * a.bbox[2]
  for i, a in enumerate(blobs):
    w, h = a.bbox[2], a.bbox[3]
    partners = 0
    for j, b in enumerate(blobs):
      if i == j or a.red != b.red:
        continue
      dx, dy = abs(a.u - b.u), abs(a.v - b.v)
      max_dx = p.pair_max_dx
      if a.post or b.post:
        max_dx = min(max_dx, p.post_pair_k * max(a.bbox[3], b.bbox[3]))
      if 2 <= dx <= max_dx and dy <= 2 + p.pair_dy_frac * dx and max(a.area, b.area) <= p.pair_area_ratio * min(a.area, b.area):
        partners += 1
    # exactly one partner: a lamp pair. Two or more: a row of house/street lights or a delineator chain.
    a.partner = partners == 1
    a.paired = a.partner or (partners == 0 and not a.post and w >= 3 and w >= p.wide_ratio * h)
  for a in blobs:
    a.post = a.post and not a.paired
  return blobs


def glow_ratio(lum, b: Blob, p: HbaParams):
  """Area of the lum >= glow_y region connected to the blob's saturated core, over the blob's area; inf if that region
  reaches the edge of a window four blob-sizes out (or the image's), i.e. spreads like a lamp's flare. lum: the whole
  frame's luma (the NV12 Y plane), never a crop: a flare cut off at a crop's edge would look compact."""
  x0, y0, w, h = b.bbox
  pad = 4 * max(w, h, 3)
  H, W = lum.shape
  ya, yb, xa, xb = y0 - pad, y0 + h + pad, x0 - pad, x0 + w + pad
  if ya < 0 or xa < 0 or yb > H or xb > W:
    return math.inf
  win = lum[ya:yb, xa:xb]
  glow = cv2.threshold(win, p.glow_y - 1, 1, cv2.THRESH_BINARY)[1]
  _, lab = cv2.connectedComponents(glow, connectivity=8)
  core = np.zeros(win.shape, bool)
  core[pad:pad + h, pad:pad + w] = win[pad:pad + h, pad:pad + w] >= p.white_y
  ids = np.unique(lab[core & (lab > 0)])
  if not len(ids):
    return math.inf
  region = np.isin(lab, ids)
  if region[0, :].any() or region[-1, :].any() or region[:, 0].any() or region[:, -1].any():
    return math.inf
  return float(region.sum()) / b.area


def town_lights(frame, geom: Geometry, center, mask, p: HbaParams, box=None):
  """Streetlights and lit windows in this frame: bright, unsaturated lights outside the corridor and above the row of
  a point town_height over the road town_ref_m out. A vehicle's lamps are in the corridor or below that row; lit signs
  that get above it (near ones) are yellow or green. box: the bounding rectangle of mask's lit pixels, if known."""
  x = p.town_ref_m
  pt = (x, np.interp(x, center[:, 0], center[:, 1]), np.interp(x, center[:, 0], center[:, 2]) - p.town_height)
  uv, ok = geom.project(np.array([pt]))
  rows = int(min(uv[0, 1], geom.h)) & ~1 if ok[0] else 0
  if rows <= 0:
    return 0
  y = frame.y[:rows] if isinstance(frame, Nv12Frame) else cv2.cvtColor(frame[:rows], cv2.COLOR_BGR2GRAY)
  # lights are spread over the whole band, so labelling it is the cost: label a 2x2 max-pooled copy (a light's peak
  # survives it), then measure each light at full resolution. Pooling the thresholded band by area averaging is that
  # max-pool: a block with any lit pixel averages to >= 64, an unlit one to 0.
  w2 = y.shape[1] & ~1
  band = cv2.threshold(y[:, :w2], p.town_y - 1, 255, cv2.THRESH_BINARY)[1]
  small = cv2.resize(band, (w2 // 2, rows // 2), interpolation=cv2.INTER_AREA)
  if box is None:
    lit = cv2.subtract(small, mask[:rows:2, :w2:2])
  else:
    # the mask is 0 outside its box: subtract only there (small[r, c] is the block at mask[2r, 2c])
    lit = small
    r0, c0 = (box[1] + 1) // 2, (box[0] + 1) // 2
    r1, c1 = min((box[1] + box[3] + 1) // 2, rows // 2), min((box[0] + box[2] + 1) // 2, w2 // 2)
    if box[2] and r1 > r0 and c1 > c0:
      lit[r0:r1, c0:c1] = cv2.subtract(lit[r0:r1, c0:c1], mask[2 * r0:2 * r1:2, 2 * c0:2 * c1:2])
  n, stats, _ = components(lit)
  count = 0
  for i in range(1, n):
    x0, y0, w, h = (2 * int(stats[i, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
    on = y[y0:y0 + h, x0:x0 + w] >= p.town_y
    if np.count_nonzero(on) < p.min_area:
      continue
    bgr = frame.crop_bgr(x0, y0, w, h) if isinstance(frame, Nv12Frame) else frame[y0:y0 + h, x0:x0 + w]
    px = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[on]
    sat, hue = int(median(px[:, 1])), int(median(px[:, 0]))
    if sat > p.town_max_sat or (p.town_sign_hue[0] <= hue <= p.town_sign_hue[1] and sat > p.town_sign_sat):
      continue
    count += 1
  return count


class WideLight(NamedTuple):
  u: float        # full-resolution wide-camera pixels
  v: float
  area: int       # px at wide_stride
  peak: int
  chroma: int
  ring: int
  reach: float    # farthest distance (m) at which its ray is at lamp height in the corridor, nan if nowhere
  lamp: bool      # passes every test: dips


def wide_params(p: HbaParams):
  """The corridor params for the wide guard: our side of the path out to wide_width on the oncoming side."""
  return replace(p, near_m=p.wide_near_m, far_m=p.wide_far_m, margin_px=p.wide_margin_px)


def lamp_reach(geom: Geometry, u, v, center, lateral, p: HbaParams):
  """Farthest distance (m) at which the pixel's ray is lamp_low..lamp_high (+- band_slack) over the road and within
  lateral = (left, right) m of the lane centre (+- 1 m, widening past the model's horizon like the corridor). nan if it
  never is. The image mask can't tell: close by, the corridor's lamp band reaches up to the horizon row, where a house
  light 200 m off to the side sits too."""
  d = geom.calib_from_view @ (geom.K_inv @ np.array([u, v, 1.0]))
  if d[0] <= 1e-3:
    return float("nan")
  xs = _geomspace(center[0, 0], center[-1, 0], 200)
  cy, cz, beyond = (np.interp(xs, center[:, 0], center[:, k]) for k in (1, 2, 3))
  above = cz - xs * d[2] / d[0]  # the ray's height over the road
  lat = xs * d[1] / d[0] - cy
  wl, wz = p.beyond_lateral * beyond + 1.0, p.beyond_vertical * beyond + p.band_slack
  ok = ((above >= p.lamp_low - wz) & (above <= p.lamp_high + wz) & (lat >= -lateral[0] - wl) & (lat <= lateral[1] + wl))
  return float(xs[ok].max()) if ok.any() else float("nan")


def wide_lights(frame: Nv12Frame, geom: Geometry, center, p: HbaParams, narrow: Geometry | None = None):
  """Bright lights in the wide frame on the oncoming side of the path, outside the narrow view (narrow: its geometry),
  looked at every wide_stride rows and columns. center: road_centerline() with wide_params(). Returns [WideLight]."""
  s = p.wide_stride
  g = geom.strided(s)
  lateral = (p.wide_width, 0.0) if p.oncoming_side < 0 else (0.0, p.wide_width)
  mask = corridor_mask(g, center, p, lateral=lateral)
  if narrow is not None:
    m = p.wide_narrow_inset_px
    corners = np.array([[m, m, 1.0], [narrow.w - m, m, 1.0], [narrow.w - m, narrow.h - m, 1.0], [m, narrow.h - m, 1.0]])
    uv, ok = g.project((narrow.calib_from_view @ (narrow.K_inv @ corners.T)).T)
    if ok.all():
      cv2.fillConvexPoly(mask, uv.astype(np.int32), 0)
  x, y, w, h = cv2.boundingRect(mask)
  if w == 0:
    return []
  yy = frame.y[s * y:s * (y + h):s, s * x:s * (x + w):s]
  lit = cv2.bitwise_and(cv2.threshold(yy, p.wide_y - 1, 255, cv2.THRESH_BINARY)[1], mask[y:y + h, x:x + w])
  n, labels, stats, cents = cv2.connectedComponentsWithStats(lit, connectivity=8)
  out = []
  for i in range(1, n):
    x0, y0, bw, bh, area = (int(v) for v in stats[i])
    if area < 2:
      continue
    on = labels[y0:y0 + bh, x0:x0 + bw] == i
    # NV12 chroma: one U, V pair per 2x2 block of full-resolution pixels
    rr, cc = np.nonzero(on)
    row, col = s * (y + y0 + rr) // 2, s * (x + x0 + cc) // 2 * 2
    chroma = int(median(np.abs(frame.uv[row, col].astype(np.int16) - 128) + np.abs(frame.uv[row, col + 1].astype(np.int16) - 128)))
    pad = max(bw, bh, 2)
    ys, xs = slice(max(0, y0 - pad), y0 + bh + pad), slice(max(0, x0 - pad), x0 + bw + pad)
    ring = yy[ys, xs][labels[ys, xs] != i]
    ring_y = int(median(ring)) if len(ring) else 0
    peak = int(yy[y0:y0 + bh, x0:x0 + bw][on].max())
    u, v = s * (x + cents[i, 0]), s * (y + cents[i, 1])
    reach = lamp_reach(geom, u, v, center, lateral, p)
    lamp = (area >= p.wide_min_area and chroma <= p.wide_max_chroma and ring_y >= p.wide_ring_y
            and bh <= p.wide_max_aspect * bw and reach >= p.wide_min_reach_m)
    out.append(WideLight(u, v, area, peak, chroma, ring_y, reach, lamp))
  return out


class HbaPolicy:
  """The decision: from live tracks (TrackView), a model lead, the wide guard, speed, darkness, town and our own beam
  to permitted + reason. Kept apart from the pixels so hba_tools/policy_sim.py can rerun it on replay csvs."""

  def __init__(self, p: HbaParams):
    self.p = p
    self.reset()

  def reset(self):
    self.threat_since: float | None = None
    self.clear_since: float | None = None
    self.speed_ok = False
    self.permitted = False
    self.permit_since: float | None = None
    self.clear_needed = self.p.clear_s
    self.ev: dict[int, tuple[float, float, float]] = {}  # tid -> (evidence, u, v) of tracks live last frame
    self.ghosts: list[tuple[float, float, float, float]] = []  # (t lost, evidence, u, v)
    self.last_t: float | None = None
    self.live_last: dict[int, TrackView] = {}  # tracks that held the corridor last frame
    self.causes: dict[str, float] = {}  # pass / far / wide / other -> when a threat last ended that way (or was there)
    self.dip_t: float | None = None

  def _ended(self, tv: TrackView, lost: bool):
    """How a track that held the corridor stopped holding it."""
    p = self.p
    if not lost:
      return "other"  # still there, but static now or its evidence drained: a sign, a reflector, a flicker
    oncoming_half = p.oncoming_side * (tv.u - FCAM_W / 2) > 0
    if oncoming_half and tv.kind != KIND_PRECEDING and (tv.near or tv.area >= p.pass_area):
      return "pass"
    return "other" if tv.near else "far"

  def _clear_needed(self, v_ego):
    p = self.p
    if not p.restore_by_cause:
      return self.clear_needed
    # only what ended just before the corridor went clear says what cleared it
    c = {k for k, t in self.causes.items() if t >= self.clear_since - p.cause_window_s}
    if "pass" in c and c <= {"pass", "wide"} and (p.pass_resets_backoff or self.clear_needed <= p.clear_s):
      self.clear_needed = p.clear_s
      return p.clear_pass_s
    if self.clear_needed > p.clear_s:
      return self.clear_needed  # backing off after early re-dips: that wins
    if c == {"far"} or c == {"far", "pass"}:
      return p.clear_far_s
    return p.clear_s

  def _min_low(self, v_ego):
    p = self.p
    lo, hi = p.min_low_v
    f = min(max((v_ego - lo) / max(hi - lo, 1e-3), 0.0), 1.0)
    return p.min_low_s[1] + (p.min_low_s[0] - p.min_low_s[1]) * f

  def _rate(self, tv: TrackView, high_beam_on):
    p = self.p
    if tv.kind == KIND_STATIC:
      return p.ev_static
    if tv.kind == KIND_ONCOMING and tv.why == "motion":
      return p.ev_motion
    if tv.strong:
      if tv.near:
        return p.ev_strong
      return p.ev_strong_hb if high_beam_on else p.ev_strong_far
    if tv.near:
      return p.ev_near_hb if high_beam_on and tv.kind != KIND_PRECEDING and not tv.paired else p.ev_near
    if tv.partner:
      return p.ev_pair
    return p.ev_wide if tv.paired else p.ev_lone

  def _evidence(self, t, views, v_ego, high_beam_on):
    """Integrates each track's evidence over dt; returns {tid: evidence}."""
    p = self.p
    dt = 0.1 if self.last_t is None else min(max(t - self.last_t, 0.0), 0.3)
    self.last_t = t
    lo, hi = p.ev_gain_v
    gain = 1.0 + (p.ev_gain_max - 1.0) * min(max((v_ego - lo) / max(hi - lo, 1e-3), 0.0), 1.0)
    seen = {tv.tid for tv in views}
    self.ghosts = [g for g in self.ghosts if t - g[0] <= p.ev_ghost_s]
    self.ghosts += [(t, e, u, v) for tid, (e, u, v) in self.ev.items() if tid not in seen and e > 0.0]
    ev = {}
    for tv in views:
      if tv.tid in self.ev:
        e = self.ev[tv.tid][0]
      else:
        # a re-detection of a light that flickered out inherits what it had built, less what drained meanwhile
        near = [g for g in self.ghosts if math.hypot(g[2] - tv.u, g[3] - tv.v) < p.gate_px]
        e = max((g[1] + p.ev_missing * (t - g[0]) for g in near), default=0.0)
        self.ghosts = [g for g in self.ghosts if g not in near]
      r = self._rate(tv, high_beam_on)
      e = min(max(e + (r * gain if r > 0 else r) * dt, 0.0), 1.0)
      if dt == 0.0 and r * gain * 0.1 >= 1.0:
        e = 1.0  # first frame after a reset: what dips on its first frame still does
      ev[tv.tid] = e
    self.ev = {tv.tid: (ev[tv.tid], tv.u, tv.v) for tv in views}
    return ev

  def step(self, t, out: HbaOut, views, model_lead, wide_threat, v_ego, dark, high_beam_on):
    """Fills out.{n_*, threat, reason, high_beam_permitted, clear_for_s, clear_needed_s, evidence}; out.town is an input."""
    p = self.p
    if p.evidence:
      ev = self._evidence(t, views, v_ego, high_beam_on)
      live = [tv for tv in views if ev[tv.tid] >= p.ev_hold]
      out.evidence = max(ev.values(), default=0.0)
      dipping = [tv for tv in views if ev[tv.tid] >= 1.0]
      confident = model_lead or wide_threat or bool(dipping)
      sure = model_lead or wide_threat or any(tv.kind == KIND_ONCOMING and tv.why == "motion" for tv in dipping)
    else:
      live = [tv for tv in views if tv.threat]
      # a threat we're sure of (model lead, a big lamp, oncoming motion) dips on its first frame...
      confident = model_lead or wide_threat or any(tv.strong or tv.kind == KIND_ONCOMING for tv in live)
      # ...but only what a sign can't be (a model lead, a wide-camera lamp, measured oncoming motion) says anything
      # about the last restore: a strong lone lamp right after a restore is most often the sign our own beam just lit
      # (route 5a flashed at signs every 2.6 s), so it backs off like a weak one
      sure = model_lead or wide_threat or any(tv.kind == KIND_ONCOMING and tv.why == "motion" for tv in live)
    now = {tv.tid for tv in views}
    held = {tv.tid for tv in live}
    for tid, tv in self.live_last.items():
      if tid not in held:
        self.causes[self._ended(tv, tid not in now)] = t
    self.live_last = {tv.tid: tv for tv in live}
    out.n_oncoming = sum(tv.kind == KIND_ONCOMING for tv in live)
    out.n_preceding = sum(tv.kind == KIND_PRECEDING for tv in live)
    out.n_candidates = sum(tv.kind == KIND_CANDIDATE for tv in live)

    reason = 0
    if out.n_oncoming or out.n_candidates or wide_threat:
      reason |= REASON_ONCOMING
    if out.n_preceding or model_lead:
      reason |= REASON_PRECEDING
    out.threat = reason != 0
    if not out.threat:
      self.threat_since = None
    elif self.threat_since is None:
      self.threat_since = t

    if v_ego > p.enable_v:
      self.speed_ok = True
    elif v_ego < p.disable_v:
      self.speed_ok = False
    if not self.speed_ok:
      reason |= REASON_SPEED
    if not dark:
      reason |= REASON_TOO_BRIGHT
    if out.town:
      reason |= REASON_TOWN

    if p.evidence:
      dip = confident
    else:
      # a threat we're sure of dips on its first frame; weak ones need dip_s (two frames) so one-frame flickers don't
      dip = self.threat_since is not None and (confident or t - self.threat_since >= p.dip_s)
    gated = not self.speed_ok or not dark or out.town
    if dip or gated:
      if self.permitted or self.dip_t is None:
        self.dip_t = t
        self.causes = {}  # a new low-beam episode: only how its own threats end counts
      if (self.permitted and dip and not sure and self.permit_since is not None
          and t - self.permit_since < p.short_permit_s):
        self.clear_needed = min(2 * self.clear_needed, p.clear_max_s)  # restored too early: wait longer next time
      self.permitted = False
      self.permit_since = None
      self.clear_since = None
    if gated or model_lead:
      self.causes["other"] = t
    if wide_threat:
      self.causes["wide"] = t
    if not (dip or gated) and not out.threat:
      if self.clear_since is None:
        self.clear_since = t
      out.clear_for_s = t - self.clear_since
      needed = self.clear_needed
      low_long_enough = True
      if p.restore_by_cause and p.evidence:
        needed = self._clear_needed(v_ego)
        low_long_enough = self.dip_t is None or t - self.dip_t >= self._min_low(v_ego)
      out.clear_needed_s = needed
      if out.clear_for_s >= needed and low_long_enough and not self.permitted:
        self.permitted = True
        self.permit_since = t
        self.causes = {}
      elif not self.permitted:
        reason |= REASON_DELAY
    if self.permitted and t - self.permit_since >= p.backoff_reset_s:
      self.clear_needed = p.clear_s
    if not out.clear_needed_s:
      out.clear_needed_s = self.clear_needed
    out.high_beam_permitted = self.permitted
    out.reason = 0 if self.permitted else reason
    out.valid = True
    return out


class HbaClassifier:
  def __init__(self, p: HbaParams | None = None):
    self.p = p or HbaParams()
    self.tracks: list[Track] = []
    self._next_id = 0
    self.policy = HbaPolicy(self.p)
    self._last_t: float | None = None
    self._beam: bool | None = None
    self._beam_switches: list[tuple[float, bool]] = []  # (t, on) of recent switches of our high beam
    self._adjacent = False
    self._town_hist: list[tuple[float, int]] = []  # (t, town lights in that frame)
    self._town = False
    self._town_low_since: float | None = None
    self._wide_hits: list[WideLight] = []  # the wide guard's lamps on its last look that saw any...
    self._wide_t: float | None = None         # ...and when that was
    self.last_mask = None
    self.last_blobs: list[Blob] = []
    self.last_wide: list[WideLight] = []      # every light the wide guard looked at on its last look

  def _associate(self, t, blobs, geom, center, r_rel):
    p = self.p
    unmatched = list(range(len(blobs)))
    for tr in self.tracks:
      last = tr.hist[-1]
      pu, pv = rotate_pixel(last.u, last.v, r_rel) if r_rel is not None else (last.u, last.v)
      tr.pred = (pu, pv)
      best, best_d = None, p.gate_px
      for j in unmatched:
        b = blobs[j]
        d = math.hypot(b.u - pu, b.v - pv)
        if d < best_d:
          best, best_d = j, d
      if best is None:
        tr.missed += 1
        continue
      unmatched.remove(best)
      tr.missed = 0
      tr.hist.append(self._obs(t, blobs[best], geom, center))
      tr.hist = [h for h in tr.hist[-40:] if h.t >= t - 4.0]
    born = self._beam_switches and self._beam_switches[-1][1] and t - self._beam_switches[-1][0] <= p.beam_born_s
    for j in unmatched:
      b = blobs[j]
      tr = Track(self._next_id, [self._obs(t, b, geom, center)], first_t=t)
      # our high beam just came on and a light without a partner appeared with it: a reflector it lit up (a wide
      # blob counts as paired elsewhere, but a lit sign is wide too)
      tr.reflector = bool(born and not b.partner)
      self._next_id += 1
      self.tracks.append(tr)
    self.tracks = [tr for tr in self.tracks if t - tr.hist[-1].t <= p.max_missed_s]

  def _obs(self, t, b: Blob, geom: Geometry, center):
    p = self.p
    rh = geom.ray_heights(b.u, b.v, center)
    hit = geom.ray_to_road(b.u, b.v, center, p.lamp_height, rh)
    if hit is None or hit[0] <= 0 or hit[0] > 2000:
      # above the lamp surface all the way out: very far (then it's near the road's far end), or not a lamp at all.
      # angle is the bearing from that far end instead.
      dist = lateral = angle = float("nan")
    else:
      dist = hit[0]
      lateral = hit[1] - np.interp(dist, center[:, 0], center[:, 1])
      angle = math.degrees(math.atan2(lateral, dist))
    high = False
    if rh is not None:
      in_band = (rh[1][1:] >= p.lamp_low - p.band_slack) & (rh[1][1:] <= p.lamp_high + p.band_slack)
      high = not in_band.any() or (b.area < p.near_band_area and center[in_band, 0].max() < p.near_band_m)
    bend = 0.0
    if math.isfinite(angle) and p.oncoming_side * angle < -p.roadside_angle_deg:
      # on a straight road a light beside it only gets farther from it along its ray: the lanes are crossed, if at all,
      # closer than the light. Crossed beyond it, the road curves into the ray.
      lanes = (p.ego_half_width + p.oncoming_width, p.ego_half_width)
      reach = lamp_reach(geom, b.u, b.v, center, lanes if p.oncoming_side < 0 else lanes[::-1], p)
      bend = reach / dist if math.isfinite(reach) else 0.0
    return Obs(t, b.u, b.v, dist, lateral, b.area, b.red, b.peak_y, angle, b.paired, b.post, b.flux, b.sat, high, b.partner,
               b.glow < p.glow_ratio, bend)

  def _beam_check(self, tr: Track):
    """A track seen on both sides of a switch of our high beam whose brightness followed it is a retroreflector."""
    p = self.p
    for ts, on in self._beam_switches:
      before = [h.flux for h in tr.hist if ts - p.beam_before_s <= h.t < ts]
      after = [h.flux for h in tr.hist if ts + p.beam_after_s[0] <= h.t <= ts + p.beam_after_s[1]]
      if not before or not after:
        continue
      ratio = (median(after) + 1.0) / (median(before) + 1.0)
      if (ratio >= p.beam_ratio) if on else (ratio <= 1.0 / p.beam_ratio):
        tr.reflector = True

  def _classify(self, tr: Track, v_ego, center):
    """Per-frame rules, then a vote over the last static_vote_s: single frames flip between rules (a far sign is
    a post on one frame and a lone light on the next), and each flip to a threat would dip the beam."""
    p = self.p
    self._classify_frame(tr, v_ego, center)
    t = tr.hist[-1].t
    tr.votes = [v for v in tr.votes if v[0] >= t - p.static_vote_s] + [(t, tr.kind == KIND_STATIC)]
    vehicle = tr.kind == KIND_ONCOMING or (tr.strong(p) and tr.hist[-1].paired)
    if (tr.kind != KIND_STATIC and not vehicle and len(tr.votes) >= 3
        and sum(v[1] for v in tr.votes) >= p.static_vote_frac * len(tr.votes)):
      tr.kind, tr.why = KIND_STATIC, "vote"
    # missing an oncoming car is glare in someone's eyes; a sign we dip for costs a few seconds of high beam
    last = tr.hist[-1]
    oncoming_side = not np.isfinite(last.angle) or p.oncoming_side * last.angle > -p.glare_side_deg
    if (tr.strong(p) and not last.red and not last.high and not last.post and oncoming_side and tr.kind == KIND_STATIC
        and not tr.reflector and tr.why != "sign"):  # a light that followed our own switching, or whose glow ends at
                                                     # its edge, is a sign, however bright
      tr.kind, tr.why = KIND_ONCOMING, "glare"

  def _classify_frame(self, tr: Track, v_ego, center):
    p = self.p
    last = tr.hist[-1]
    tr.dist, tr.lateral = last.dist, last.lateral
    red = tr.red_frac > 0.5
    any_red = red or last.red
    if not tr.reflector and self._beam_switches:
      self._beam_check(tr)
    if tr.reflector:
      # it lit up with our beam: a sign, until it shows what a sign can't, a lamp pair or oncoming motion
      tr.obj_speed = self._motion_fit(tr, v_ego)
      if sum(h.partner for h in tr.hist[-5:]) >= 3 or (np.isfinite(tr.obj_speed) and tr.obj_speed > p.oncoming_speed):
        tr.reflector = False
      else:
        tr.obj_speed = float("nan")
        tr.kind, tr.why = KIND_STATIC, "beam"
        return
    recent = tr.hist[-5:]
    if sum(h.compact for h in recent) * 2 > len(recent) and sum(h.partner for h in recent) < 3:
      # its glow ends at its edge: a sign our beams light, not a lamp (unless it shows a lamp pair, or moves at us)
      tr.obj_speed = self._motion_fit(tr, v_ego)
      if not (np.isfinite(tr.obj_speed) and tr.obj_speed > p.oncoming_speed):
        tr.obj_speed = float("nan")
        tr.kind, tr.why = KIND_STATIC, "sign"
        return
    if sum(h.post for h in recent) * 2 > len(recent):  # unpaired and tall, however bright: lamps are round or wide
      tr.obj_speed = float("nan")
      tr.kind, tr.why = KIND_STATIC, "post"  # delineator post
      return
    if sum(h.high for h in recent) * 2 > len(recent):
      tr.obj_speed = float("nan")
      tr.kind, tr.why = KIND_STATIC, "high"  # above lamp height, off the far road: building, sign, streetlight
      return
    angles = np.array([h.angle for h in recent], dtype=float)
    near_lane = np.isfinite(last.lateral) and abs(last.lateral) < p.big_white_max_lateral
    big_white = not red and last.area >= p.strong_area and near_lane  # real headlights pointing at the road (driveway)
    # red lights on the far side: far out they're taillights on a curve the model's path lags; with a lane there
    # they're cars in it. Close by on a road without one they're parked cars and reflectors.
    near = np.isfinite(last.dist) and last.dist < p.motion_max_dist
    exempt = big_white or (any_red and (self._adjacent or not near))
    finite = angles[np.isfinite(angles)]  # np.nanmedian goes through masked arrays for small inputs: slow
    bend = sum(h.bend > p.roadside_bend_k for h in recent) * 2 > len(recent)
    if not exempt and not bend and len(finite) and p.oncoming_side * median(finite) < -p.roadside_angle_deg:
      tr.obj_speed = float("nan")
      tr.kind, tr.why = KIND_STATIC, "roadside"  # roadside on our side: reflectors, parked cars, driveway markers
      return
    tr.obj_speed = self._motion_fit(tr, v_ego)
    if np.isfinite(tr.obj_speed):
      tr.why = "motion"
      if abs(tr.obj_speed) < p.static_speed:
        tr.kind = KIND_STATIC      # moves with the road: reflector, sign, parked car, house light
      elif tr.obj_speed > p.oncoming_speed and not red:
        tr.kind = KIND_ONCOMING
      elif red:
        tr.kind = KIND_PRECEDING
      elif tr.obj_speed < 0:
        # a white light "moving away" sits lower than a lamp: raised pavement markers, low reflectors
        tr.kind = KIND_STATIC
      else:
        tr.kind = KIND_CANDIDATE
    else:
      tr.kind, tr.why = (KIND_PRECEDING if red else KIND_CANDIDATE), "colour"

  def _motion_fit(self, tr: Track, v_ego):
    """The object's own speed from the inverse-depth slope over motion_s (>0 coming at us, ~0 static, <0 moving away),
    NaN when the track can't support it: too short, too far (1 px of row is tens of metres), or we're too slow."""
    p = self.p
    last = tr.hist[-1]
    seg = [h for h in tr.hist if h.t >= last.t - p.motion_s]
    dists = np.array([h.dist for h in seg])
    if not (len(seg) >= p.motion_min_obs and seg[-1].t - seg[0].t >= 0.5 * p.motion_s and v_ego >= p.min_motion_v_ego
            and np.isfinite(dists).all() and dists.max() < p.motion_max_dist):
      return float("nan")
    ts = np.array([h.t for h in seg]) - seg[0].t
    inv = 1.0 / np.clip(dists, 1.0, 2000.0)
    slope = np.polyfit(ts, inv, 1)[0]   # d(1/d)/dt = closing / d^2
    return slope * tr.dist ** 2 - v_ego

  def _ego_rotation(self, t, ang_vel):
    """Device rotation since the last frame, for predicting tracks. None without gyro.
    Only frame to frame: the model's road is already in the current frame's calibrated coordinates, so the projection
    needs no correction. (Rotating it by a high-passed integral of the gyro swung the corridor ~13 degrees off the road
    in sustained turns on route 46, and oncoming cars in curves got the high beam.)"""
    dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.5)
    self._last_t = t
    if ang_vel is None or dt == 0.0:
      return None
    return cv2.Rodrigues(np.asarray(ang_vel, dtype=float) * dt)[0]

  def _beam_update(self, t, high_beam_on):
    if high_beam_on is None:
      return
    if self._beam is not None and high_beam_on != self._beam:
      self._beam_switches.append((t, bool(high_beam_on)))
    self._beam = bool(high_beam_on)
    horizon = self.p.beam_after_s[1] + 0.2
    self._beam_switches = [s for s in self._beam_switches if t - s[0] <= horizon]

  def _town_update(self, t, n, speed_limit):
    """Town from streetlight density with hysteresis, or from a posted limit. Returns (town, mean lights per frame)."""
    p = self.p
    self._town_hist = [h for h in self._town_hist if h[0] > t - p.town_window_s] + [(t, n)]
    density = sum(h[1] for h in self._town_hist) / len(self._town_hist)
    full = t - self._town_hist[0][0] >= 0.5 * p.town_window_s  # one bright frame after a reset is no town
    if density >= p.town_on and full:
      self._town = True
      self._town_low_since = None
    elif density < p.town_off:
      if self._town_low_since is None:
        self._town_low_since = t
      if t - self._town_low_since >= p.town_exit_s:
        self._town = False
    else:
      self._town_low_since = None
    limit_town = speed_limit is not None and 0.0 < speed_limit <= p.town_limit
    return self._town or limit_town, density

  def update(self, t, frame, model, rpy_calib, height, v_ego, dark=True, valid=True, high_beam_on=None,
             ang_vel=None, speed_limit=None, wide=None, wide_euler=None) -> HbaOut:
    """frame: the narrow-camera image, BGR array or Nv12Frame. high_beam_on: our high beam as the car reports it,
    None if unknown. ang_vel: device-frame angular velocity (rad/s, livePose), None if unknown. speed_limit: the
    posted limit (m/s, mapd), None or 0 if unknown. wide: a wide-camera Nv12Frame from about the same time, only on
    the frames the wide guard should look (its hits hold for wide_hold_s), with liveCalibration.wideFromDeviceEuler."""
    p, out = self.p, HbaOut()
    r_rel = self._ego_rotation(t, ang_vel)
    if not valid or model is None or rpy_calib is None:
      self._reset()
      out.reason = REASON_FAULT
      return out
    self._beam_update(t, high_beam_on)
    geom = Geometry(rpy_calib, height)
    center = road_centerline(model, p, height)
    mask, mask_box = corridor_mask(geom, center, p, with_box=True)
    self._adjacent = adjacent_far_lane(model, p)
    red_mask, box = None, mask_box
    if self._adjacent:
      red_mask, red_box = corridor_mask(geom, center, p, p.adjacent_lane_width, with_box=True)
      box = union_box(mask_box, red_box)
    blobs = find_blobs(frame, mask, p, red_mask, box)
    self.last_mask, self.last_blobs, self.last_center, self.last_geom = mask, blobs, center, geom
    self._associate(t, blobs, geom, center, r_rel)
    # by day the sky between the trees is bright and pale too: count only in the dark
    n_town = town_lights(frame, geom, center, mask, p, mask_box) if dark else 0
    out.town, out.town_lights = self._town_update(t, n_town, speed_limit)
    for tr in self.tracks:
      if tr.missed == 0:
        self._classify(tr, v_ego, center)
    wide_threat = self._wide_update(t, wide, wide_euler, model, rpy_calib, height, geom, dark)
    out.n_wide = len(self._wide_hits) if wide_threat else 0

    lead = max(((pr, x) for pr, x, _ in model["leads"]), default=(0.0, 0.0))
    out.lead_prob = lead[0]
    model_lead = lead[0] > p.lead_prob and lead[1] < p.lead_max_x
    views = [tr.view(p) for tr in self.tracks if tr.missed == 0]
    return self.policy.step(t, out, views, model_lead, wide_threat, v_ego, dark, bool(self._beam))

  def _wide_update(self, t, wide, wide_euler, model, rpy_calib, height, narrow: Geometry, dark):
    """Looks at the wide frame if there is one. Returns whether a wide lamp threatens (held for wide_hold_s)."""
    p = self.p
    if wide is not None and wide_euler is not None and dark:
      wp = wide_params(p)
      self.last_wide = wide_lights(wide, Geometry(rpy_calib, height, wide_euler), road_centerline(model, wp, height), wp,
                                   narrow)
      hits = [w for w in self.last_wide if w.lamp]
      if hits:
        self._wide_hits, self._wide_t = hits, t
    return self._wide_t is not None and t - self._wide_t <= p.wide_hold_s

  def _reset(self):
    self.tracks = []
    self._wide_hits, self._wide_t = [], None
    self.policy.reset()
    self._beam_switches = []
    self._town_hist = []
    self._town = False
    self._town_low_since = None
