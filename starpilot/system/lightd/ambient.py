"""Low-beam ambient estimator from camera auto-exposure state. StarPinguPilot.

Pure logic, no messaging, so it runs the same on device and in offline replay.

The camera is used as a light meter: scene luminance ~ linear(median grey) / applied EV.
That is converted to stops and offset by a per-camera constant so it reads roughly log2(lux).

Two channels share one absolute threshold:
  slow (tau ~20 s): dusk, dawn, storms. Ordinary shade never lasts long enough to pull it down.
  fast (tau 0.3 s): tunnels and garages. Needs a sudden drop well below the slow channel, held for
                    min(dwell time, dwell distance), so highway underpasses (<1 s, <35 m in logged
                    drives) and dusk shadow flicker don't trigger it.
The sun only adds a night latch, so bright streets after sunset can't turn the lights off.
Tuned on 108 Audi TT drives, May-June 2026 (Fort Collins), see DESIGN.md.
"""
import math
from dataclasses import dataclass, field

# OS04C10 (comma four) AE constants, system/camerad/sensors/os04c10.cc
OS04C10_MIN_EV = 2.0
OS04C10_MAX_EV = 2352 * 1.0 * 8.5

LUX_PER_BUCKET = 714.286  # GK2_Helligkeit / RLS 3-bit ambient bucket scale (K-matrix)

REGIME_UNKNOWN, REGIME_NIGHT, REGIME_TWILIGHT, REGIME_DAY = range(4)


@dataclass
class CameraSample:
  t: float                  # seconds, monotonic
  grey: float               # measuredGreyFraction (median Y of the AE rect, gamma encoded)
  ev_percent: float         # exposureValPercent (linear map of the EV applied to this frame)
  frame_id: int = 0


@dataclass
class AmbientParams:
  k_wide: float = 23.0      # stops -> roughly log2(lux); places clear-sky sunset near 400-700 lux
  k_narrow: float = 22.4    # narrow reads ~0.6 stop brighter than wide at dusk (summer replay), only used as fallback
  on_threshold: float = 9.8   # log2 lux; clear evenings cross it at sun ~+5 deg (~30 min before sunset)
  hysteresis: float = 0.8     # stops above the threshold before releasing
  sensitivity: float = 0.0    # user offset in stops, + turns lights on earlier
  tau_slow: float = 20.0
  tau_fast_dark: float = 0.3
  tau_fast_light: float = 1.0
  tunnel_drop: float = 3.0        # fast must be this far below slow, so gradual dusk/storms stay on the slow channel
  tunnel_dwell_s: float = 2.0     # fast channel must stay dark this long...
  tunnel_dwell_m: float = 60.0    # ...or this far, whichever comes first
  tunnel_release_s: float = 1.0
  night_elev: float = 0.0         # sun below this: latched dark (sunset to sunrise)
  day_elev: float = 12.0          # only for the reported regime
  stale_s: float = 0.2
  settle_s: float = 6.0           # camerad AE takes up to ~5 s to converge after start (logged)
  self_illum_delta: float = 0.0   # stops our own low beam adds to the measurement


@dataclass
class AmbientOut:
  valid: bool = False
  regime: int = REGIME_UNKNOWN
  wide_b: float = float("nan")      # raw log2 lux estimates per camera
  narrow_b: float = float("nan")
  fast_b: float = float("nan")
  slow_b: float = float("nan")
  ambient_b: float = float("nan")   # what the bucket is derived from
  lux: float = float("nan")
  bucket: int = 0
  dark_request: bool = False
  dark_slow: bool = False
  dark_tunnel: bool = False
  night_latch: bool = False
  on_threshold: float = float("nan")
  confidence: float = 0.0
  tunnel_cue: float = float("nan")  # narrow - wide, stops
  wide_saturated: bool = False      # AE pinned at an end of its range
  narrow_saturated: bool = False


def scene_stops(grey: float, ev_percent: float, min_ev: float = OS04C10_MIN_EV, max_ev: float = OS04C10_MAX_EV) -> float:
  g = min(max(grey, 1e-3), 0.999)
  linear = g / (10.0 - 9.0 * g)  # inverse of the OS04C10 gamma 10x / (1 + 9x)
  ev = min_ev + min(max(ev_percent, 0.0), 100.0) / 100.0 * (max_ev - min_ev)
  return math.log2(linear) - math.log2(ev)


def ema(prev: float, x: float, dt: float, tau: float) -> float:
  return prev + (x - prev) * (1.0 - math.exp(-dt / tau))


@dataclass
class AmbientEstimator:
  p: AmbientParams = field(default_factory=AmbientParams)
  _fast: float | None = None
  _slow: float | None = None
  _last_t: float | None = None
  _valid_since: float | None = None
  _dark_slow: bool = False
  _dark_tunnel: bool = False
  _below_s: float = 0.0
  _below_m: float = 0.0
  _above_s: float = 0.0
  _tunnel: float | None = None

  def _sample_ok(self, s: CameraSample | None, t: float) -> bool:
    return s is not None and t - s.t < self.p.stale_s and 0.0 < s.grey < 1.0 and 0.0 <= s.ev_percent <= 100.0

  def _reset(self, t: float) -> None:
    self._fast = self._slow = self._valid_since = self._tunnel = None
    self._dark_tunnel = False
    self._below_s = self._below_m = self._above_s = 0.0
    self._last_t = t

  def update(self, t: float, wide: CameraSample | None, narrow: CameraSample | None,
             solar_elev: float | None, v_ego: float = 0.0, low_beam_on: bool = False) -> AmbientOut:
    p, out = self.p, AmbientOut()
    wide_ok, narrow_ok = self._sample_ok(wide, t), self._sample_ok(narrow, t)
    if wide_ok:
      out.wide_b = scene_stops(wide.grey, wide.ev_percent) + p.k_wide
      out.wide_saturated = wide.ev_percent <= 0.0 or wide.ev_percent >= 100.0
    if narrow_ok:
      out.narrow_b = scene_stops(narrow.grey, narrow.ev_percent) + p.k_narrow
      out.narrow_saturated = narrow.ev_percent <= 0.0 or narrow.ev_percent >= 100.0

    if not (wide_ok or narrow_ok):
      self._reset(t)
      return out

    raw = out.wide_b if wide_ok else out.narrow_b
    if low_beam_on:
      raw -= p.self_illum_delta

    dt = 0.0 if self._last_t is None else max(t - self._last_t, 0.0)
    self._last_t = t
    if self._valid_since is None:
      self._valid_since = t
    if t - self._valid_since < p.settle_s:
      # AE is still converging after camerad start; seeding the slow channel here leaves it wrong for a minute
      self._fast = self._slow = raw
    else:
      self._fast = ema(self._fast, raw, dt, p.tau_fast_dark if raw < self._fast else p.tau_fast_light)
      if not self._dark_tunnel:  # the slow channel is the outdoor ambient; tunnels and garages don't count
        self._slow = ema(self._slow, raw, dt, p.tau_slow)

    if wide_ok and narrow_ok:
      cue = out.narrow_b - out.wide_b
      self._tunnel = cue if self._tunnel is None else ema(self._tunnel, cue, dt, 0.5)
      out.tunnel_cue = self._tunnel

    on = p.on_threshold + p.sensitivity
    off = on + p.hysteresis
    out.on_threshold = on

    # slow channel: dusk, dawn, storms
    if self._slow < on:
      self._dark_slow = True
    elif self._slow > off:
      self._dark_slow = False

    # fast channel: tunnels and garages, ignoring short underpasses
    if self._fast < on and (self._dark_tunnel or self._slow - self._fast >= p.tunnel_drop):
      self._below_s += dt
      self._below_m += dt * max(v_ego, 0.0)
      self._above_s = 0.0
      if self._below_s >= p.tunnel_dwell_s or self._below_m >= p.tunnel_dwell_m:
        self._dark_tunnel = True
    else:
      self._below_s = self._below_m = 0.0
      self._above_s = self._above_s + dt if self._fast > off else 0.0
      if self._above_s >= p.tunnel_release_s:
        self._dark_tunnel = False

    if solar_elev is None:
      out.regime = REGIME_UNKNOWN
    elif solar_elev < p.night_elev:
      out.regime = REGIME_NIGHT
    else:
      out.regime = REGIME_DAY if solar_elev >= p.day_elev else REGIME_TWILIGHT
    out.night_latch = out.regime == REGIME_NIGHT

    out.fast_b, out.slow_b = self._fast, self._slow
    out.dark_slow, out.dark_tunnel = self._dark_slow, self._dark_tunnel
    out.dark_request = out.night_latch or self._dark_slow or self._dark_tunnel

    out.ambient_b = self._fast if self._dark_tunnel else self._slow
    out.lux = 2.0 ** out.ambient_b
    out.bucket = int(min(max(out.lux // LUX_PER_BUCKET, 0), 7))

    if out.night_latch:
      out.confidence = 1.0
    else:
      edge = on if out.dark_request else off
      out.confidence = min(1.0, abs(out.ambient_b - edge) / 2.0) * (0.7 if out.regime == REGIME_UNKNOWN else 1.0)

    out.valid = t - self._valid_since >= p.settle_s
    return out
