"""StarPinguPilot: Volkswagen PQ HCA mode/rate and camera-light (lightd) preferences.

The HCA rates and the camera-light TX permission are latched into the panda safety param at car start, before the
car interface is built, so the panda enforces exactly what was selected. Lower HCA rates and the HCA mode can still be
changed live; they apply on the next engagement (see opendbc/car/volkswagen/carcontroller.py).
"""
from dataclasses import dataclass

from opendbc.car.volkswagen import fla, hca_tuning
from opendbc.car.volkswagen.values import VolkswagenFlags
from openpilot.starpilot.saved_source import read_saved

HCA_KEYS = ("VolkswagenHCAMode", "VolkswagenHCADeltaRateUp", "VolkswagenHCADeltaRateDown")
LIGHTD_KEY = "VolkswagenLightdMode"
DEFAULTS = {"VolkswagenHCAMode": hca_tuning.DEFAULT_HCA_MODE, "VolkswagenHCADeltaRateUp": 0,
            "VolkswagenHCADeltaRateDown": 0, LIGHTD_KEY: fla.LIGHTD_OFF}
READ_PERIOD_FRAMES = 100  # 1 s at the 100 Hz controller rate


def _read_int(params, key: str) -> int:
  try:
    raw, readable = read_saved(params, key, 8)
    return int(raw) if readable and raw not in (None, b"") else DEFAULTS[key]
  except (OSError, TypeError, ValueError):
    return DEFAULTS[key]


def _safe_mode(params) -> bool:
  try:
    raw, readable = read_saved(params, "SafeMode", 8)
    return not readable or raw not in (None, b"0")
  except (OSError, TypeError, ValueError):
    return True


def pq(cp) -> bool:
  return cp.brand == "volkswagen" and bool(cp.flags & VolkswagenFlags.PQ)


def lightd_mode(params) -> int:
  """Selected lightd mode, off in Safe Mode."""
  return fla.LIGHTD_OFF if _safe_mode(params) else _read_int(params, LIGHTD_KEY)


@dataclass(frozen=True)
class PqStartup:
  rate_up: int = 0    # HCA_DELTA_RATES index
  rate_down: int = 0
  lightd_mode: int = fla.LIGHTD_OFF

  @classmethod
  def read(cls, params):
    if _safe_mode(params):
      return cls()
    return cls(_read_int(params, "VolkswagenHCADeltaRateUp"), _read_int(params, "VolkswagenHCADeltaRateDown"),
               _read_int(params, LIGHTD_KEY))

  def prepare(self, cp) -> None:
    """Latch the selected HCA rates (and mFLA_1 TX on the TT bus) into the PQ panda safety param."""
    if not pq(cp) or not cp.safetyConfigs:
      return
    bits = hca_tuning.encode_pq_safety_param(self.rate_up, self.rate_down)
    if cp.flags & VolkswagenFlags.NO_EXT_CAN and self.lightd_mode >= fla.LIGHTD_LOW_BEAM:
      bits |= fla.PQ_LIGHT_TX
    cp.safetyConfigs[-1].safetyParam |= bits


class HcaPreference:
  """Live HCA mode/rate selection for the PQ carcontroller, re-read once a second."""
  def __init__(self, params):
    self.params = params
    self.frames = 0
    self.value = self._read()

  def _read(self) -> tuple[int, int, int]:
    if _safe_mode(self.params):
      return DEFAULTS["VolkswagenHCAMode"], 0, 0
    return tuple(_read_int(self.params, key) for key in HCA_KEYS)

  def read(self) -> tuple[int, int, int]:
    if self.frames % READ_PERIOD_FRAMES == 0:
      self.value = self._read()
    self.frames += 1
    return self.value
