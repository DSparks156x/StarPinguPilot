"""StarPinguPilot: parked Volkswagen PQ HCA tuning and camera-lights choices shared by native settings and Galaxy."""
from opendbc.car.volkswagen.fla import LIGHTD_SETTINGS
from opendbc.car.volkswagen.hca_tuning import DEFAULT_HCA_MODE, HCA_SETTINGS
from opendbc.car.volkswagen.values import VolkswagenFlags
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

# param -> (title, option labels); the saved value is the option index
SETTINGS = {**HCA_SETTINGS, **LIGHTD_SETTINGS}
KEYS = frozenset(SETTINGS)
DEFAULTS = {"VolkswagenHCAMode": DEFAULT_HCA_MODE, "VolkswagenHCADeltaRateUp": 0, "VolkswagenHCADeltaRateDown": 0,
            "VolkswagenLightdMode": 0}
REASONS = {
  "VolkswagenHCAMode": "HCA status sent while steering. Applies on the next engagement.",
  "VolkswagenHCADeltaRateUp": "Steering torque rise limit, enforced by the panda. A higher rate applies at the next car start.",
  "VolkswagenHCADeltaRateDown": "Steering torque fall limit, enforced by the panda. A higher rate applies at the next car start.",
  "VolkswagenLightdMode": "Camera-controlled low beam and high beam assist. Needs the patched J519. Applies at the next car start.",
}


def read_choice(params, key):
  """(index, raw, readable, valid); invalid bytes read as the default index."""
  labels = SETTINGS[key][1]
  try:
    raw, readable = read_saved(params, key, 8)
  except (OSError, RuntimeError, TypeError, ValueError):
    return DEFAULTS[key], None, False, False
  if raw is None:
    return DEFAULTS[key], raw, readable, readable
  try:
    value = int(raw)
  except ValueError:
    return DEFAULTS[key], raw, readable, False
  valid = readable and raw == str(value).encode() and 0 <= value < len(labels)
  return (value if valid else DEFAULTS[key]), raw, readable, valid


class VolkswagenPqFeature:
  def __init__(self, owner):
    self.owner = owner

  def capability(self):
    cp = self.owner.vehicle_params()
    try:
      if cp is None or cp.brand != "volkswagen" or cp.notCar or not cp.flags & VolkswagenFlags.PQ:
        return None
      # not safetyParam: it carries the latched HCA rates, which these settings change
      return (str(cp.carFingerprint), int(cp.flags))
    except (AttributeError, TypeError, ValueError, OverflowError):
      return None

  def rows(self, parked):
    capability = self.capability()
    if capability is None:
      return ()
    available = parked and self.owner.authority("parked_preferences")
    rows = []
    for key, (title, labels) in SETTINGS.items():
      value, raw, readable, valid = read_choice(self.owner.params, key)
      default = labels[DEFAULTS[key]]
      reason = REASONS[key] if valid else ("Restore the default to repair this saved choice." if readable else "Saved choice cannot be read.")
      rows.append(FeatureRow(key, title, labels[value] if valid else "Invalid saved choice", raw, choices=labels,
                             available=available and readable, capability=capability,
                             repair_value="" if valid else default, default_value=default, reason=reason))
    return tuple(rows)

  def apply(self, request):
    if request.key not in KEYS or request.value not in SETTINGS[request.key][1]:
      return False
    if request.dependencies or request.related_source is not None or request.display_unit or request.direction:
      return False
    value = SETTINGS[request.key][1].index(request.value)
    owner = self.owner

    def authorized():
      capability = self.capability()
      _, raw, readable, valid = read_choice(owner.params, request.key)
      return (owner.authority("parked_preferences") and bool(request.vehicle_fingerprint) and
              owner.vehicle_fingerprint() == request.vehicle_fingerprint and capability is not None and
              capability == request.capability and readable and raw == request.expected and
              (valid or value == DEFAULTS[request.key]))

    result = commit_exact(owner.params, key=request.key, max_bytes=8, raw=str(value).encode(), expected=request.expected,
                          authorized=authorized, temp_prefix=".vw-pq-")
    return result.committed and result.verified
