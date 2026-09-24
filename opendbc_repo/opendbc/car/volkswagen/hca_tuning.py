# StarPinguPilot: user-selectable HCA status and torque rate limits for PQ.
#
# The selected rates are latched into the panda safety param when the car starts, so the panda enforces
# exactly what was chosen. Lower rates can be picked live (applied on the next engagement); raising a rate
# above what was latched takes effect on the next car start.

# VolkswagenHCAMode param index -> HCA_1.HCA_Status sent while steering
HCA_MODES = (5, 7)
DEFAULT_HCA_MODE = 1

# VolkswagenHCADeltaRateUp/Down param index -> HCA counts per 50Hz frame
# Must match VOLKSWAGEN_PQ_HCA_DELTA_RATES in opendbc/safety/modes/volkswagen_pq.h
HCA_DELTA_RATES = (10, 30, 50, 150, 300)

# Settings UI: param -> (title, option labels), param value = option index
HCA_SETTINGS = {
  "VolkswagenHCAMode": ("HCA Mode", tuple(f"HCA {status}" for status in HCA_MODES)),
  "VolkswagenHCADeltaRateUp": ("HCA Rate Up", tuple(str(rate) for rate in HCA_DELTA_RATES)),
  "VolkswagenHCADeltaRateDown": ("HCA Rate Down", tuple(str(rate) for rate in HCA_DELTA_RATES)),
}

PQ_RATE_UP_SHIFT = 3
PQ_RATE_DOWN_SHIFT = 6
PQ_RATE_MASK = 0x7


def _clip_index(idx, options) -> int:
  try:
    idx = int(idx)
  except (TypeError, ValueError):
    return 0
  return idx if 0 <= idx < len(options) else 0


def hca_status(mode_idx) -> int:
  try:
    mode_idx = int(mode_idx)
  except (TypeError, ValueError):
    mode_idx = DEFAULT_HCA_MODE
  return HCA_MODES[mode_idx] if 0 <= mode_idx < len(HCA_MODES) else HCA_MODES[DEFAULT_HCA_MODE]


def delta_rates(up_idx, down_idx) -> tuple[int, int]:
  return HCA_DELTA_RATES[_clip_index(up_idx, HCA_DELTA_RATES)], HCA_DELTA_RATES[_clip_index(down_idx, HCA_DELTA_RATES)]


def encode_pq_safety_param(up_idx, down_idx) -> int:
  up_idx, down_idx = _clip_index(up_idx, HCA_DELTA_RATES), _clip_index(down_idx, HCA_DELTA_RATES)
  return ((up_idx + 1) << PQ_RATE_UP_SHIFT) | ((down_idx + 1) << PQ_RATE_DOWN_SHIFT)


def decode_pq_safety_param(safety_param: int, stock_up: int, stock_down: int) -> tuple[int, int]:
  """Rates the panda will enforce for this safety param."""
  rates = []
  for shift, stock in ((PQ_RATE_UP_SHIFT, stock_up), (PQ_RATE_DOWN_SHIFT, stock_down)):
    field = (safety_param >> shift) & PQ_RATE_MASK
    rates.append(HCA_DELTA_RATES[field - 1] if 1 <= field <= len(HCA_DELTA_RATES) else stock)
  return rates[0], rates[1]
