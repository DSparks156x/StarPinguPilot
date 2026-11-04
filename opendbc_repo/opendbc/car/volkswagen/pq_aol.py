"""StarPinguPilot: PQ stock-cruise always-on lateral capability (see safety/modes/volkswagen_pq_aol.h)."""
from opendbc.car import structs
from opendbc.car.volkswagen.stock_aol import safety_config
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags, VolkswagenSafetyFlags

AOL_WORD = int(VolkswagenSafetyFlags.PQ_AOL)
PQ_CARS = frozenset(car for car in CAR if car.config.flags & VolkswagenFlags.PQ)
KNOWN_FLAGS = int(VolkswagenFlags.PQ | VolkswagenFlags.PQ_CC_ONLY | VolkswagenFlags.NO_EXT_CAN | VolkswagenFlags.HAS_BSM)


def word_supported(param):
  return bool(param & AOL_WORD) and not param & VolkswagenSafetyFlags.LONG_CONTROL


def _qualified(cp, *, marked_only=False):
  if (cp is None or cp.brand != "volkswagen" or cp.carFingerprint not in PQ_CARS or
      cp.passive or cp.notCar or cp.dashcamOnly or cp.flags & ~KNOWN_FLAGS or
      cp.steerControlType != structs.CarParams.SteerControlType.torque or
      cp.openpilotLongitudinalControl or not cp.pcmCruise):
    return False
  safety = safety_config(cp)
  return (safety is not None and safety.safetyModel == structs.CarParams.SafetyModel.volkswagenPq and
          not safety.safetyParam & VolkswagenSafetyFlags.LONG_CONTROL and
          (cp.alternativeExperience == 32 and word_supported(safety.safetyParam) or
           not marked_only and cp.alternativeExperience == 0 and not safety.safetyParam & AOL_WORD))


def qualified(cp, *, marked_only=False):
  try:
    return bool(_qualified(cp, marked_only=marked_only))
  except (AttributeError, TypeError, ValueError, OverflowError, IndexError):
    return False
