# StarPinguPilot: PQ always-on lateral qualification and safety word (car side of safety/modes/volkswagen_pq_aol.h).
from opendbc.car import structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, VolkswagenSafetyFlags
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.aol.vehicle import native_matches_cp, native_profile_supported, policy_for as vehicle_policy_for
from openpilot.starpilot.car.volkswagen import aol

PQ_MODEL = int(structs.CarParams.SafetyModel.volkswagenPq)
# Bus 1 as fingerprinted on a real TT Mk2 (2026-11-03 swaglog), trimmed to what interface.py looks at.
TT_BUS1 = {0x1A0: 8, 0xC2: 8, 0x288: 8, 0x480: 8, 0x38A: 4, 0x3D2: 7, 0x390: 8}


def tt_cp(aol_word=True):
  fingerprint = {b: {} for b in range(8)}
  fingerprint[1] = dict(TT_BUS1)
  CP = CarInterface.get_params(CAR.AUDI_TT_MK2, fingerprint, [], False, False, False)
  if aol_word:  # what card.py does with the port policy
    policy = aol.policy_for(CP)
    CP.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
    CP.alternativeExperience |= policy.alternative_experience_addition
  return CP


def test_tt_policy_adds_pq_aol_word():
  CP = tt_cp(aol_word=False)
  assert aol.qualified(CP) and not aol.qualified(CP, marked_only=True)
  policy = aol.policy_for(CP)
  assert policy.intent_supported and policy.runtime_supported and policy.normal_runtime_supported
  assert policy.safety_param_addition == VolkswagenSafetyFlags.PQ_AOL and policy.alternative_experience_addition == 32


def test_marked_tt_is_accepted_natively():
  CP = tt_cp()
  param = CP.safetyConfigs[-1].safetyParam
  assert param & VolkswagenSafetyFlags.PQ_NO_EXT_CAN and param & VolkswagenSafetyFlags.PQ_AOL
  assert aol.qualified(CP, marked_only=True)
  assert aol.native_accepts_cp(CP, PQ_MODEL, param)
  assert native_profile_supported(PQ_MODEL, param)
  assert vehicle_policy_for(CP).intent_supported


def test_profile_rejects_mismatched_words():
  assert not native_profile_supported(PQ_MODEL, int(VolkswagenSafetyFlags.PQ_NO_EXT_CAN))
  assert not native_profile_supported(PQ_MODEL, int(VolkswagenSafetyFlags.PQ_AOL | VolkswagenSafetyFlags.LONG_CONTROL))
  assert not aol.native_profile_supported(int(structs.CarParams.SafetyModel.volkswagen), int(VolkswagenSafetyFlags.PQ_AOL))


def test_openpilot_longitudinal_pq_not_qualified():
  CP = tt_cp(aol_word=False)
  CP.openpilotLongitudinalControl = True
  CP.pcmCruise = False
  assert not aol.qualified(CP)


def _tick(owner, available, tick):
  state = structs.CarState(canValid=True, vEgo=20., gearShifter=structs.CarState.GearShifter.drive)
  state.cruiseState.available = available
  owner.update(state, now_ns=1_000_000_000 + tick * 10_000_000)


def test_main_on_at_startup_needs_off_first():
  # PQ stalks latch: a main switch already on at startup must not latch lateral; an off->on cycle does.
  owner = aol.create_intent(tt_cp(), AolSettings(True, 5., 0, 0, (0, 0, 0), (0, 0, 0)))
  for tick in range(15):
    _tick(owner, True, tick)
  assert not owner.allowed_latch
  for tick in range(15, 20):
    _tick(owner, False, tick)
  for tick in range(20, 35):
    _tick(owner, True, tick)
  assert owner.allowed_latch


def test_native_matches_cp_tt():
  CP = tt_cp()
  assert native_matches_cp(CP, PQ_MODEL, CP.safetyConfigs[-1].safetyParam)
