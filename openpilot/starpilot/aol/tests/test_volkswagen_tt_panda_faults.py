"""StarPinguPilot: the TT Mk2 has only powertrain CAN, so the panda latches CAN interrupt-rate diagnostics from boot."""
import pytest

from opendbc.car import structs
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, VolkswagenFlags, VolkswagenSafetyFlags
from openpilot.cereal import log
from openpilot.starpilot.aol.panda_faults import blocking_fault_list, blocking_fault_mask

FaultType = log.PandaState.FaultType
# Every drive on the TT, 2026-11-03 and 2026-11-04, on old and new panda firmware
TT_FAULTS = (FaultType.interruptRateCan1, FaultType.interruptRateCan2, FaultType.interruptRateCan3)
TT_MASK = (1 << 2) | (1 << 3) | (1 << 4)
TT_BUS1 = [1657, 640, 672, 896, 1160, 1184, 1754, 704, 1056, 1096, 1440, 866, 648, 1152, 1162, 1416, 208, 194, 800,
           1360, 1088, 1344, 1352, 174, 416, 684, 928, 906, 1192, 1312, 976, 80, 428, 916, 1420, 1824, 946, 978, 1394,
           1392, 912, 1504, 914, 919, 1500, 1488, 1306, 1490, 2000, 1408, 1530, 1524]


def tt(aol=False):
  fingerprint = {b: {} for b in range(8)}
  fingerprint[1] = {addr: 8 for addr in TT_BUS1}
  cp = CarInterface.get_params(CAR.AUDI_TT_MK2, fingerprint, [], False, False, False)
  if aol:
    cp.alternativeExperience = 32
    cp.safetyConfigs[0].safetyParam |= int(VolkswagenSafetyFlags.PQ_AOL)
  return cp


@pytest.mark.parametrize('aol', (False, True))
def test_tt_unwired_bus_interrupt_faults_do_not_block(aol):
  cp = tt(aol)
  assert not blocking_fault_list(list(TT_FAULTS), cp)
  assert not blocking_fault_mask(TT_MASK, cp)
  for fault in TT_FAULTS:
    assert not blocking_fault_list([fault], cp)


@pytest.mark.parametrize('fault,value', tuple(FaultType.schema.enumerants.items()))
def test_tt_every_other_fault_still_blocks(fault, value):
  cp = tt()
  blocks = value not in [int(f) for f in TT_FAULTS]
  assert blocking_fault_list([value], cp) == blocks
  assert blocking_fault_list([*TT_FAULTS, value], cp) == blocks
  assert blocking_fault_mask(TT_MASK | (1 << value), cp) == blocks


@pytest.mark.parametrize('mutation', ('ext_can', 'long', 'wrong_model', 'no_bus1_word', 'extra_panda', 'other_pq'))
def test_other_configurations_cannot_borrow_exception(mutation):
  cp = tt()
  if mutation == 'ext_can':
    cp.flags = int(cp.flags) & ~int(VolkswagenFlags.NO_EXT_CAN)
  elif mutation == 'long':
    cp.openpilotLongitudinalControl = True
  elif mutation == 'wrong_model':
    cp.safetyConfigs[0].safetyModel = structs.CarParams.SafetyModel.volkswagen
  elif mutation == 'no_bus1_word':
    cp.safetyConfigs[0].safetyParam &= ~int(VolkswagenSafetyFlags.PQ_NO_EXT_CAN)
  elif mutation == 'extra_panda':
    cp.safetyConfigs = [structs.CarParams.SafetyConfig(), cp.safetyConfigs[0]]
  elif mutation == 'other_pq':
    cp.carFingerprint = CAR.VOLKSWAGEN_PASSAT_NMS
  assert blocking_fault_list(list(TT_FAULTS), cp)
  assert blocking_fault_mask(TT_MASK, cp)


def test_can3_alone_still_blocks_other_brands():
  from openpilot.starpilot.aol.tests.test_honda_panda_faults import honda
  assert blocking_fault_list([FaultType.interruptRateCan3], honda())
