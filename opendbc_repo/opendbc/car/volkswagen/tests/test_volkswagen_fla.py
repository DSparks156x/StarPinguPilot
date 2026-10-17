# StarPinguPilot: mFLA_1 (0x65A) packing for camera lights.
# Layout: CentElecRE/artifacts/claude_handoff_06_finalize_byte0.md (byte 0 carrier) + comma_strategy_signal_contract.md (HBA)
import itertools

import pytest

from cereal import custom
from opendbc.can.packer import CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.tests.test_car_interfaces import get_test_starpilot_toggles
from opendbc.car.volkswagen.fla import (FLA_STEP, HBA_OFF, LIGHT_INVALID, PQ_LIGHT_TX, HbaCommand, LightCommand, apply_light_carrier,
                                        create_fla_1)
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR

packer = CANPacker("vw_pq")
HBA_STATES = (HBA_OFF, HbaCommand(True, True, 0), HbaCommand(True, False, 1), HbaCommand(True, False, 2, sensor_blocked=True))


def frame(light=LIGHT_INVALID, hba=HBA_OFF):
  addr, dat, bus = create_fla_1(packer, 1, light, hba)
  assert addr == 0x65A and bus == 1 and len(dat) == 8
  return dat


@pytest.mark.parametrize("light, expected", [
  (LightCommand(True, False, 0), 0x80),
  (LightCommand(True, True, 0), 0x88),
  (LightCommand(True, True, 2), 0x8A),
  (LightCommand(True, True, 7), 0x8F),
  (LightCommand(False, False, 0), 0x00),
  (LightCommand(False, True, 5), 0x00),  # invalid is always 00, whatever else is set
])
def test_byte0_handoff_vectors(light, expected):
  assert frame(light)[0] == expected


def test_reserved_bits_always_zero():
  for valid, on, code in itertools.product((False, True), (False, True), range(8)):
    assert frame(LightCommand(valid, on, code))[0] & 0x70 == 0


@pytest.mark.parametrize("bad", [dict(brightness=8), dict(brightness=-1), dict(brightness=2.0), dict(brightness=True),
                                 dict(valid=1), dict(headlight_on=None)])
def test_light_command_validation(bad):
  with pytest.raises(ValueError):
    LightCommand(**{"valid": True, "headlight_on": True, "brightness": 3, **bad})


def test_carrier_preserves_bytes_1_to_7():
  original = bytes(range(0x11, 0x19))
  out = apply_light_carrier(bytearray(original), LightCommand(True, True, 6))
  assert out[0] == 0x8E and bytes(out[1:]) == original[1:]
  for hba in HBA_STATES:
    reference = frame(LIGHT_INVALID, hba)
    for light in (LightCommand(True, False, 0), LightCommand(True, True, 7)):
      assert frame(light, hba)[1:] == reference[1:]


def test_low_beam_and_hba_expire_independently():
  light = LightCommand(True, True, 3)
  for hba in HBA_STATES:
    assert frame(light, hba)[0] == 0x8B           # HBA state never touches the low-beam byte
  hba_on = HbaCommand(True, True, 0)
  for lt in (LIGHT_INVALID, light):
    dat = frame(lt, hba_on)
    assert dat[3] & 0x7F == 100 and dat[7] & 0x40  # low-beam validity never touches the HBA demand
  dat = frame(light, HBA_OFF)                      # HBA expired while low beam keeps running
  assert dat[0] == 0x8B and dat[3] & 0x7F == 0 and not dat[7] & 0x40


def test_hba_permitted_sets_qualifier_and_full_level():
  dat = frame(hba=HbaCommand(active=True, permitted=True, reason=0))
  assert dat[3] & 0x7F == 100        # FL1_Fernlicht_PWM
  assert dat[7] & 0x40               # FL1_FLA_Aufblenden
  assert dat[2] == 0                 # no dip reason
  assert dat[1] & 0x20               # FL1_FLA_aktiv


def test_hba_dipped_or_stale_clears_both_demand_fields():
  for hba in (HbaCommand(active=True, permitted=False, reason=1), HBA_OFF):
    dat = frame(hba=hba)
    assert dat[3] & 0x7F == 0
    assert not dat[7] & 0x40
  assert frame(hba=HbaCommand(active=True, permitted=False, reason=1))[2] == 1
  assert frame(hba=HBA_OFF)[2] == 255
  assert not frame(hba=HBA_OFF)[1] & 0x20


def test_never_sets_sticky_defect():
  for light in (LIGHT_INVALID, LightCommand(True, True, 3)):
    for hba in HBA_STATES:
      assert not frame(light, hba)[1] & 0x40   # FL1_ISP_defekt
  assert frame(hba=HbaCommand(True, False, 2, sensor_blocked=True))[1] & 0x80


def test_isp_mirror_fields_report_no_mirror():
  dat = frame(LightCommand(True, True, 7))
  assert dat[4] & 0x3F == 62   # FL1_ISP_Lichtwert_vo init (byte 4 is OEM again, not the carrier)
  assert dat[4] & 0xC0 == 0
  assert dat[5] & 0x3F == 62   # FL1_ISP_Lichtwert_hi init
  assert dat[6] & 0x1F == 30   # FL1_ISP_Himmelsrichtung init
  assert not dat[6] & 0xE0     # Taste, EC_aktiv, Kompass_aktiv
  assert dat[7] & 0x3F == 0    # FL1_Dimmwert_ASP


def tt_interface(light_tx: bool):
  CP = CarInterface.get_non_essential_params(CAR.AUDI_TT_MK2)
  if light_tx:
    CP.safetyConfigs[-1].safetyParam |= PQ_LIGHT_TX
  CI = CarInterface(CP, custom.StarPilotCarParams.new_message())
  CI.starpilot_toggles = get_test_starpilot_toggles()
  return CI


def run_frames(CI, n):
  CC = structs.CarControl().as_reader()
  sent = []
  for i in range(n):
    CI.update([], CI.starpilot_toggles)
    _, can_sends = CI.apply(CC, int(i * DT_CTRL * 1e9), CI.starpilot_toggles)
    sent += [m for m in can_sends if m[0] == 0x65A]
  return sent


def test_tt_single_sender_10hz_only_with_safety_bit():
  assert run_frames(tt_interface(light_tx=False), 50) == []

  CI = tt_interface(light_tx=True)
  sent = run_frames(CI, 50)
  assert len(sent) == 50 // FLA_STEP and all(bus == 1 for _, _, bus in sent)  # one 0x65A per 100 ms, TT: bus 1
  assert all(dat[0] == 0 and dat[7] & 0x40 == 0 for _, dat, _ in sent)    # no producer yet: neutral frames

  CI.CS.fla_command = (LightCommand(True, True, 3), HbaCommand(True, True, 0))
  sent = run_frames(CI, FLA_STEP)
  assert len(sent) == 1
  dat = sent[0][1]
  assert dat[0] == 0x8B and dat[3] & 0x7F == 100 and dat[7] & 0x40
