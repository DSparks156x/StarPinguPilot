#!/usr/bin/env python3
# StarPinguPilot: PQ safety for cars without an extended CAN (Audi TT Mk2) and user-selectable HCA rates
import unittest

from opendbc.car.volkswagen.hca_tuning import HCA_DELTA_RATES, encode_pq_safety_param
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety
import opendbc.safety.tests.test_volkswagen_pq as pq
from opendbc.safety.tests.test_volkswagen_pq import MSG_HCA_1, MSG_GRA_NEU, MSG_LDW_1


class TestVolkswagenPqNoExtCanSafety(pq.TestVolkswagenPqStockSafety):
  BUS = 1
  TX_MSGS = [[MSG_HCA_1, 1], [MSG_GRA_NEU, 1], [MSG_LDW_1, 1]]
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  RELAY_MALFUNCTION_ADDRS = {1: (MSG_HCA_1, MSG_LDW_1)}

  def setUp(self):
    self.packer = CANPackerSafety("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, VolkswagenSafetyFlags.PQ_NO_EXT_CAN.value)
    self.safety.init_tests()

  def _speed_msg(self, speed):
    return self.packer.make_can_msg_safety("Bremse_1", self.BUS, {"BR1_Rad_kmh": speed})

  def _torque_driver_msg(self, torque):
    return self.packer.make_can_msg_safety("Lenkhilfe_3", self.BUS, {"LH3_LM": abs(torque), "LH3_LMSign": torque < 0})

  def _torque_cmd_msg(self, torque, steer_req=1, hca_status=5):
    values = {"LM_Offset": abs(torque), "LM_OffSign": torque < 0, "HCA_Status": hca_status if steer_req else 3}
    return self.packer.make_can_msg_safety("HCA_1", self.BUS, values)

  def _motor_2_msg(self, brake_pressed=False, cruise_engaged=False):
    return self.packer.make_can_msg_safety("Motor_2", self.BUS, {"MO2_BLS": brake_pressed, "MO2_Sta_GRA": cruise_engaged})

  def _motor_5_msg(self, main_switch=False):
    return self.packer.make_can_msg_safety("Motor_5", self.BUS, {"MO5_GRA_Hauptsch": main_switch})

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("Motor_3", self.BUS, {"MO3_Pedalwert": gas})

  def _button_msg(self, _set=False, resume=False, cancel=False, bus=1):
    values = {"GRA_Neu_Setzen": _set, "GRA_Recall": resume, "GRA_Abbrechen": cancel}
    return self.packer.make_can_msg_safety("GRA_Neu", bus, values)

  def test_bus_0_ignored(self):
    # Car messages on bus 0 must not affect the safety state when everything lives on bus 1
    self.safety.set_controls_allowed(0)
    self._rx(self.packer.make_can_msg_safety("Motor_2", 0, {"MO2_Sta_GRA": 1}))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_hca_7(self):
    self.safety.set_controls_allowed(1)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_RATE_UP, hca_status=7)))


class TestVolkswagenPqRateSafety(pq.TestVolkswagenPqStockSafety):
  # The common realtime limit test needs MAX_RT_DELTA < MAX_TORQUE, so the faster rates are covered by test_all_rates
  RATE_UP_IDX = 0
  RATE_DOWN_IDX = 2
  MAX_RATE_UP = HCA_DELTA_RATES[RATE_UP_IDX]
  MAX_RATE_DOWN = HCA_DELTA_RATES[RATE_DOWN_IDX]
  MAX_RT_DELTA = (MAX_RATE_UP * 75 + 3) // 4

  def setUp(self):
    self.packer = CANPackerSafety("vw_pq")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, encode_pq_safety_param(self.RATE_UP_IDX, self.RATE_DOWN_IDX))
    self.safety.init_tests()

  def test_all_rates(self):
    for up_idx, rate in enumerate(HCA_DELTA_RATES):
      with self.subTest(rate=rate):
        self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, encode_pq_safety_param(up_idx, 0))
        self.safety.init_tests()
        self.safety.set_controls_allowed(1)
        self._set_prev_torque(0)
        self.assertTrue(self._tx(self._torque_cmd_msg(rate)))
        # one count over the selected rate (or over max torque for the 300 rate) is blocked
        self._set_prev_torque(0)
        self.assertFalse(self._tx(self._torque_cmd_msg(rate + 1)))


class TestVolkswagenPqRateNoExtCanSafety(TestVolkswagenPqNoExtCanSafety):
  MAX_RATE_UP = HCA_DELTA_RATES[0]
  MAX_RATE_DOWN = HCA_DELTA_RATES[0]
  MAX_RT_DELTA = (MAX_RATE_UP * 75 + 3) // 4

  def setUp(self):
    self.packer = CANPackerSafety("vw_pq")
    self.safety = libsafety_py.libsafety
    param = VolkswagenSafetyFlags.PQ_NO_EXT_CAN.value | encode_pq_safety_param(0, 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagenPq, param)
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()
