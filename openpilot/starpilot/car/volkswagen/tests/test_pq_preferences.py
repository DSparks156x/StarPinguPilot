"""StarPinguPilot: PQ HCA rate / camera-light startup latch and live HCA selection."""
from pathlib import Path
import tempfile
import unittest

from opendbc.car.volkswagen import fla, hca_tuning
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC
from openpilot.common.params import Params
from openpilot.starpilot.car.volkswagen.pq_preferences import HcaPreference, PqStartup, lightd_mode
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


class TestPqPreferences(unittest.TestCase):
  def setUp(self):
    temporary = tempfile.TemporaryDirectory()
    self.addCleanup(temporary.cleanup)
    self.params = Params(temporary.name)

  def raw(self, key, value):
    path = Path(self.params.get_param_path(key))
    if value is None:
      path.unlink(missing_ok=True)
    else:
      path.write_bytes(value)

  def startup_param(self, candidate):
    cp = CarInterface.get_non_essential_params(candidate)
    base = cp.safetyConfigs[-1].safetyParam
    VehicleStartupPreferences.read(self.params, enabled=True).prepare(cp)
    return cp, cp.safetyConfigs[-1].safetyParam ^ base

  def test_rates_and_light_bit_latched_at_startup(self):
    self.raw("SafeMode", b"0")
    self.raw("VolkswagenHCADeltaRateUp", b"3")
    self.raw("VolkswagenHCADeltaRateDown", b"4")
    self.raw("VolkswagenLightdMode", b"2")
    cp, added = self.startup_param(CAR.AUDI_TT_MK2)
    self.assertEqual(added, hca_tuning.encode_pq_safety_param(3, 4) | fla.PQ_LIGHT_TX)
    # the controller never exceeds what the panda latched
    CC = CarController({bus: dbc for bus, dbc in DBC[cp.carFingerprint].items()}, cp)
    self.assertEqual((CC.CCP.STEER_DELTA_UP, CC.CCP.STEER_DELTA_DOWN), (150, 300))
    self.assertTrue(CC.fla_tx)

  def test_light_bit_only_on_tt_bus_and_low_beam_modes(self):
    self.raw("SafeMode", b"0")
    for mode, expected in ((b"0", False), (b"1", False), (b"2", True), (b"3", True)):
      self.raw("VolkswagenLightdMode", mode)
      self.assertEqual(bool(self.startup_param(CAR.AUDI_TT_MK2)[1] & fla.PQ_LIGHT_TX), expected)
      self.assertFalse(self.startup_param(CAR.VOLKSWAGEN_PASSAT_NMS)[1] & fla.PQ_LIGHT_TX)

  def test_non_pq_untouched(self):
    self.raw("VolkswagenHCADeltaRateUp", b"4")
    self.assertEqual(self.startup_param(CAR.VOLKSWAGEN_GOLF_MK7)[1], 0)

  def test_safe_mode_uses_defaults(self):
    self.raw("SafeMode", b"1")
    self.raw("VolkswagenHCADeltaRateUp", b"4")
    self.raw("VolkswagenLightdMode", b"3")
    self.assertEqual(PqStartup.read(self.params), PqStartup())
    self.assertEqual(lightd_mode(self.params), fla.LIGHTD_OFF)
    self.assertEqual(HcaPreference(self.params).read(), (hca_tuning.DEFAULT_HCA_MODE, 0, 0))

  def test_live_selection_reread_and_capped(self):
    self.raw("SafeMode", b"0")
    self.raw("VolkswagenHCAMode", b"0")
    preference = HcaPreference(self.params)
    self.assertEqual(preference.read(), (0, 0, 0))
    self.raw("VolkswagenHCAMode", b"1")
    self.raw("VolkswagenHCADeltaRateUp", b"4")
    self.assertEqual(preference.read(), (0, 0, 0))  # re-read once a second
    for _ in range(99):
      preference.read()
    self.assertEqual(preference.read(), (1, 4, 0))

    cp = CarInterface.get_non_essential_params(CAR.AUDI_TT_MK2)
    cp.safetyConfigs[-1].safetyParam |= hca_tuning.encode_pq_safety_param(1, 1)
    CC = CarController({bus: dbc for bus, dbc in DBC[cp.carFingerprint].items()}, cp)
    CC.update_pq_hca_tuning(*preference.read())
    self.assertEqual(CC.hca_status, 7)
    self.assertEqual((CC.CCP.STEER_DELTA_UP, CC.CCP.STEER_DELTA_DOWN), (30, 10))  # up capped at the latched 30

  def test_garbage_values_fall_back(self):
    self.raw("SafeMode", b"0")
    self.raw("VolkswagenHCAMode", b"x")
    self.raw("VolkswagenHCADeltaRateUp", b"99")
    self.assertEqual(HcaPreference(self.params).read()[0], hca_tuning.DEFAULT_HCA_MODE)
    self.assertEqual(hca_tuning.delta_rates(99, 0), (10, 10))


if __name__ == "__main__":
  unittest.main()
