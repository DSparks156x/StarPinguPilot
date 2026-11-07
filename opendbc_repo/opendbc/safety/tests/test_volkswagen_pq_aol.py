# StarPinguPilot: PQ stock-cruise always-on lateral (opendbc/safety/modes/volkswagen_pq_aol.h), Audi TT Mk2 bus layout.
import unittest

from opendbc.car import structs
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py

BUS = 1
TT_PARAM = VolkswagenSafetyFlags.PQ_NO_EXT_CAN | VolkswagenSafetyFlags.PQ_AOL


class TestVolkswagenPqAol(unittest.TestCase):
  def tearDown(self):
    self.safety.init_tests()
    self.safety.set_alternative_experience(0)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.noOutput, 0)

  def configure(self, param=TT_PARAM, experience=32):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.safety.set_alternative_experience(experience)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.volkswagenPq, int(param))
    self.safety.set_timer(1_000_000)
    self.safety.set_aol_test_heartbeat(True)
    self.packer = CANPackerSafety("vw_pq")
    self.time = 1_000_000
    self.counter = 0

  def packet(self, name, fields, bus=BUS):
    return self.packer.make_can_msg_safety(name, bus, fields)

  def feed(self, main=True, engaged=False, reverse=False, eps=3, cancel=False, brake=False, moving=True, omit=None,
           wrong_bus=None, request=1):
    self.time += 20_000
    self.counter = (self.counter + 1) % 16
    self.safety.set_timer(self.time)
    sources = (
      ("Bremse_1", {"BR1_Rad_kmh": 40 if moving else 0}),
      ("Lenkhilfe_3", {"LH3_LM": 0, "COUNTER": self.counter}),
      ("Lenkhilfe_2", {"LH2_Sta_HCA": eps}),
      ("Gate_Komf_1", {"GK1_Rueckfahr": int(reverse)}),
      ("Motor_3", {"MO3_Pedalwert": 0}),
      ("Motor_2", {"MO2_BLS": int(brake), "MO2_Sta_GRA": 1 if engaged else 0}),
      ("Motor_5", {"MO5_GRA_Hauptsch": int(main)}),
      ("GRA_Neu", {"GRA_Abbrechen": int(cancel), "COUNTER": self.counter}),
    )
    for name, fields in sources:
      if name != omit:
        self.safety.safety_rx_hook(self.packet(name, fields, 0 if name == wrong_bus else BUS))
    self.safety.safety_tick()
    self.safety.aol_set_host_request(request)

  def hca(self, torque=4, hca_status=5):
    return self.packet("HCA_1", {"LM_Offset": abs(torque), "LM_OffSign": torque < 0, "HCA_Status": hca_status})

  def arm(self):
    for _ in range(12):
      self.feed(main=False)
    self.feed()
    assert self.safety.aol_get_permission_mask() == 1

  def test_main_on_at_startup_never_arms(self):
    # PQ stalks latch: a main switch left on at startup must not start steering (StarPilot 6 behaviour).
    self.configure()
    for _ in range(30):
      self.feed()
      assert self.safety.aol_get_permission_mask() == 0
    assert not self.safety.safety_tx_hook(self.hca())

  def test_main_cycle_arms_lateral_only(self):
    self.configure()
    self.arm()
    assert not self.safety.get_controls_allowed()
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    assert self.safety.safety_tx_hook(self.hca())
    self.safety.aol_set_host_request(3)
    assert self.safety.aol_get_permission_mask() == 1  # never longitudinal
    self.feed(main=False)
    assert self.safety.aol_get_permission_mask() == 0
    self.feed()
    assert self.safety.aol_get_permission_mask() == 1  # the next main-on edge re-arms

  def test_stock_engagement_arms(self):
    self.configure()
    for _ in range(12):
      self.feed()
    assert self.safety.aol_get_permission_mask() == 0
    self.feed(engaged=True)
    self.feed(engaged=True)
    assert self.safety.aol_get_permission_mask() == 1
    self.feed()  # cancelling cruise keeps lateral, like StarPilot 6
    assert self.safety.aol_get_permission_mask() == 1

  def test_only_host_request_denies_while_armed(self):
    # StarPilot 6 conditions: stops, reverse and a busy EPS are the host's to pause. A denial reads as a disarm there.
    self.configure()
    self.arm()
    for kwargs in ({"moving": False}, {"reverse": True}, {"eps": 4}, {"eps": 1}, {"brake": True, "cancel": True}):
      self.feed(**kwargs)
      assert self.safety.aol_get_permission_mask() == 1, kwargs
    self.feed(request=0)
    assert self.safety.aol_get_permission_mask() == 0
    self.feed()
    assert self.safety.aol_get_permission_mask() == 1

  def test_main_cycle_without_heartbeat_arms(self):
    # 2026-11-06 drive: idle AOL has no heartbeat until the host sees the same edge, then one refused request disarmed it
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    for _ in range(12):
      self.feed(main=False, request=0)
    self.feed(request=0)
    assert self.safety.aol_get_permission_mask() == 0
    self.safety.set_aol_test_heartbeat(True)
    self.feed()
    assert self.safety.aol_get_permission_mask() == 1

  def test_stock_engagement_before_heartbeat_arms(self):
    # 2026-11-06 drive: cruise engages first, the host heartbeat follows; the arm must survive that gap
    self.configure()
    self.safety.set_aol_test_heartbeat(False)
    for _ in range(12):
      self.feed(request=0)
    for _ in range(5):
      self.feed(engaged=True, request=0)
    assert self.safety.get_controls_allowed()
    self.safety.set_aol_test_heartbeat(True)
    self.feed(engaged=True)
    assert self.safety.aol_get_permission_mask() == 1

  def test_heartbeat_gap_keeps_arm(self):
    self.configure()
    self.arm()
    self.safety.set_aol_test_heartbeat(False)
    self.feed()
    assert self.safety.aol_get_permission_mask() == 0
    self.safety.set_aol_test_heartbeat(True)
    self.feed()
    assert self.safety.aol_get_permission_mask() == 1

  def test_eps_fault_is_the_hosts_to_pause(self):
    # StarPilot 6 conditions, and the host and panda arms cannot disagree after the rack recovers
    for eps in (0, 2):
      self.configure()
      self.arm()
      self.feed(eps=eps)
      assert self.safety.aol_get_permission_mask() == 1

  def test_engaged_cruise_arms_after_rx_recovers(self):
    self.configure()
    for _ in range(12):
      self.feed()
    self.feed(engaged=True)
    self.feed(engaged=True)
    assert self.safety.aol_get_permission_mask() == 1
    self.feed(main=False, engaged=True)
    assert self.safety.aol_get_permission_mask() == 0

  def test_each_source_missing_or_wrong_bus_expires(self):
    for name in ("Motor_5",):
      for kwargs in ({"omit": name}, {"wrong_bus": name}):
        self.configure()
        self.arm()
        for _ in range(20):
          self.feed(**kwargs)
        assert self.safety.aol_get_permission_mask() == 0, (name, kwargs)

  def test_host_request_timeout(self):
    self.configure()
    self.arm()
    self.time += 400_000
    self.safety.set_timer(self.time)
    assert self.safety.aol_get_permission_mask() == 0

  def test_mismatched_configuration_sends_nothing(self):
    for param, experience in ((TT_PARAM, 0), (VolkswagenSafetyFlags.PQ_NO_EXT_CAN, 32),
                              (TT_PARAM | VolkswagenSafetyFlags.LONG_CONTROL, 32)):
      self.configure(param, experience)
      self.safety.set_controls_allowed(True)
      assert not self.safety.safety_tx_hook(self.hca()), (param, experience)

  def test_without_aol_stock_behaviour_unchanged(self):
    self.configure(VolkswagenSafetyFlags.PQ_NO_EXT_CAN, 0)
    self.safety.set_controls_allowed(True)
    self.safety.set_desired_torque_last(0)
    self.safety.set_rt_torque_last(0)
    assert self.safety.safety_tx_hook(self.hca())


if __name__ == "__main__":
  unittest.main()
