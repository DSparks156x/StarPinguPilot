from openpilot.starpilot.system.wifi_preference import FAILURE_BACKOFF_S, MIN_SIGNAL, WifiConnection, choose, split_terse

HOME = WifiConnection("u-home", "openpilot connection Stadium Walk-UF104", "Stadium Walk-UF104", 10, True, True)
SOCIAL = WifiConnection("u-social", "openpilot connection Social at Stadium Walk", "Social at Stadium Walk", 5, True, True)
PHONE = WifiConnection("u-phone", "openpilot connection Hotspot", "Hotspot", 0, True, True)
TETHER = WifiConnection("u-tether", "Hotspot", "weedle-de6b", 0, False, False)
SAVED = [HOME, SOCIAL, PHONE, TETHER]


def test_switches_home_when_carried_inside():
  visible = {"Stadium Walk-UF104": 67, "Social at Stadium Walk": 37}
  assert choose(SAVED, SOCIAL.uuid, visible, {}, 0.) == HOME


def test_keeps_fallback_when_home_out_of_range():
  assert choose(SAVED, SOCIAL.uuid, {"Social at Stadium Walk": 37}, {}, 0.) is None
  assert choose(SAVED, SOCIAL.uuid, {"Stadium Walk-UF104": MIN_SIGNAL - 1}, {}, 0.) is None


def test_never_leaves_best_network_or_hotspot_or_disconnected_state():
  visible = {"Stadium Walk-UF104": 67, "Social at Stadium Walk": 90}
  assert choose(SAVED, HOME.uuid, visible, {}, 0.) is None
  assert choose(SAVED, TETHER.uuid, visible, {}, 0.) is None
  assert choose(SAVED, None, visible, {}, 0.) is None


def test_picks_highest_priority_and_backs_off_after_failure():
  visible = {"Stadium Walk-UF104": 40, "Social at Stadium Walk": 80, "Hotspot": 90}
  assert choose(SAVED, PHONE.uuid, visible, {}, 0.) == HOME
  backoff = {HOME.uuid: FAILURE_BACKOFF_S}
  assert choose(SAVED, PHONE.uuid, visible, backoff, 1.) == SOCIAL
  assert choose(SAVED, PHONE.uuid, visible, backoff, FAILURE_BACKOFF_S) == HOME


def test_split_terse_unescapes():
  assert split_terse(r"a\:b:c\\d:") == ["a:b", "c\\d", ""]
