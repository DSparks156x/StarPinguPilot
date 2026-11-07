from openpilot.starpilot.system.wifi_preference import FAILURE_BACKOFF_S, MIN_SIGNAL, WifiConnection, choose, parse_ssid_mode, split_terse

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
  assert choose(SAVED, None, visible, {}, 0.) is None  # NetworkManager may still be connecting
  assert choose(SAVED, "u-unsaved", visible, {}, 0.) is None


def test_joins_best_network_after_wifi_stays_down():
  # NetworkManager stops retrying after a few failures, e.g. a fringe signal from the car
  visible = {"Stadium Walk-UF104": 45, "Social at Stadium Walk": 90}
  assert choose(SAVED, None, visible, {}, 0., disconnected_long=True) == HOME
  assert choose(SAVED, None, {"Social at Stadium Walk": 90}, {}, 0., disconnected_long=True) == SOCIAL
  assert choose(SAVED, None, {"weedle-de6b": 90}, {}, 0., disconnected_long=True) is None
  assert choose(SAVED, None, {}, {}, 0., disconnected_long=True) is None


def test_weak_home_still_beats_fallback():
  assert choose(SAVED, SOCIAL.uuid, {"Stadium Walk-UF104": 25, "Social at Stadium Walk": 90}, {}, 0.) == HOME


def test_picks_highest_priority_and_backs_off_after_failure():
  visible = {"Stadium Walk-UF104": 40, "Social at Stadium Walk": 80, "Hotspot": 90}
  assert choose(SAVED, PHONE.uuid, visible, {}, 0.) == HOME
  backoff = {HOME.uuid: FAILURE_BACKOFF_S}
  assert choose(SAVED, PHONE.uuid, visible, backoff, 1.) == SOCIAL
  assert choose(SAVED, PHONE.uuid, visible, backoff, FAILURE_BACKOFF_S) == HOME


def test_parse_ssid_mode_from_comma_output():
  # 2026-11-07 on the comma: one value per line. Reading it as one ':' line gave 'Stadium Walk-UF104\ninfrastructure',
  # which never matched the scan, so the helper never switched.
  assert parse_ssid_mode("Stadium Walk-UF104\ninfrastructure\n") == ("Stadium Walk-UF104", True)
  assert parse_ssid_mode("Social at Stadium Walk\ninfrastructure\n") == ("Social at Stadium Walk", True)
  assert parse_ssid_mode("weedle-de6b\nap\n") == ("weedle-de6b", False)
  assert parse_ssid_mode("a\\:b\n\n") == ("a:b", True)


def test_split_terse_unescapes():
  assert split_terse(r"a\:b:c\\d:") == ["a:b", "c\\d", ""]
