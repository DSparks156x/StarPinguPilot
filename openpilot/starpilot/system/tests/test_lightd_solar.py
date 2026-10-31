from datetime import UTC, datetime

import pytest

from openpilot.starpilot.system.lightd.solar import solar_position


def ts(*args):
  return datetime(*args, tzinfo=UTC).timestamp()


def test_solstice_noon_elevation():
  # Greenwich, June solstice: max elevation = 90 - lat + 23.44
  noon = ts(2024, 6, 20, 11)
  best = max(solar_position(40.0, 0.0, noon + s)[0] for s in range(0, 7200, 60))
  assert best == pytest.approx(90 - 40 + 23.44, abs=0.1)


def test_london_sunrise():
  # London sunrise 2024-06-21 is 03:43 UTC: refracted center ~ -0.27 deg (upper limb on horizon)
  elev, az = solar_position(51.5074, -0.1278, ts(2024, 6, 21, 3, 43))
  assert elev == pytest.approx(-0.27, abs=0.25)
  assert 40 < az < 55  # rises in the north-east


def test_night_and_azimuth_quadrants():
  elev, _ = solar_position(51.5074, -0.1278, ts(2024, 12, 21, 0, 0))
  assert elev < -50
  _, az_morning = solar_position(40.0, -105.0, ts(2024, 3, 20, 15, 0))  # 08:00 local
  _, az_evening = solar_position(40.0, -105.0, ts(2024, 3, 20, 23, 0))  # 16:00 local
  assert 90 < az_morning < 135
  assert 225 < az_evening < 270
