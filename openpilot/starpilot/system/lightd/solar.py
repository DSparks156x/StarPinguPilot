"""Solar position from lat/lon + UTC (NOAA solar calculator equations). StarPinguPilot.

Accurate to roughly 0.01 deg for current dates, no network or ephemeris files needed.
Refraction is applied, so elevation matches what a light sensor sees near the horizon.
"""
import math


def julian_day(unix_s: float) -> float:
  return unix_s / 86400.0 + 2440587.5


def solar_position(lat_deg: float, lon_deg: float, unix_s: float) -> tuple[float, float]:
  """Returns (elevation_deg, azimuth_deg) of the sun, azimuth clockwise from north."""
  jc = (julian_day(unix_s) - 2451545.0) / 36525.0

  geom_mean_long = (280.46646 + jc * (36000.76983 + jc * 0.0003032)) % 360.0
  geom_mean_anom = 357.52911 + jc * (35999.05029 - 0.0001537 * jc)
  ecc = 0.016708634 - jc * (0.000042037 + 0.0000001267 * jc)
  m = math.radians(geom_mean_anom)
  eq_ctr = (math.sin(m) * (1.914602 - jc * (0.004817 + 0.000014 * jc)) +
            math.sin(2 * m) * (0.019993 - 0.000101 * jc) + math.sin(3 * m) * 0.000289)
  true_long = geom_mean_long + eq_ctr
  omega = math.radians(125.04 - 1934.136 * jc)
  app_long = math.radians(true_long - 0.00569 - 0.00478 * math.sin(omega))

  mean_obliq = 23.0 + (26.0 + (21.448 - jc * (46.815 + jc * (0.00059 - jc * 0.001813))) / 60.0) / 60.0
  obliq = math.radians(mean_obliq + 0.00256 * math.cos(omega))
  decl = math.asin(math.sin(obliq) * math.sin(app_long))

  y = math.tan(obliq / 2) ** 2
  l0 = math.radians(geom_mean_long)
  eq_time = 4 * math.degrees(y * math.sin(2 * l0) - 2 * ecc * math.sin(m) + 4 * ecc * y * math.sin(m) * math.cos(2 * l0) -
                             0.5 * y * y * math.sin(4 * l0) - 1.25 * ecc * ecc * math.sin(2 * m))  # minutes

  minutes_utc = (unix_s % 86400.0) / 60.0
  true_solar_time = (minutes_utc + eq_time + 4 * lon_deg) % 1440.0
  hour_angle = true_solar_time / 4 - 180.0 if true_solar_time >= 0 else true_solar_time / 4 + 180.0
  ha = math.radians(hour_angle)

  lat = math.radians(lat_deg)
  cos_zen = math.sin(lat) * math.sin(decl) + math.cos(lat) * math.cos(decl) * math.cos(ha)
  zen = math.acos(max(-1.0, min(1.0, cos_zen)))
  elev = 90.0 - math.degrees(zen)

  # atmospheric refraction
  if elev > 85.0:
    refr = 0.0
  else:
    te = math.tan(math.radians(elev))
    if elev > 5.0:
      refr = 58.1 / te - 0.07 / te ** 3 + 0.000086 / te ** 5
    elif elev > -0.575:
      refr = 1735.0 + elev * (-518.2 + elev * (103.4 + elev * (-12.79 + elev * 0.711)))
    else:
      refr = -20.772 / te
  elev += refr / 3600.0

  sin_zen = math.sin(zen)
  if abs(sin_zen) < 1e-9:
    az = 180.0 if lat_deg > 0 else 0.0
  else:
    cos_az = (math.sin(lat) * math.cos(zen) - math.sin(decl)) / (math.cos(lat) * sin_zen)
    az_raw = math.degrees(math.acos(max(-1.0, min(1.0, cos_az))))
    az = (az_raw + 180.0) % 360.0 if hour_angle > 0 else (540.0 - az_raw) % 360.0

  return elev, az
