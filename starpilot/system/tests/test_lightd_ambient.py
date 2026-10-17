from openpilot.starpilot.system.lightd.ambient import AmbientEstimator, CameraSample, REGIME_DAY, REGIME_NIGHT, scene_stops

DT = 0.05


def sample_for(t, target_b, k):
  """Camera sample that reads target_b (log2 lux), with AE converged to a mid grey like camerad."""
  lin = 0.3 / (10.0 - 9.0 * 0.3)
  ev = lin / 2.0 ** (target_b - k)
  ev_percent = (ev - 2.0) / (19992.0 - 2.0) * 100.0
  assert 0.0 <= ev_percent <= 100.0
  return CameraSample(t, 0.3, ev_percent)


def run(est, b_of_t, seconds, elev, v=15.0, t0=0.0):
  outs = []
  for i in range(int(round(seconds / DT))):
    t = t0 + i * DT
    s = sample_for(t, b_of_t(t), est.p.k_wide)
    outs.append(est.update(t, s, s, elev, v))
  return outs


def first(outs, pred):
  return next((i * DT for i, o in enumerate(outs) if pred(o)), None)


def daylight(est, seconds=60.0):
  return run(est, lambda t: 15.5, seconds, elev=40.0)


def test_scene_stops_monotonic():
  assert scene_stops(0.3, 10.0) > scene_stops(0.3, 20.0)  # more exposure needed -> darker
  assert scene_stops(0.4, 10.0) > scene_stops(0.3, 10.0)


def test_daylight_stays_light_and_valid():
  outs = daylight(AmbientEstimator())
  assert not any(o.dark_request for o in outs)
  assert outs[-1].valid and outs[-1].regime == REGIME_DAY
  assert abs(outs[-1].slow_b - 15.5) < 0.05


def test_not_valid_before_settle():
  assert not any(o.valid for o in daylight(AmbientEstimator(), 5.0))


def test_tunnel_turns_on_within_dwell():
  est = AmbientEstimator()
  daylight(est)
  outs = run(est, lambda t: 7.9, 20.0, elev=40.0, v=16.0, t0=60.0)  # logged tunnel: ~7.9 for 20 s
  t_on = first(outs, lambda o: o.dark_request)
  assert t_on is not None and t_on < 3.0
  assert outs[-1].dark_tunnel


def test_underpass_does_not_turn_on():
  est = AmbientEstimator()
  daylight(est)
  run(est, lambda t: 9.3, 1.0, elev=40.0, v=34.0, t0=60.0)  # logged overpasses: <1 s, <35 m, down to ~9-10
  outs = run(est, lambda t: 15.5, 5.0, elev=40.0, v=34.0, t0=61.0)
  assert not any(o.dark_request for o in outs)


def test_short_garage_at_crawl_turns_on_by_time():
  est = AmbientEstimator()
  daylight(est)
  outs = run(est, lambda t: 9.0, 5.0, elev=40.0, v=2.0, t0=60.0)
  t_on = first(outs, lambda o: o.dark_request)
  assert t_on is not None and t_on < 2.6


def test_daytime_shade_does_not_turn_on():
  est = AmbientEstimator()
  daylight(est)
  outs = run(est, lambda t: 11.0, 60.0, elev=40.0, t0=60.0)  # long deep shade, ~2000 lux
  assert not any(o.dark_request for o in outs)


def test_storm_turns_on_via_slow_channel():
  est = AmbientEstimator()
  daylight(est)
  # storm clouds roll in over a few minutes, down to ~700 lux
  outs = run(est, lambda t: max(15.5 - (t - 60.0) * 0.05, 9.4), 240.0, elev=30.0, t0=60.0)
  t_on = first(outs, lambda o: o.dark_request)
  assert t_on is not None and t_on > 100.0
  assert outs[-1].dark_slow and not outs[-1].dark_tunnel


def test_night_latch_ignores_bright_lights():
  est = AmbientEstimator()
  outs = run(est, lambda t: 12.0, 30.0, elev=-3.0)  # gas station canopy after sunset
  assert all(o.dark_request for o in outs)
  assert outs[-1].regime == REGIME_NIGHT and outs[-1].confidence == 1.0


def test_dusk_hysteresis():
  est = AmbientEstimator()
  on, hyst = est.p.on_threshold, est.p.hysteresis
  run(est, lambda t: on + 2.0, 100.0, elev=6.0)
  assert run(est, lambda t: on - 0.3, 100.0, elev=6.0, t0=100.0)[-1].dark_request
  # back above the threshold but inside the hysteresis band: stays on
  assert run(est, lambda t: on + hyst / 2, 100.0, elev=6.0, t0=200.0)[-1].dark_request
  assert not run(est, lambda t: on + hyst + 0.5, 100.0, elev=6.0, t0=300.0)[-1].dark_request


def test_sensitivity_shifts_threshold():
  early = AmbientEstimator()
  early.p.sensitivity = 1.0
  outs = run(early, lambda t: early.p.on_threshold + 0.5, 120.0, elev=6.0)
  assert outs[-1].dark_request
  assert not run(AmbientEstimator(), lambda t: 9.8 + 0.5, 120.0, elev=6.0)[-1].dark_request


def test_startup_ae_transient_is_ignored():
  # camerad starts reading ~14 at dusk and takes a few seconds to converge to ~10 (logged)
  est = AmbientEstimator()
  outs = run(est, lambda t: max(14.0 - t, 9.4), 20.0, elev=5.5)
  assert abs(outs[-1].slow_b - 9.4) < 0.05
  assert outs[-1].dark_request


def test_stale_camera_invalidates():
  est = AmbientEstimator()
  daylight(est, 5.0)
  old = CameraSample(4.9, 0.3, 50.0)
  out = est.update(6.0, old, old, 30.0)
  assert not out.valid
