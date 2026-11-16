"""StarPinguPilot: generic StarPilot torque must not hold a low-speed limit cycle on the TT's rack.

The plant is the TT near centre at 1-10 mph, measured from drives 00000025/26: torque behaves like a steering-rate
command (~200 deg/s per unit torque, 20 ms lag) behind ~0.1 s of loop delay. With friction fed the low-speed-scaled
error, the controller held a 2 Hz, ~9 deg peak-to-peak wobble here, matching the car.
"""

from types import SimpleNamespace
import math

import numpy as np
import pytest
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.starpilot.lateral.controller_selection import ControllerMode

KV, TAU, DELAY_FRAMES = 200.0, 0.02, 10


def closed_loop(speed, desired_deg, start_deg, *, scaled_friction=False, undamped=False):
  cp = interfaces['AUDI_TT_MK2'].get_non_essential_params('AUDI_TT_MK2')
  ci = interfaces['AUDI_TT_MK2'](cp)
  controller = LatControlTorque(cp.as_reader(), ci, 0.01, controller_mode=ControllerMode.STARPILOT)
  controller.update_torque_parameters(1.77, -0.066, 0.13)
  if scaled_friction:
    controller.starpilot_extension.policy.friction_error_context = lambda scaled, physical, speed: scaled
  if undamped:
    controller.starpilot_extension.policy.error_rate_context = lambda rate, kp, lsf, speed: 0.0
  vm = VehicleModel(cp)
  params = log.VehicleParameters.new_message(angleOffsetDeg=0.0, roll=0.0)
  rng = np.random.default_rng(0)
  angle, rate = start_deg, 0.0
  commands = [0.0] * (DELAY_FRAMES + 1)
  angles = []
  for desired in desired_deg:
    measured = round((angle + rng.normal(0.0, 0.15)) / 0.15) * 0.15
    cs = SimpleNamespace(vEgo=speed, steeringAngleDeg=measured, steeringPressed=False)
    curvature = -vm.calc_curvature(math.radians(desired), speed, 0.0)
    output, _, _ = controller.update(True, cs, vm, params, False, curvature, False, 0.28)
    assert math.isfinite(output)
    commands.append(output)
    rate += (KV * commands[-1 - DELAY_FRAMES] - rate) * 0.01 / TAU
    angle += rate * 0.01
    angles.append(angle)
  return np.array(angles)


def hold_peak_to_peak(speed, **kwargs):
  return float(np.ptp(closed_loop(speed, [0.0] * 1200, 8.0, **kwargs)[400:]))


def turn_exit_overshoot(speed, **kwargs):
  # Desired wheel angle unwinds 250 -> 0 deg at 250 deg/s, then holds straight.
  desired = np.r_[np.full(50, 250.0), np.linspace(250.0, 0.0, 100), np.zeros(350)]
  angles = closed_loop(speed, desired, 250.0, **kwargs)
  return max(0.0, -float(angles[150:].min()))


@pytest.mark.parametrize('speed', [1.0, 2.2, 4.5])
def test_tt_holds_centre_at_low_speed(speed):
  assert hold_peak_to_peak(speed) < 1.5


def test_plant_reproduces_scaled_friction_wobble():
  assert hold_peak_to_peak(2.2, scaled_friction=True) > 5.0


@pytest.mark.parametrize('speed', [1.5, 3.5])
def test_rate_damping_cuts_turn_exit_overshoot(speed):
  # Drives 00000028/29: the wheel ran ~10+ deg past centre coming out of low-speed turns.
  undamped = turn_exit_overshoot(speed, undamped=True)
  assert undamped > 10.0
  assert turn_exit_overshoot(speed) < 0.6 * undamped
