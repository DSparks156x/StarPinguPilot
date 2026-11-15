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


def hold_peak_to_peak(speed, *, scaled_friction=False):
  cp = interfaces['AUDI_TT_MK2'].get_non_essential_params('AUDI_TT_MK2')
  ci = interfaces['AUDI_TT_MK2'](cp)
  controller = LatControlTorque(cp.as_reader(), ci, 0.01, controller_mode=ControllerMode.STARPILOT)
  controller.update_torque_parameters(1.77, -0.066, 0.13)
  if scaled_friction:
    controller.starpilot_extension.policy.friction_error_context = lambda scaled, physical, speed: scaled
  vm = VehicleModel(cp)
  params = log.VehicleParameters.new_message(angleOffsetDeg=0.0, roll=0.0)
  rng = np.random.default_rng(0)
  angle, rate = 8.0, 0.0
  commands = [0.0] * (DELAY_FRAMES + 1)
  angles = []
  for frame in range(1200):
    measured = round((angle + rng.normal(0.0, 0.15)) / 0.15) * 0.15
    cs = SimpleNamespace(vEgo=speed, steeringAngleDeg=measured, steeringPressed=False)
    output, _, _ = controller.update(True, cs, vm, params, False, 0.0, False, 0.28)
    assert math.isfinite(output)
    commands.append(output)
    rate += (KV * commands[-1 - DELAY_FRAMES] - rate) * 0.01 / TAU
    angle += rate * 0.01
    if frame >= 400:
      angles.append(angle)
  return float(np.ptp(angles))


@pytest.mark.parametrize('speed', [1.0, 2.2, 4.5])
def test_tt_holds_centre_at_low_speed(speed):
  assert hold_peak_to_peak(speed) < 1.5


def test_plant_reproduces_scaled_friction_wobble():
  assert hold_peak_to_peak(2.2, scaled_friction=True) > 5.0
