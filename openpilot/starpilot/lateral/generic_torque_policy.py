"""Shared StarPilot torque shaping using the vehicle's own calibration and conversion."""

import numpy as np

from openpilot.common.scalar_clip import clip_scalar
from openpilot.starpilot.lateral.controller_selection import generic_torque_supported
from openpilot.starpilot.lateral.hkg_torque_policy import HKGShapedTorquePolicy
from openpilot.starpilot.lateral.torque_shaping import get_standard_friction_threshold

# StarPinguPilot: derivative time on the measured lateral-acceleration rate, scaled like P so the damping is a fixed
# fraction of P in wheel-angle terms (P is ~0.09 torque/deg of wheel at every speed). On the TT the low-speed loop
# (rate-actuator rack, ~0.1 s delay) peaked ~1.7x near 1-1.4 Hz and overshot ~12 deg out of turns (drives 00000028/29).
# In closed-loop sim through the policy's own 2 Hz rate filter, 0.04 s halves that overshoot and the peak drops to
# ~1.4x; beyond ~0.06 s measurement noise starts moving the wheel at hold. Faded out above town speeds, where the
# plant was not identified.
DAMPING_TIME = 0.04
DAMPING_FADE_BP = [7.5, 15.0]


class GenericTorquePolicy(HKGShapedTorquePolicy):
  supported_cp = staticmethod(generic_torque_supported)

  def __init__(self, parent, cp):
    super().__init__(parent, cp)
    # Unit D gain: error_rate_context returns the damping already in lateral-acceleration units.
    parent.pid._k_d = ([0.0], [1.0])

  def error_rate_context(self, rate, kp, lsf, speed):
    return -rate * (kp + lsf) * DAMPING_TIME * float(np.interp(speed, DAMPING_FADE_BP, [1.0, 0.0]))

  def feedforward(self, value, setpoint, jerk, speed):
    return value

  def friction(self, setpoint, jerk, speed):
    return get_standard_friction_threshold(speed), 1.0

  def friction_error_context(self, scaled_error, physical_error, speed):
    # StarPinguPilot: friction sees the physical lateral-acceleration error, as in the standard controller.
    # The low-speed factor multiplies the error ~48x at 5 mph, which turned friction into a +-relay
    # around centre. With ~0.1 s of loop delay on a rack that behaves like a rate actuator at low speed,
    # that relay held a 2 Hz, +-10 deg wheel limit cycle on the TT (drives 00000025/26). P keeps the
    # scaled error.
    return physical_error

  def output(self, value, setpoint, speed):
    return clip_scalar(value, -self.parent.steer_max, self.parent.steer_max)
