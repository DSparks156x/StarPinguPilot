"""Shared StarPilot torque shaping using the vehicle's own calibration and conversion."""

from openpilot.common.scalar_clip import clip_scalar
from openpilot.starpilot.lateral.controller_selection import generic_torque_supported
from openpilot.starpilot.lateral.hkg_torque_policy import HKGShapedTorquePolicy
from openpilot.starpilot.lateral.torque_shaping import get_standard_friction_threshold


class GenericTorquePolicy(HKGShapedTorquePolicy):
  supported_cp = staticmethod(generic_torque_supported)

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
