"""Optional bounded owner hooks. No Params/I/O/fit or controller mutation."""

from dataclasses import replace
from openpilot.starpilot.flm.fullrate_collector import ControlEvidence, AppliedEvidence
from openpilot.starpilot.flm.learning_wire import write_control, write_applied


def control_message(message, owner, cc, *, control_ns, state_ns, limited):
  """Actual adapter owns identity and stage snapshot; no default fallback model."""
  adapter = getattr(owner, 'automatic_flm_adapter', None)
  if adapter is None:
    return
  try:
    evidence = adapter.capture_evidence(control_ns=int(control_ns), state_ns=int(state_ns), active=bool(cc.latActive), limited=bool(limited))
    if not isinstance(evidence, ControlEvidence):
      return
    write_control(message.starpilotLateralState, evidence)
    return True
  except (AttributeError, TypeError, ValueError, OverflowError) as error:
    from openpilot.starpilot.flm.tracking_diagnostics import count as diag_count
    diag_count('controls_write_error', error)
    # Caller discards its copy after any failed write, including partial fields.
    return False


def _domain(cp):
  if str(cp.steerControlType) == 'angle':
    return 'angle'
  if str(cp.steerControlType) == 'curvature':
    return 'curvature'
  return 'normalized_torque'


def begin_applied_frame(owner):
  owner.learning_previous_applied_evidence = getattr(owner, 'learning_applied_evidence', None)
  owner.learning_applied_evidence = None


def applied_snapshot(owner, cc, cs, output, *, apply_ns, state_ns):
  """Only a real emitted steering command creates a new source/time proof."""
  try:
    cp, sm = owner.CP, owner.sm
    domain = _domain(cp)
    value = float(output.steeringAngleDeg if domain == 'angle' else output.curvature if domain == 'curvature' else output.torque)
    valid = bool(sm.valid['carControl'] and sm.alive['carControl'] and sm.seen['carControl'])
    fault = bool(not cs.canValid or cs.canTimeout or cs.steerFaultTemporary or cs.steerFaultPermanent or cs.accFaulted)
    override = bool(
      cs.steeringPressed or cs.brakePressed or cs.regenBraking or any(str(button.type) == 'cancel' for button in cs.buttonEvents)
    )
    native_allowed = bool(sm.seen['pandaStates'] and sm.alive['pandaStates'] and sm.valid['pandaStates'])
    projection = None
    if domain == 'normalized_torque':
      controller = owner.CI.CC
      if getattr(controller, 'learning_projection_observer', None) is None:
        return None
      projection = getattr(controller, 'learning_projection', None)
      if projection is None:
        from openpilot.starpilot.flm.tracking_diagnostics import count as diag_count
        previous = getattr(owner, 'learning_previous_applied_evidence', None)
        # No CAN emission means no new command/time. Current health cannot
        # renew the immutable previous command or its original native lease.
        if (
          previous is not None
          and previous.domain == domain
          and valid
          and cc.latActive
          and not fault
          and not override
          and 0 <= apply_ns - previous.apply_ns <= 100_000_000
        ):
          # Only carrier metadata advances. The original command/application,
          # state source, codec witness and native permission remain unchanged.
          return replace(previous, carrier_frame=int(controller.frame), carrier_observed_ns=int(apply_ns))
        if cc.latActive:
          diag_count('card_applied_none', 'no_projection_and_no_carry', previous=previous is not None, valid=valid, fault=fault,
                     override=override, gap_ms=(apply_ns - previous.apply_ns) / 1e6 if previous is not None else None)
        return None
      from opendbc.car.steering_projection import inspect_projection

      inspected = inspect_projection(owner.CI, cc, output, projection)
      limited = inspected is not True
      if cc.latActive:
        from openpilot.starpilot.flm.tracking_diagnostics import count as diag_count
        diag_count('card_applied', 'projection_exact' if inspected is True else 'projection_limited' if inspected is False else 'projection_unknown',
                   requested=getattr(projection, 'requested_torque', None), final_raw=getattr(projection, 'final_raw', None),
                   unlimited_raw=getattr(projection, 'unlimited_raw', None), override=override, fault=fault)
    else:
      # These domains require their own actual-emission projection provider;
      # a generic CI.apply call cannot claim a new wire command on hold ticks.
      return None
    return AppliedEvidence(
      int(sm.logMonoTime['carControl']),
      int(state_ns),
      int(apply_ns),
      domain,
      value,
      valid,
      bool(cc.latActive),
      override,
      fault,
      limited,
      native_allowed,
      projection,
      carrier_frame=int(controller.frame),
      carrier_observed_ns=int(apply_ns),
    )
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as error:
    from openpilot.starpilot.flm.tracking_diagnostics import count as diag_count
    diag_count('card_applied_error', error)
    return None


def applied_message(message, evidence):
  if evidence is not None:
    try:
      write_applied(message.starpilotCarState, evidence)
      return True
    except (AttributeError, TypeError, ValueError, OverflowError):
      return False
