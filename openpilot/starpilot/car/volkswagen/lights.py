"""StarPinguPilot: card side of lightd for the Audi TT Mk2 (camera low beam + high beam assist via mFLA_1 0x65A).

lightd -> semantic J519 commands, packed by opendbc/car/volkswagen/fla.py in the VW carcontroller. Low beam and HBA
lapse independently: a stale or missing lightd message revokes both, lightd's own validity flags revoke each one.
The mode is re-read every second, so lowering it mid-drive takes effect immediately.
"""
from dataclasses import asdict

import openpilot.cereal.messaging as messaging
from opendbc.car import structs
from opendbc.car.volkswagen import fla
from openpilot.starpilot.car.volkswagen.pq_preferences import READ_PERIOD_FRAMES, lightd_mode


def light_tx_latched(cp) -> bool:
  """The panda only allows mFLA_1 when the light TX bit was latched at car start."""
  return any(cfg.safetyModel == structs.CarParams.SafetyModel.volkswagenPq and cfg.safetyParam & fla.PQ_LIGHT_TX
             for cfg in cp.safetyConfigs)


class LightCommandSource:
  def __init__(self, params):
    self.params = params
    self.sm = messaging.SubMaster(['starpilotLightState'])
    self.frames = 0
    self.mode = lightd_mode(params)

  def update(self, CS) -> None:
    if self.frames % READ_PERIOD_FRAMES == 0:
      self.mode = lightd_mode(self.params)
    self.frames += 1
    self.sm.update(0)

    light, hba = fla.LIGHT_INVALID, fla.HBA_OFF
    if self.sm.alive['starpilotLightState'] and self.sm.valid['starpilotLightState']:
      ls = self.sm['starpilotLightState']
      # an out-of-range code from lightd means the estimate is not trustworthy: send invalid, never crash card
      if self.mode >= fla.LIGHTD_LOW_BEAM and ls.lowBeamValid and 0 <= ls.brightnessCode <= 7:
        light = fla.LightCommand(valid=True, headlight_on=bool(ls.darkRequest), brightness=int(ls.brightnessCode))
      if self.mode >= fla.LIGHTD_LOW_AND_HIGH_BEAM and ls.hbaValid:
        hba = fla.HbaCommand(active=True, permitted=ls.highBeamPermitted, reason=ls.hbaReason,
                             sensor_blocked=ls.sensorBlocked)
    CS.fla_command = (light, hba)


def compose(CS, companion, *, valid: bool, car_state_stamp: int):
  """Add the TT light state to this frame's starpilotCarState companion (lightd input)."""
  lights = getattr(CS, 'light_state', None)
  if lights is None:
    return companion
  if companion is None:
    companion = messaging.new_message('starpilotCarState', valid=valid, logMonoTime=car_state_stamp)
  companion.starpilotCarState.sourceCarStateMonoTime = car_state_stamp
  for name, value in asdict(lights).items():
    setattr(companion.starpilotCarState, name, value)
  return companion
