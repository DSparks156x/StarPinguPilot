# StarPinguPilot: comma camera lights for the Audi TT Mk2, sent to the J519 in the factory HBA frame mFLA_1 (0x65A).
#
# Two independent producers share the frame (CentElecRE/artifacts/comma_strategy_signal_contract.md and
# claude_handoff_06_finalize_byte0.md):
#  - low beam: valid / headlight_on / brightness 0..7 in the matrix-void byte 0, read by the patched J519 in place of
#    G397. Needs a byte-0 J519 image: the reviewed SGO ACCC6C5C...21B1A still reads byte 4 and must not be used with this.
#  - HBA: the stock FLA fields (FL1_FLA_Aufblenden + FL1_Fernlicht_PWM). Not covered by the low-beam valid bit,
#    so on HBA expiry both are cleared explicitly.
# One sender for 0x65A (the VW carcontroller), 10 Hz, neutral content while a producer is unavailable.
from dataclasses import dataclass

from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

# VolkswagenLightdMode param index
LIGHTD_OFF, LIGHTD_SHADOW, LIGHTD_LOW_BEAM, LIGHTD_LOW_AND_HIGH_BEAM = range(4)
LIGHTD_SETTINGS = {
  "VolkswagenLightdMode": ("Camera Lights", ("Off", "Shadow (log only)", "Low Beam", "Low + High Beam")),
}

# panda safety param bit allowing mFLA_1 TX, must match FLAG_VOLKSWAGEN_PQ_LIGHT_TX in safety/modes/volkswagen_pq.h
PQ_LIGHT_TX = VolkswagenSafetyFlags.PQ_LIGHT_TX.value

FLA_STEP = 10  # 10 Hz at the 100 Hz carcontroller rate; matrix cycle is 1000 ms + on change, gateway timeout 5000 ms

# FL1_Abblendursache bits
REASON_FAULT = 255

# low-beam carrier, zero-based byte 0 (void in the K-matrix)
LIGHT_BYTE = 0
LIGHT_VALID = 0x80
LIGHT_ON = 0x08
LIGHT_BRIGHTNESS_MASK = 0x07  # bits 4..6 reserved, always zero

# OEM interior-mirror (ISP) fields we would be the only sender of. The TT has a standalone compass mirror, not a
# CAN one. These say "no ISP mirror": data fields at their matrix init values, active flags 0.
# OPEN QUESTION: nobody has checked whether the cluster, RNS-E or gateway reacts to these when mFLA_1 starts
# appearing. If something complains, try the K-matrix init values for EC/compass active (1), or feed real data.
ISP_DEFAULTS = {
  "FL1_ISP_Lichtwert_vo": 62,      # init
  "FL1_ISP_Lichtwert_hi": 62,      # init
  "FL1_ISP_Himmelsrichtung": 30,   # init
  "FL1_ISP_Taste": 0,
  "FL1_ISP_EC_aktiv": 0,
  "FL1_ISP_Kompass_aktiv": 0,
  "FL1_Dimmwert_ASP": 0,
}


@dataclass(frozen=True)
class LightCommand:
  valid: bool = False
  headlight_on: bool = False
  brightness: int = 0  # 0..7, low = dark (coming/leaving-home eligibility only, does not switch the low beam)

  def __post_init__(self):
    if not isinstance(self.valid, bool) or not isinstance(self.headlight_on, bool):
      raise ValueError(f"LightCommand flags must be bool: {self}")
    if isinstance(self.brightness, bool) or not isinstance(self.brightness, int) or not 0 <= self.brightness <= 7:
      raise ValueError(f"LightCommand brightness must be an int 0..7: {self}")


@dataclass(frozen=True)
class HbaCommand:
  active: bool = False        # HBA producer enabled and fresh
  permitted: bool = False     # scene allows high beam
  reason: int = REASON_FAULT  # FL1_Abblendursache
  sensor_blocked: bool = False


LIGHT_INVALID = LightCommand()
HBA_OFF = HbaCommand()


def fla_values(hba: HbaCommand) -> dict[str, int]:
  """OEM mFLA_1 signal values. FL1_ISP_defekt is never set: it latches a sticky fault in the J519 until power cycle."""
  raise_beam = hba.active and hba.permitted
  values = {
    "FL1_KD_Fehler": 0,
    "FL1_FLA_aktiv": int(hba.active),
    "FL1_ISP_defekt": 0,
    "FL1_Sensor_blockiert": int(hba.active and hba.sensor_blocked),
    "FL1_Abblendursache": 0 if raise_beam else (hba.reason if hba.active else REASON_FAULT),
    # binary policy: request 100 % with the qualifier, 0 + no qualifier otherwise. Never 127 (the J519 doubles it).
    "FL1_Fernlicht_PWM": 100 if raise_beam else 0,
    "FL1_FLA_Aufblenden": int(raise_beam),
  }
  values.update(ISP_DEFAULTS)
  return values


def light_byte(light: LightCommand) -> int:
  """Byte 0: 0x80 valid | 0x08 low beam requested | brightness 0..7. Anything not valid is 0x00."""
  if not light.valid:
    return 0x00
  return LIGHT_VALID | (LIGHT_ON if light.headlight_on else 0) | (light.brightness & LIGHT_BRIGHTNESS_MASK)


def apply_light_carrier(dat: bytearray, light: LightCommand) -> bytearray:
  """Put the low-beam command into byte 0 of a packed 8-byte frame, leaving bytes 1..7 untouched."""
  dat[LIGHT_BYTE] = light_byte(light)
  return dat


def create_fla_1(packer, bus: int, light: LightCommand, hba: HbaCommand):
  addr, dat, bus = packer.make_can_msg("mFLA_1", bus, fla_values(hba))
  if len(dat) != 8:  # the J519 discards short frames
    return addr, dat, bus
  return addr, bytes(apply_light_carrier(bytearray(dat), light)), bus
