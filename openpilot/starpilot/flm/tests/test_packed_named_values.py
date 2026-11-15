"""StarPinguPilot: packed named values (LearningControlEvidence.packedNamedValues)."""
import math
import struct

import pytest

from openpilot.cereal import custom
from openpilot.starpilot.flm import learning_wire as wire


SNAPSHOT = (('k', 1.5), ('neg_zero', -0.0), ('flag', True), ('off', False), ('label', 'abc'))
INPUTS = (('speed', 12.25), ('active', True))
STAGE = (('setpoint', -0.0), ('jerk', 3.0))


def test_round_trip_keeps_exact_values_and_kinds():
  snapshot, inputs, stage = wire.unpack_named_values(wire.pack_named_values(SNAPSHOT, INPUTS, STAGE))
  assert tuple(snapshot) == SNAPSHOT and tuple(inputs) == INPUTS and tuple(stage) == STAGE
  assert [type(v) for _, v in snapshot] == [float, float, bool, bool, str]
  assert struct.pack('!d', snapshot[1][1]) == struct.pack('!d', -0.0)
  assert math.copysign(1.0, stage[0][1]) < 0


def test_empty_sections_round_trip():
  assert wire.unpack_named_values(wire.pack_named_values((), (), ())) == ([], [], [])


@pytest.mark.parametrize('mutate', [
  lambda b: b'XXXX' + b[4:],          # magic
  lambda b: b[:-1],                   # truncated
  lambda b: b + b'\x00',              # trailing bytes
  lambda b: b[:10] + b'\xff' + b[11:],  # header count beyond bounds/layout
])
def test_malformed_blob_is_rejected(mutate):
  blob = wire.pack_named_values(SNAPSHOT, INPUTS, STAGE)
  with pytest.raises(ValueError):
    wire.unpack_named_values(mutate(blob))


def test_names_with_nul_are_rejected():
  with pytest.raises(ValueError):
    wire.pack_named_values((('a\x00b', 1.0),), (), ())


def _listed(snapshot, inputs, stage):
  row = custom.LearningControlEvidence.new_message(
    replaySnapshot=wire._named_values(snapshot), replayInputs=wire._named_values(inputs),
    stageValues=[{'name': n, 'value': v} for n, v in stage])
  return row.as_reader()


def test_packed_and_listed_decode_identically():
  row = _listed(SNAPSHOT, INPUTS, STAGE)
  snapshot, inputs, stage = wire.unpack_named_values(wire.pack_named_values(SNAPSHOT, INPUTS, STAGE))
  assert wire._checked_pairs(snapshot, 1024) == wire._read_named_values(row.replaySnapshot, 1024)
  assert wire._checked_pairs(inputs, wire.EXTENDED_INPUTS) == wire._read_named_values(row.replayInputs, wire.EXTENDED_INPUTS)
  assert tuple(wire._shared_scalar_pair(n, v) for n, v in stage) == \
    tuple(wire._shared_scalar_pair(str(v.name), float(v.value)) for v in row.stageValues)


def test_snapshot_count_reads_both_layouts():
  listed = _listed(SNAPSHOT, INPUTS, STAGE)
  assert wire.replay_snapshot_count(listed) == len(SNAPSHOT)
  packed = custom.LearningControlEvidence.new_message(
    packedNamedValues=wire.pack_named_values(SNAPSHOT, INPUTS, STAGE)).as_reader()
  assert wire.replay_snapshot_count(packed) == len(SNAPSHOT)
