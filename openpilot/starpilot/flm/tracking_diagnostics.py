"""StarPinguPilot: bounded counters for every place FLM drops or refuses data.

Report-only. Nothing here gates collection, tracking or control.

Each process (controlsd, card, steerlearnerd) keeps its own counters. Every
key also keeps the first and the latest example (small dict of the values at
that point). Once a minute the process writes
/data/media/0/flm_reports/diag-<process>-<boot>.json and a cloudlog
'flm_diag' event with the counts since the previous flush, so a drive shows
which check FLM sits on minute by minute.
"""

import json
import os
import sys
import time
from collections import Counter

MAX_KEYS = 256
MAX_SAMPLE_FIELDS = 24
FLUSH_NS = 60_000_000_000
DIRECTORY = '/data/media/0/flm_reports'

_counts: Counter = Counter()
_flushed: Counter = Counter()
_gauges: dict = {}
_first: dict = {}
_last: dict = {}
_next_flush = 0
_started = 0
_minutes: list = []


def _value(value):
  if isinstance(value, (bool, int)) or value is None:
    return value
  if isinstance(value, float):
    return round(value, 6)
  text = str(value)
  return text if len(text) <= 160 else text[:157] + '...'


def _sample(fields):
  return {str(k)[:48]: _value(v) for k, v in list(fields.items())[:MAX_SAMPLE_FIELDS]}


def count(group, reason='', /, **sample):
  try:
    key = f'{group}:{str(reason)[:96]}' if reason != '' else str(group)
    if key not in _counts and len(_counts) >= MAX_KEYS:
      key = 'overflow'
    _counts[key] += 1
    if sample:
      row = _sample(sample)
      row['t'] = time.monotonic_ns()
      if key not in _first:
        _first[key] = row
      _last[key] = row
    _maybe_flush()
  except Exception:
    pass


def transition(group, old, new):
  """Count a reason field change (old -> new)."""
  if old != new:
    count(group, new)


def maximum(name, value):
  try:
    if name in _gauges or len(_gauges) < 32:
      _gauges[name] = max(_gauges.get(name, value), value)
  except Exception:
    pass


def snapshot(limit=40):
  """Small copy for the learner report (its file is capped at 32 KB); full detail is in the diag file."""
  return {'counts': {key[:100]: value for key, value in _counts.most_common(limit)}, 'max': dict(_gauges), 'keys': len(_counts)}


def full_snapshot():
  return {'process': _process(), 'counts': dict(_counts.most_common(MAX_KEYS)), 'max': dict(_gauges),
          'firstExample': dict(_first), 'lastExample': dict(_last), 'perMinute': list(_minutes)}


def reset():
  global _next_flush, _started
  for value in (_counts, _flushed, _gauges, _first, _last):
    value.clear()
  _minutes.clear()
  _next_flush = _started = 0


def _process():
  try:
    from setproctitle import getproctitle
    name = getproctitle().split()[0].rsplit('.', 1)[-1].rsplit('/', 1)[-1]
  except Exception:
    name = os.path.basename(sys.argv[0] or 'python') if sys.argv else 'python'
  name = ''.join(c for c in (name[:-3] if name.endswith('.py') else name) if c.isalnum() or c in '_-')[:32] or 'python'
  return f'{name}-{os.getpid()}'


def _maybe_flush():
  global _next_flush, _started
  now = time.monotonic_ns()
  if not _started:
    _started = now
    _next_flush = now + FLUSH_NS
    return
  if now < _next_flush:
    return
  _next_flush = now + FLUSH_NS
  flush(now)


def flush(now=None):
  try:
    now = time.monotonic_ns() if now is None else now
    delta = {key: value - _flushed.get(key, 0) for key, value in _counts.items() if value != _flushed.get(key, 0)}
    _flushed.clear()
    _flushed.update(_counts)
    _minutes.append({'minute': round((now - _started) / 60e9, 1), 'counts': dict(sorted(delta.items(), key=lambda kv: -kv[1])[:48])})
    del _minutes[:-90]
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('flm_diag', process=_process(), counts=delta)
    except Exception:
      pass
    if os.path.isdir(DIRECTORY):
      boot = getattr(flush, 'boot', None)
      if boot is None:
        boot = flush.boot = time.strftime('%Y-%m-%d--%H-%M-%S')
      path = os.path.join(DIRECTORY, f'diag-{_process()}-{boot}.json')
      temporary = path + '.tmp'
      with open(temporary, 'w') as stream:
        json.dump(full_snapshot(), stream, separators=(',', ':'))
      os.replace(temporary, path)
      _prune()
  except Exception:
    pass


def _prune():
  try:
    files = sorted((os.path.join(DIRECTORY, name) for name in os.listdir(DIRECTORY) if name.startswith('diag-') and name.endswith('.json')),
                   key=os.path.getmtime)
    for old in files[:-60]:
      os.unlink(old)
  except OSError:
    pass


def traced_reason(group):
  """Class attribute: `reason = traced_reason('monitor')` counts every reason change."""
  def getter(self):
    return self.__dict__.get('_traced_reason')

  def setter(self, value):
    old = self.__dict__.get('_traced_reason', None)
    self.__dict__['_traced_reason'] = value
    if old != value:
      count(group, value)

  return property(getter, setter)
