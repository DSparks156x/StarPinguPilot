"""StarPinguPilot: bounded counters for where live tracking stops.

Report-only. Nothing here gates collection, tracking or control. The learner
report copies a snapshot once a minute so a drive shows which check the
tracking pipeline actually sits on (monitor resets, pose pairing, episode
windows, actor gates, optimizer interrupts).
"""

from collections import Counter

MAX_KEYS = 96
_counts: Counter = Counter()
_gauges: dict = {}


def count(group, reason=''):
  try:
    key = f'{group}:{str(reason)[:80]}' if reason != '' else str(group)
    if key in _counts or len(_counts) < MAX_KEYS:
      _counts[key] += 1
    else:
      _counts['overflow'] += 1
  except Exception:
    pass


def maximum(name, value):
  try:
    if name in _gauges or len(_gauges) < 16:
      _gauges[name] = max(_gauges.get(name, value), value)
  except Exception:
    pass


def snapshot():
  return {'counts': dict(_counts.most_common(MAX_KEYS)), 'max': dict(_gauges)}


def reset():
  _counts.clear()
  _gauges.clear()
