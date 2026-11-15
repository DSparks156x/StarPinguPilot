"""StarPinguPilot: sampling profiler for steerlearnerd (report-only).

While engaged the learner publishes its 20 Hz tracking status at ~15 Hz, which
controls refuses for trials. This samples the learner's main thread every 5 ms
from a daemon thread and records:
  - where the main thread is (innermost frame, and the deepest steerlearnerd.py line)
  - garbage collector pauses
  - main loop iteration durations
and writes /data/media/0/flm_reports/profile-steerlearnerd-<boot>.json once a minute.
Nothing here changes what the learner does.
"""

import gc
import json
import os
import sys
import threading
import time
from collections import Counter

DIRECTORY = '/data/media/0/flm_reports'
SAMPLE_S = 0.005
WRITE_S = 60.0
LOOP_BINS_MS = (5, 10, 20, 35, 50, 75, 100, 150, 250, 500, 1000)

_state = {'started': False}


def _key(frame):
  code = frame.f_code
  return f'{os.path.basename(code.co_filename)}:{code.co_name}:{frame.f_lineno}'


class _Profile:
  def __init__(self):
    self.main_id = threading.main_thread().ident
    self.inner = Counter()
    self.learner_line = Counter()
    self.files = Counter()
    self.samples = 0
    self.minute = Counter()
    self.minutes = []
    self.gc_start = 0.0
    self.gc = Counter()
    self.gc_max_ms = 0.0
    self.loop_last = 0.0
    self.loop_hist = Counter()
    self.loop_max_ms = 0.0
    self.loops = 0
    self.boot = time.strftime('%Y-%m-%d--%H-%M-%S')
    self.started = time.monotonic()

  def gc_callback(self, phase, info):
    now = time.monotonic()
    if phase == 'start':
      self.gc_start = now
    elif self.gc_start:
      ms = (now - self.gc_start) * 1e3
      generation = info.get('generation', -1)
      self.gc[f'gen{generation}_count'] += 1
      self.gc[f'gen{generation}_ms'] += ms
      self.gc_max_ms = max(self.gc_max_ms, ms)
      self.gc_start = 0.0

  def loop(self):
    now = time.monotonic()
    if self.loop_last:
      ms = (now - self.loop_last) * 1e3
      self.loops += 1
      self.loop_max_ms = max(self.loop_max_ms, ms)
      label = next((f'<{b}' for b in LOOP_BINS_MS if ms < b), f'>={LOOP_BINS_MS[-1]}')
      self.loop_hist[label] += 1
    self.loop_last = now

  def sample(self):
    frame = sys._current_frames().get(self.main_id)
    if frame is None:
      return
    self.samples += 1
    self.inner[_key(frame)] += 1
    learner = None
    seen_files = set()
    depth = 0
    while frame is not None and depth < 80:
      name = os.path.basename(frame.f_code.co_filename)
      if learner is None and name == 'steerlearnerd.py':
        learner = f'{frame.f_code.co_name}:{frame.f_lineno}'
      seen_files.add(name)
      frame = frame.f_back
      depth += 1
    if learner is not None:
      self.learner_line[learner] += 1
      self.minute[learner] += 1
    for name in seen_files:
      self.files[name] += 1

  def snapshot(self):
    total = max(1, self.samples)
    share = lambda counter, n: {k: round(100.0 * v / total, 2) for k, v in counter.most_common(n)}
    return {
      'samples': self.samples, 'sampleMs': SAMPLE_S * 1e3, 'uptimeS': round(time.monotonic() - self.started, 1),
      'percentByLearnerLine': share(self.learner_line, 40),
      'percentInnermost': share(self.inner, 40),
      'percentByFileOnStack': share(self.files, 40),
      'gc': {k: round(v, 1) for k, v in self.gc.items()}, 'gcMaxMs': round(self.gc_max_ms, 1),
      'loops': self.loops, 'loopMaxMs': round(self.loop_max_ms, 1), 'loopMsHistogram': dict(self.loop_hist),
      'perMinuteTopLearnerLines': self.minutes[-60:],
    }

  def write(self):
    self.minutes.append({'minute': round((time.monotonic() - self.started) / 60, 1),
                         'top': dict(self.minute.most_common(12))})
    self.minute = Counter()
    if not os.path.isdir(DIRECTORY):
      return
    path = os.path.join(DIRECTORY, f'profile-steerlearnerd-{self.boot}.json')
    with open(path + '.tmp', 'w') as stream:
      json.dump(self.snapshot(), stream, separators=(',', ':'))
    os.replace(path + '.tmp', path)


def _run(profile):
  next_write = time.monotonic() + WRITE_S
  while True:
    time.sleep(SAMPLE_S)
    try:
      profile.sample()
      if time.monotonic() >= next_write:
        next_write += WRITE_S
        profile.write()
    except Exception:
      pass


def start():
  if _state['started']:
    return
  try:
    profile = _Profile()
    gc.callbacks.append(profile.gc_callback)
    threading.Thread(target=_run, args=(profile,), name='flm-profile', daemon=True).start()
    _state['profile'] = profile
    _state['started'] = True
  except Exception:
    pass


def loop():
  profile = _state.get('profile')
  if profile is not None:
    profile.loop()
