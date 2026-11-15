"""CPU placement of the optional receiver and its bounded numerical child."""
import os
from pathlib import Path
from openpilot.common.hardware import HARDWARE


def configure_cpu(*, numerical=False):
  # These devices isolate cores6/7 from ordinary load balancing. The receiver
  # otherwise migrates among busy cores0..5 while its100Hz tape queues up.
  # Model/DM FIFO owners preempt the CFS receiver. Numerical work stays on
  # the background cluster, below its owners. No other process is moved.
  if HARDWARE.get_device_type() not in ('tici', 'tizi', 'mici'):
    return False
  if numerical:
    # Kernel affinity intersects online/allowed CPUs; this never enables one.
    mask = {0, 1, 2, 3}
  else:
    if 7 not in os.sched_getaffinity(0):
      return False
    try:
      online = Path('/sys/devices/system/cpu/cpu7/online').read_text().strip()
    except OSError:
      return False
    if online != '1':
      return False
    # StarPinguPilot: big core 7 only, at normal priority. Cores 6/7 are isolated from load balancing, so a {0-3, 7} mask
    # left the receiver on the little 1.8 GHz cores 0-3 (61-82% busy while engaged), roughly 3x slower per cycle than
    # the 2.8 GHz big core. On drive 00000025 core 7 averaged 23% busy with the learner off it: the earlier "89% busy"
    # included the learner itself. modeld/dmonitoringmodeld stay FIFO and always preempt it.
    mask = {7}
  if os.sched_getscheduler(0) != os.SCHED_OTHER:
    raise ValueError('optional learner requires ordinary scheduling')
  os.sched_setaffinity(0, mask)
  os.setpriority(os.PRIO_PROCESS, 0, 19 if numerical else 0)
  if numerical:
    os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
  return True
