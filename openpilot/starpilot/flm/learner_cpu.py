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
    # StarPinguPilot: core 7 alone starved it. On a comma 4 drive modeld (FIFO) and dmonitoringmodeld (FIFO) kept core 7
    # 89% busy, the receiver got ~60% of a core it needed more of, fell behind its 100 ms row limits and never launched a
    # trial, while an unthrottled replay of the same drive did. Let it also use the balanced cluster, at normal priority.
    mask = {0, 1, 2, 3, 7}
  if os.sched_getscheduler(0) != os.SCHED_OTHER:
    raise ValueError('optional learner requires ordinary scheduling')
  os.sched_setaffinity(0, mask)
  os.setpriority(os.PRIO_PROCESS, 0, 19 if numerical else 0)
  if numerical:
    os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
  return True
