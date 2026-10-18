import os
import subprocess
import sys

from openpilot.common.basedir import BASEDIR


def test_services_h_matches_services_py():
  # StarPinguPilot: services.h is committed, and loggerd only logs what it lists. A stale copy silently dropped
  # starpilotLightState from every rlog.
  out = subprocess.check_output([sys.executable, os.path.join(BASEDIR, "cereal", "services.py")], text=True)
  with open(os.path.join(BASEDIR, "cereal", "services.h")) as f:
    assert f.read().replace("\r\n", "\n") == out.replace("\r\n", "\n"), "regenerate: python3 cereal/services.py > cereal/services.h"
