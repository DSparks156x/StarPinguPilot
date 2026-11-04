#!/usr/bin/env python3
"""StarPinguPilot: switch to a higher-priority saved Wi-Fi network once it is in range.

NetworkManager only uses connection.autoconnect-priority when it picks a network from scratch; it never leaves a
working connection for a better one. A comma that joined a weaker fallback network (e.g. the building Wi-Fi in the
parking lot) therefore keeps it after being carried home. Offroad, this checks every 30 s whether a saved network
with a higher priority than the active one is visible, and activates it.
"""
import os
import subprocess
import time
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog

CHECK_PERIOD_S = 30.
MIN_SIGNAL = 30          # nmcli percent; don't hop to a network at the edge of range
FAILURE_BACKOFF_S = 600.  # after a failed switch, leave that network alone for a while
NMCLI_TIMEOUT_S = 20.


@dataclass(frozen=True)
class WifiConnection:
  uuid: str
  name: str
  ssid: str
  priority: int
  autoconnect: bool
  infrastructure: bool  # False for the comma's own hotspot (access point mode)


def split_terse(line: str) -> list[str]:
  """Split one `nmcli -t` line on unescaped ':' and unescape '\\:' and '\\\\'."""
  fields, cur, escaped = [], [], False
  for ch in line:
    if escaped:
      cur.append(ch)
      escaped = False
    elif ch == '\\':
      escaped = True
    elif ch == ':':
      fields.append(''.join(cur))
      cur = []
    else:
      cur.append(ch)
  fields.append(''.join(cur))
  return fields


def choose(connections: list[WifiConnection], active_uuid: str | None, visible: dict[str, int],
           backoff_until: dict[str, float], now: float) -> WifiConnection | None:
  """The saved network to switch to, or None. Only leaves an active infrastructure connection, never a hotspot,
  and never acts while disconnected (NetworkManager's own autoconnect handles that)."""
  by_uuid = {c.uuid: c for c in connections}
  active = by_uuid.get(active_uuid) if active_uuid else None
  if active is None or not active.infrastructure:
    return None
  candidates = [c for c in connections
                if c.autoconnect and c.infrastructure and c.priority > active.priority and
                visible.get(c.ssid, -1) >= MIN_SIGNAL and backoff_until.get(c.uuid, 0.) <= now]
  return max(candidates, key=lambda c: (c.priority, visible[c.ssid]), default=None)


def nmcli(*args: str) -> str:
  return subprocess.run(["nmcli", *args], capture_output=True, text=True, check=True, timeout=NMCLI_TIMEOUT_S).stdout


class NetworkManagerState:
  def __init__(self):
    self._details: dict[str, tuple[str, bool]] = {}  # uuid -> (ssid, infrastructure), fixed per saved connection

  def read(self) -> tuple[list[WifiConnection], str | None, dict[str, int]]:
    connections, active_uuid = [], None
    for line in nmcli("-t", "-f", "NAME,UUID,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY,ACTIVE", "connection", "show").splitlines():
      fields = split_terse(line)
      if len(fields) != 6 or fields[2] != "802-11-wireless":
        continue
      name, uuid, _, autoconnect, priority, active = fields
      if uuid not in self._details:
        ssid, mode = (split_terse(nmcli("-t", "-g", "802-11-wireless.ssid,802-11-wireless.mode", "connection", "show",
                                        "uuid", uuid).strip()) + [""])[:2]
        self._details[uuid] = (ssid, mode in ("", "infrastructure"))
      ssid, infrastructure = self._details[uuid]
      connections.append(WifiConnection(uuid, name, ssid, int(priority or 0), autoconnect == "yes", infrastructure))
      if active == "yes":
        active_uuid = uuid

    visible: dict[str, int] = {}
    for line in nmcli("-t", "-f", "SSID,SIGNAL", "device", "wifi", "list").splitlines():
      fields = split_terse(line)
      if len(fields) == 2 and fields[0] and fields[1].isdigit():
        visible[fields[0]] = max(visible.get(fields[0], 0), int(fields[1]))
    return connections, active_uuid, visible


def main() -> None:
  os.nice(19)
  state = NetworkManagerState()
  backoff_until: dict[str, float] = {}
  while True:
    try:
      connections, active_uuid, visible = state.read()
      target = choose(connections, active_uuid, visible, backoff_until, time.monotonic())
      if target is not None:
        active = next(c for c in connections if c.uuid == active_uuid)
        cloudlog.warning(f"wifi_preference: switching from {active.ssid!r} (priority {active.priority}) to "
                         f"{target.ssid!r} (priority {target.priority}, signal {visible[target.ssid]})")
        try:
          nmcli("connection", "up", "uuid", target.uuid)
        except subprocess.SubprocessError:
          cloudlog.exception(f"wifi_preference: failed to activate {target.ssid!r}")
          backoff_until[target.uuid] = time.monotonic() + FAILURE_BACKOFF_S
    except Exception:
      cloudlog.exception("wifi_preference: check failed")
    time.sleep(CHECK_PERIOD_S)


if __name__ == "__main__":
  main()
