#!/usr/bin/env python3
"""StarPinguPilot: switch to a higher-priority saved Wi-Fi network once it is in range.

NetworkManager only uses connection.autoconnect-priority when it picks a network from scratch; it never leaves a
working connection for a better one. A comma that joined a weaker fallback network (e.g. the building Wi-Fi in the
parking lot) therefore keeps it after being carried home. Offroad, this checks every 30 s whether a saved network
with a higher priority than the active one is visible, and activates it. It also joins the best visible saved network
when Wi-Fi has been down for two checks, since NetworkManager stops retrying a network after a few failures (a fringe
signal from the car), and it asks for a fresh scan, since NetworkManager rarely rescans while connected.
"""
import os
import subprocess
import time
from dataclasses import dataclass

from openpilot.common.swaglog import cloudlog

CHECK_PERIOD_S = 30.
MIN_SIGNAL = 20          # nmcli percent; don't hop to a network at the edge of range
FAILURE_BACKOFF_S = 600.  # after a failed switch, leave that network alone for a while
NMCLI_TIMEOUT_S = 20.
RESCAN_PERIOD_S = 120.   # while not on the best saved network


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
           backoff_until: dict[str, float], now: float, disconnected_long: bool = False) -> WifiConnection | None:
  """The saved network to switch to, or None. Leaves an active infrastructure connection for a higher priority one,
  never the comma's own hotspot. With Wi-Fi down for a while (disconnected_long), joins the best visible one."""
  by_uuid = {c.uuid: c for c in connections}
  active = by_uuid.get(active_uuid) if active_uuid else None
  if active is None:
    if active_uuid is not None or not disconnected_long:
      return None  # an unsaved connection, or NetworkManager may still be connecting
    floor = -1 << 31
  elif not active.infrastructure:
    return None
  else:
    floor = active.priority
  candidates = [c for c in connections
                if c.autoconnect and c.infrastructure and c.priority > floor and
                visible.get(c.ssid, -1) >= MIN_SIGNAL and backoff_until.get(c.uuid, 0.) <= now]
  return max(candidates, key=lambda c: (c.priority, visible[c.ssid]), default=None)


def nmcli(*args: str) -> str:
  return subprocess.run(["nmcli", *args], capture_output=True, text=True, check=True, timeout=NMCLI_TIMEOUT_S).stdout


class NetworkManagerState:
  def __init__(self):
    self._details: dict[str, tuple[str, bool]] = {}  # uuid -> (ssid, infrastructure), fixed per saved connection

  def read(self, rescan: bool = False) -> tuple[list[WifiConnection], str | None, dict[str, int]]:
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
    for line in nmcli("-t", "-f", "SSID,SIGNAL", "device", "wifi", "list", "--rescan", "yes" if rescan else "auto").splitlines():
      fields = split_terse(line)
      if len(fields) == 2 and fields[0] and fields[1].isdigit():
        visible[fields[0]] = max(visible.get(fields[0], 0), int(fields[1]))
    return connections, active_uuid, visible


def describe(connections: list[WifiConnection], active_uuid: str | None, visible: dict[str, int]) -> str:
  active = next((c.ssid for c in connections if c.uuid == active_uuid), None if active_uuid is None else "unsaved")
  seen = ", ".join(f"{c.ssid!r} {visible[c.ssid]}%" for c in sorted(connections, key=lambda c: -c.priority)
                   if c.infrastructure and c.ssid in visible)
  return f"on {active!r}, saved in range: {seen or 'none'}"


def main() -> None:
  os.nice(19)
  state = NetworkManagerState()
  backoff_until: dict[str, float] = {}
  last_rescan = -RESCAN_PERIOD_S
  was_disconnected = False
  last_active: str | None = ""
  on_best = False
  while True:
    try:
      now = time.monotonic()
      rescan = not on_best and now - last_rescan >= RESCAN_PERIOD_S
      if rescan:
        last_rescan = now
      connections, active_uuid, visible = state.read(rescan)
      best = max((c.priority for c in connections if c.autoconnect and c.infrastructure), default=None)
      on_best = any(c.uuid == active_uuid and c.priority == best for c in connections)
      if active_uuid != last_active:
        cloudlog.warning(f"wifi_preference: {describe(connections, active_uuid, visible)}")
        last_active = active_uuid
      disconnected = active_uuid is None
      target = choose(connections, active_uuid, visible, backoff_until, now, disconnected and was_disconnected)
      was_disconnected = disconnected
      if target is not None:
        state_now = describe(connections, active_uuid, visible)
        cloudlog.warning(f"wifi_preference: joining {target.ssid!r} (priority {target.priority}), {state_now}")
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
