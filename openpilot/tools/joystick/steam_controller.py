#!/usr/bin/env python3
"""Userspace reader for the (2026) Valve Steam Controller, aka "Triton".

The comma's 4.9 kernel has no driver for it, so on its own the controller only
shows up in "lizard mode" as a keyboard + mouse with no joystick axes. This
talks to it over /dev/hidraw instead: it turns lizard mode off with a feature
report and decodes the raw state reports. Protocol follows SDL's
src/joystick/hidapi/SDL_hidapi_steam_triton.c.

Works over Bluetooth (28de:1303), USB (28de:1302) and the wireless dongles
(28de:1304 / 28de:1305).
"""
import fcntl
import os
import select
import struct
import time
from pathlib import Path

VALVE_VID = 0x28DE
TRITON_PIDS = {0x1302, 0x1303, 0x1304, 0x1305}  # USB, BLE, Proteus dongle, Nereid dongle

READ_SIZE = 64
ID_STATE = 0x42
ID_STATE_BLE = 0x45
ID_STATE_TIMESTAMP = 0x47
ID_WIRELESS_STATUS_X = 0x46
ID_WIRELESS_STATUS = 0x79
WIRELESS_DISCONNECT = 1
WIRELESS_CONNECT = 2
STATE_IDS = {ID_STATE, ID_STATE_BLE, ID_STATE_TIMESTAMP}

FEATURE_REPORT_BYTES = 64  # including the leading report id 1
ID_SET_SETTINGS_VALUES = 0x87
SETTING_LIZARD_MODE = 9
LIZARD_MODE_OFF = 0
LIZARD_RESEND_INTERVAL = 3.0  # the firmware falls back to lizard mode otherwise; SDL resends every 3 s

BUTTON_Y = 0x00000008

# seq u8, buttons u32, triggers L/R s16, left stick X/Y s16, right stick X/Y s16
STATE = struct.Struct('<BIhhhhhh')

STALE_TIMEOUT = 0.3  # state streams continuously (~130 Hz over BLE); zero outputs if it stops


def _hidiocsfeature(length: int) -> int:
  # _IOC(_IOC_WRITE | _IOC_READ, 'H', 0x06, length)
  return (3 << 30) | (length << 16) | (ord('H') << 8) | 0x06


def find_hidraw_paths(sysfs_root: str = '/sys/class/hidraw') -> list[str]:
  """hidraw nodes of connected Steam Controllers that carry controller state."""
  paths = []
  for node in sorted(Path(sysfs_root).glob('hidraw*')):
    try:
      uevent = (node / 'device' / 'uevent').read_text()
      rdesc = (node / 'device' / 'report_descriptor').read_bytes()
    except OSError:
      continue
    hid_id = next((line.split('=', 1)[1] for line in uevent.splitlines() if line.startswith('HID_ID=')), '')
    try:
      _bus, vid, pid = (int(v, 16) for v in hid_id.split(':'))
    except ValueError:
      continue
    # dongles expose extra interfaces; only the controller slots declare state report ids
    if vid == VALVE_VID and pid in TRITON_PIDS and any(bytes([0x85, rid]) in rdesc for rid in STATE_IDS):
      paths.append(f'/dev/{node.name}')
  return paths


class SteamControllerState:
  def __init__(self):
    self.connected = False
    self.reset()

  def reset(self):
    self.buttons = 0
    self.left_trigger = self.right_trigger = 0  # 0..32767
    self.left_x = self.left_y = self.right_x = self.right_y = 0  # s16, up and right positive

  @property
  def y_button(self) -> bool:
    return bool(self.buttons & BUTTON_Y)

  def handle_report(self, data: bytes) -> bool:
    if not data:
      return False
    report_id = data[0]
    if report_id in (ID_WIRELESS_STATUS, ID_WIRELESS_STATUS_X) and len(data) >= 2:
      if data[1] == WIRELESS_DISCONNECT:
        self.connected = False
        self.reset()
      elif data[1] == WIRELESS_CONNECT:
        self.connected = True
      return True
    if report_id not in STATE_IDS or len(data) < 1 + STATE.size:
      return False
    (_seq, self.buttons, self.left_trigger, self.right_trigger,
     self.left_x, self.left_y, self.right_x, self.right_y) = STATE.unpack_from(data, 1)
    self.connected = True
    return True


class SteamControllerReader:
  def __init__(self):
    self.state = SteamControllerState()
    self.fds: dict[int, str] = {}
    self.last_report = 0.
    self.last_lizard_update = 0.

  @property
  def is_open(self) -> bool:
    return bool(self.fds)

  def close(self):
    for fd in self.fds:
      try:
        os.close(fd)
      except OSError:
        pass
    self.fds = {}
    self.state.connected = False
    self.state.reset()

  def open(self) -> bool:
    if self.fds:
      return True
    for path in find_hidraw_paths():
      try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
      except PermissionError:
        # hidraw nodes are root-only on AGNOS; comma has passwordless sudo
        os.system(f'sudo chmod a+rw {path}')
        try:
          fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
          continue
      except OSError:
        continue
      self.fds[fd] = path
    self.last_report = time.monotonic()
    self.last_lizard_update = 0.
    return bool(self.fds)

  def _disable_lizard_mode(self):
    buf = bytearray(FEATURE_REPORT_BYTES)
    buf[0:6] = bytes([1, ID_SET_SETTINGS_VALUES, 3, SETTING_LIZARD_MODE]) + struct.pack('<H', LIZARD_MODE_OFF)
    for fd in self.fds:
      try:
        fcntl.ioctl(fd, _hidiocsfeature(len(buf)), buf)
      except OSError:
        pass  # dongle slot without an awake controller

  def poll(self, timeout: float = 0.1) -> bool:
    """Read pending reports. Returns True if state changed."""
    if not self.fds:
      return False
    now = time.monotonic()
    if now - self.last_lizard_update >= LIZARD_RESEND_INTERVAL:
      self._disable_lizard_mode()
      self.last_lizard_update = now

    try:
      readable, _, _ = select.select(list(self.fds), [], [], timeout)
    except OSError:
      self.close()
      return False

    handled = False
    for fd in readable:
      while True:
        try:
          data = os.read(fd, READ_SIZE)
        except BlockingIOError:
          break
        except OSError:
          self.close()  # unpaired / unplugged
          return False
        if not data:
          break
        if self.state.handle_report(data):
          handled = True
          if data[0] in STATE_IDS:
            self.last_report = time.monotonic()

    if time.monotonic() - self.last_report > STALE_TIMEOUT:
      self.state.reset()
    return handled


if __name__ == '__main__':
  print('Steam Controller interfaces:', find_hidraw_paths() or '<none>')
  reader = SteamControllerReader()
  if not reader.open():
    raise SystemExit('No Steam Controller found (paired / dongle plugged in?)')
  last_print = 0.
  while True:
    reader.poll()
    if time.monotonic() - last_print > 0.1:
      last_print = time.monotonic()
      s = reader.state
      print(f'connected={int(s.connected)} LX={s.left_x:6d} LY={s.left_y:6d} RX={s.right_x:6d} RY={s.right_y:6d} '
            f'LT={s.left_trigger:5d} RT={s.right_trigger:5d} buttons={s.buttons:08x}', flush=True)
