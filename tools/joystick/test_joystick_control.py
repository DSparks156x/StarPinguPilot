from openpilot.tools.joystick import joystick_control


def test_evdev_axes_are_normalized_for_joystick_mode():
  joystick = joystick_control.Joystick.__new__(joystick_control.Joystick)
  joystick.axes_values = {"ABS_RY": 0.0, "ABS_X": 0.0}
  joystick.min_axis_value = {"ABS_RY": 0.0, "ABS_X": 0.0}
  joystick.max_axis_value = {"ABS_RY": 255.0, "ABS_X": 255.0}
  joystick.deadzone = 0.1
  joystick.cancel = False

  assert joystick._handle_event(joystick_control.EV_ABS, joystick_control.ABS_CODES["ABS_X"], 0)
  assert joystick.axes_values["ABS_X"] == 1.0
  assert joystick._handle_event(joystick_control.EV_ABS, joystick_control.ABS_CODES["ABS_RY"], 255)
  assert joystick.axes_values["ABS_RY"] == -1.0
  assert joystick._handle_event(joystick_control.EV_ABS, joystick_control.ABS_CODES["ABS_X"], 128)
  assert joystick.axes_values["ABS_X"] == 0.0


def test_evdev_ignores_unconfigured_axes_and_tracks_cancel_button():
  joystick = joystick_control.Joystick.__new__(joystick_control.Joystick)
  joystick.axes_values = {"ABS_RY": 0.0, "ABS_X": 0.0}
  joystick.min_axis_value = {"ABS_RY": 0.0, "ABS_X": 0.0}
  joystick.max_axis_value = {"ABS_RY": 255.0, "ABS_X": 255.0}
  joystick.deadzone = 0.1
  joystick.cancel = False

  assert not joystick._handle_event(joystick_control.EV_ABS, joystick_control.ABS_CODES["ABS_Z"], 255)
  assert joystick._handle_event(joystick_control.EV_KEY, joystick_control.BTN_NORTH, 1)
  assert joystick.cancel
  assert joystick._handle_event(joystick_control.EV_KEY, joystick_control.BTN_NORTH, 0)
  assert not joystick.cancel



def _steam_report(lx=0, ly=0, rx=0, ry=0, lt=0, rt=0, buttons=0, report_id=0x45):
  import struct
  from openpilot.tools.joystick import steam_controller as sc
  data = bytearray(46)
  data[0] = report_id
  sc.STATE.pack_into(data, 1, 7, buttons, lt, rt, lx, ly, rx, ry)
  return bytes(data)


def test_steam_report_decoding():
  from openpilot.tools.joystick import steam_controller as sc
  state = sc.SteamControllerState()

  assert state.handle_report(_steam_report(lx=-32768, ry=32767, lt=10, buttons=sc.BUTTON_Y))
  assert (state.left_x, state.right_y, state.left_trigger, state.y_button) == (-32768, 32767, 10, True)
  assert not state.handle_report(bytes([0x40, 0, 0, 0, 0, 0]))  # lizard-mode mouse report

  # dongle reports the controller lost: everything zeroes
  assert state.handle_report(bytes([sc.ID_WIRELESS_STATUS, sc.WIRELESS_DISCONNECT]))
  assert not state.connected and state.left_x == 0 and state.right_y == 0


def test_steam_axes_map_to_joystick_mode():
  joystick = joystick_control.Joystick.__new__(joystick_control.Joystick)
  joystick.is_pc = False
  joystick._load_profile('Steam')
  reader = joystick_control.SteamControllerReader()
  reader.fds = {-1: '/dev/hidraw0'}
  reader.poll = lambda: True
  joystick._steam = reader

  reader.state.handle_report(_steam_report(lx=-32767, ry=32767))
  joystick._update_steam()
  assert joystick.axes_values == {'ABS_RY': 1.0, 'ABS_X': 1.0}  # full gas, full left

  reader.state.handle_report(_steam_report(lx=1000, ry=-32767, buttons=0x08))
  joystick._update_steam()
  assert joystick.axes_values == {'ABS_RY': -1.0, 'ABS_X': 0.0}  # full brake, stick inside deadzone
  assert joystick.cancel
