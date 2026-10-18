from cereal import log
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, GreyBigButton
from openpilot.selfdrive.ui.ui_state import ui_state


class PingulinkDaemonButton(GreyBigButton):
  def __init__(self):
    super().__init__("pingulink status", "")
    self._params = Params()
    self._last_state = None

  def _update_state(self):
    super()._update_state()
    onroad = ui_state.ignition
    wifi = (ui_state.sm["deviceState"].networkType == log.DeviceState.NetworkType.wifi)
    
    # Mode determination
    mode = "Fast" if (onroad or wifi) else "Slow"
      
    url = self._params.get("PingulinkUrl") or "None"
    
    # Queue counts
    counts = self._params.get("PingulinkPendingCount")
    if counts:
      try:
        pending, auto = counts.split(",")
        queue_str = f"Queue: {pending} Req, {auto} Auto"
      except ValueError:
        queue_str = "Queue: Error"
    else:
      queue_str = "Queue: Idle"
      
    state_str = f"Srv: {url}\nMode: {mode} | {queue_str}"
    
    if state_str != self._last_state:
      self._last_state = state_str
      self.set_value(state_str)


class PingulinkStatusButton(GreyBigButton):
  def __init__(self):
    super().__init__("connection diagnostics", "")
    self._params = Params()
    self._last_ping = None
    self._last_status = None

  def _update_state(self):
    super()._update_state()
    last_ping = self._params.get("PingulinkLastPing") or "Never"
    last_status = self._params.get("PingulinkLastPingStatus") or "Unknown"
    
    # Shorten status for MICI screen
    if "Success" in last_status:
      status_short = "OK"
    elif "Failed" in last_status:
      status_short = last_status.replace("Failed ", "Err ")
    else:
      status_short = last_status[:10]
      
    state_str = f"Last: {last_ping}\nResult: {status_short}"
    
    if state_str != self._last_ping or last_status != self._last_status:
      self._last_ping = state_str
      self._last_status = last_status
      self.set_value(state_str)


class PingulinkLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

    # Active toggles
    enable_pingulink = BigParamControl("enable pingulink", "PingulinkEnable")

    self._scroller.add_widgets([
      enable_pingulink,
      PingulinkDaemonButton(),
      PingulinkStatusButton(),
    ])

    self._refresh_toggles = (
      ("PingulinkEnable", enable_pingulink),
    )

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
