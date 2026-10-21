import json
import random
import time
import pyray as rl
from cereal import car, custom, log
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app, ASSETS_DIR
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus

# Sprites and the animation index come from pingu_sprites/pack_sprites.py
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
ThermalStatus = log.DeviceState.ThermalStatus
ConfidenceClass = log.ModelDataV2.ConfidenceClass
AlertStatus = log.SelfdriveState.AlertStatus
DMAlertLevel = log.DriverMonitoringState.AlertLevel
GearShifter = car.CarState.GearShifter
StarPilotEventName = custom.StarPilotOnroadEvent.EventName
GO_EVENTS = (StarPilotEventName.greenLight, StarPilotEventName.leadDeparting)

FPS = 6
STOPPED_SPEED = 0.1      # m/s
SLEEP_AFTER = 30.        # s stopped before Pingu dozes off
TAILGATE_GAP = 0.8       # s time gap to the lead that earns a "noot noot"
TAILGATE_MIN_SPEED = 5.  # m/s
HARD_BRAKE = -3.         # m/s^2
CURVE_ACCEL = 2.         # m/s^2 lateral before Pingu leans into it
HIGHWAY_SPEED = 29.      # m/s, ~65 mph
SPEEDING_MARGIN = 1.1    # over the mapd limit, plus 1 m/s
HOLD_TIME = 2.           # s to keep short events on screen so they don't flicker
IDLE_EXTRA_EVERY = (15., 40.)
IDLE_EXTRAS = ("look_around", "giggle", "fishing", "fish_snack", "snowman", "juggle")


def _lat_saturated(controls_state) -> bool:
  lat = controls_state.lateralControlState
  try:
    return bool(getattr(lat, lat.which()).saturated)
  except (AttributeError, ValueError):
    return False


class TamagotchiWidget(Widget):
  def __init__(self, scale=1.0):
    """scale is relative to the original 192x208 art."""
    super().__init__()
    manifest = json.loads(ASSETS_DIR.joinpath("pingu_spritesheet.json").read_text())
    self._anims = manifest["anims"]
    self._frames = manifest["frames"]  # [x, y, w, h, x in cell, y in cell], atlas pixels
    self._cell_w, cell_h = manifest["cell"]
    self._draw_scale = scale / manifest["scale"]

    self.set_rect(rl.Rectangle(0, 0, self._cell_w * self._draw_scale, cell_h * self._draw_scale))
    self._texture = gui_app.texture("pingu_spritesheet.png")

    self._anim = "idle"
    self._flip = False
    self._frame = 0
    self._last_update = time.monotonic()

    self._was_engaged = False
    self._was_high_beam = False
    self._was_red_light = False
    self._stopped_since: float | None = None
    self._until: dict[str, float] = {}
    self._extra: str | None = None
    self._next_extra = time.monotonic() + random.uniform(*IDLE_EXTRA_EVERY)

  def _hold(self, anim: str, now: float, secs: float = HOLD_TIME):
    self._until[anim] = now + secs

  def _held(self, anim: str, now: float) -> bool:
    return now < self._until.get(anim, 0.)

  def _pick_anim(self, now: float) -> tuple[str, bool]:
    """(animation, mirrored) for the current car state, most important first."""
    sm = ui_state.sm
    if not sm.valid["carState"]:
      return "idle", False

    cs = sm["carState"]
    ss = sm["selfdriveState"]
    engaged = ui_state.status != UIStatus.DISENGAGED
    stopped = cs.vEgo < STOPPED_SPEED

    # events that play for a moment
    if ui_state.engaged != self._was_engaged:
      self._hold("shades" if ui_state.engaged else "your_turn", now)
      self._was_engaged = ui_state.engaged
    high_beam = sm["starpilotCarState"].highBeamOn
    if high_beam and not self._was_high_beam:
      self._hold("high_beam", now)
    self._was_high_beam = high_beam
    if cs.aEgo < HARD_BRAKE and not self._held("hockey_stop", now):
      self._hold("hockey_stop", now, 2.5)
    if engaged and _lat_saturated(sm["controlsState"]):
      self._hold("steer_fight", now, 1.)

    # green light: StarPilot's alert (if that toggle is on), or the red light it stopped for clears
    plan = sm["starpilotPlan"]
    red_light = plan.redLight or plan.forcingStop
    if any(e.name in GO_EVENTS for e in plan.starpilotEvents) or (stopped and self._was_red_light and not red_light):
      self._hold("green_light", now, 2.5)
    self._was_red_light = red_light

    if not stopped:
      self._stopped_since = None
    elif self._stopped_since is None:
      self._stopped_since = now
    stopped_for = now - self._stopped_since if self._stopped_since is not None else 0.

    if sm["deviceState"].thermalStatus >= ThermalStatus.overheated:
      return "melt", False
    if ss.alertStatus == AlertStatus.critical:
      return "take_wheel", False
    if cs.seatbeltUnlatched or cs.doorOpen:
      return "buckle", False
    if cs.gearShifter == GearShifter.reverse:
      return "reverse", False
    if self._held("hockey_stop", now):
      return "hockey_stop", False

    dm_level = sm["driverMonitoringState"].alertLevel
    if dm_level >= DMAlertLevel.two:
      return "referee", False
    if dm_level == DMAlertLevel.one:
      return "watching", False

    # lane changes and blind spots
    meta = sm["modelV2"].meta
    lc_right = meta.laneChangeDirection == LaneChangeDirection.right
    if meta.laneChangeState in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
      return ("slide_right" if lc_right else "slide_left"), False
    if meta.laneChangeState == LaneChangeState.preLaneChange:
      if cs.rightBlindspot if lc_right else cs.leftBlindspot:
        return "hold_up", lc_right
      return ("look_right" if lc_right else "look_left"), False
    if cs.leftBlinker:
      return ("hold_up", False) if cs.leftBlindspot else ("walk_left", False)
    if cs.rightBlinker:
      return ("hold_up", True) if cs.rightBlindspot else ("walk_right", False)

    if engaged and (cs.steerFaultTemporary or self._held("steer_fight", now)):
      return "steer_fight", False
    if ui_state.status == UIStatus.OVERRIDE:
      return "chill", False
    for anim in ("your_turn", "shades", "high_beam", "green_light"):
      if self._held(anim, now):
        return anim, False

    # model thinks a disengagement is coming
    if engaged and not stopped and sm["modelV2"].confidence == ConfidenceClass.red:
      self._hold("drunk", now)
    if engaged and self._held("drunk", now):
      return "drunk", False

    lead = sm["radarState"].leadOne
    if lead.status and cs.vEgo > TAILGATE_MIN_SPEED and lead.dRel / cs.vEgo < TAILGATE_GAP:
      self._hold("noot", now)
    if not stopped and self._held("noot", now):
      return "noot", False

    speed_limit = sm["mapdOut"].speedLimit
    if speed_limit > 0 and cs.vEgo > speed_limit * SPEEDING_MARGIN + 1.:
      return "speed_demon", False
    if red_light:
      return "stop_sign", False

    if stopped:
      if stopped_for > SLEEP_AFTER:
        return "sleep", False
      if engaged:
        return "wave", False
      return self._idle(now), False

    if abs(sm["controlsState"].curvature) * cs.vEgo ** 2 > CURVE_ACCEL:
      return "carve", False
    if cs.vEgo > HIGHWAY_SPEED:
      return "toboggan", False
    if engaged:
      return ("scientist" if ss.experimentalMode else "jump"), False
    return "walk_front", False

  def _idle(self, now: float) -> str:
    """Idle (flashlight and nightcap with the headlights on), with a little routine now and then."""
    if self._extra is not None and self._held(self._extra, now):
      return self._extra
    if now >= self._next_extra:
      self._extra = random.choice(IDLE_EXTRAS)
      play_for = self._anims[self._extra]["frames"] / FPS
      if not self._anims[self._extra]["loop"]:
        play_for += 1.  # linger on the last frame
      self._hold(self._extra, now, play_for)
      self._next_extra = now + play_for + random.uniform(*IDLE_EXTRA_EVERY)
      return self._extra
    self._extra = None
    return "night" if ui_state.sm["starpilotCarState"].lowBeamOn else "idle"

  def _render(self, _):
    now = time.monotonic()
    anim, flip = self._pick_anim(now)
    if (anim, flip) != (self._anim, self._flip):
      self._anim, self._flip = anim, flip
      self._frame = 0
      self._last_update = now

    info = self._anims[self._anim]
    if now - self._last_update > 1.0 / FPS:
      if info["loop"]:
        self._frame = (self._frame + 1) % info["frames"]
      else:
        self._frame = min(self._frame + 1, info["frames"] - 1)  # play once, hold the last frame
      self._last_update = now

    x, y, w, h, ox, oy = self._frames[info["start"] + self._frame]
    if self._flip:
      ox = self._cell_w - ox - w
    s = self._draw_scale
    src_rect = rl.Rectangle(x, y, -w if self._flip else w, h)
    dst_rect = rl.Rectangle(self.rect.x + ox * s, self.rect.y + oy * s, w * s, h * s)
    rl.draw_texture_pro(self._texture, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)
