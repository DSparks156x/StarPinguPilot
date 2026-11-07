# StarPinguPilot: Pingu reads cereal fields by name, so run his state picker on real capnp messages.
# 2026-11-07: lead.status (renamed lead.present in StarPilot 7) took the UI down as soon as carState turned valid.
from types import SimpleNamespace

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.ui.ui_state import UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import tamagotchi

SERVICES = ("carState", "selfdriveState", "starpilotCarState", "controlsState", "starpilotLongitudinalPlan", "deviceState",
            "carControl", "driverMonitoringState", "modelV2", "radarState", "slcState")


def moving(m):
  m["carState"].vEgo = 15.


def tailgating(m):
  moving(m)
  m["radarState"].leadOne.present = True
  m["radarState"].leadOne.dRel = 3.


def speeding(m):
  moving(m)
  m["slcState"].observationKind = "valid"
  m["slcState"].speedLimit = 5.


def curving(m):
  moving(m)
  m["controlsState"].curvature = 0.1


def lane_change(m):
  moving(m)
  m["modelV2"].meta.laneChangeState = "preLaneChange"
  m["modelV2"].meta.laneChangeDirection = "right"


def joystick(m):
  m["controlsState"].lateralControlState.init("debugState")


SCENARIOS = {
  "idle": lambda m: None,
  "walk_front": moving,
  "noot": tailgating,
  "speed_demon": speeding,
  "carve": curving,
  "look_right": lane_change,
  "gamer": joystick,
  "toboggan": lambda m: setattr(m["carState"], "vEgo", 35.),
  "hockey_stop": lambda m: setattr(m["carState"], "aEgo", -5.),
  "buckle": lambda m: setattr(m["carState"], "doorOpen", True),
  "reverse": lambda m: setattr(m["carState"], "gearShifter", "reverse"),
  "walk_left": lambda m: setattr(m["carState"], "leftBlinker", True),
  "melt": lambda m: setattr(m["deviceState"], "thermalStatus", "overheated"),
  "take_wheel": lambda m: setattr(m["selfdriveState"], "alertStatus", "critical"),
  "watching": lambda m: setattr(m["driverMonitoringState"], "alertLevel", "one"),
  "referee": lambda m: setattr(m["driverMonitoringState"], "alertLevel", "two"),
  "stop_sign": lambda m: setattr(m["starpilotLongitudinalPlan"], "forceStopHolding", True),
}


class FakeSM(dict):
  def __init__(self, messages):
    super().__init__(messages)
    self.valid = dict.fromkeys(messages, True)


@pytest.fixture
def widget(monkeypatch):
  monkeypatch.setattr(gui_app, "texture", lambda *a, **k: None)
  return tamagotchi.TamagotchiWidget(scale=0.75)


def pick(monkeypatch, widget, setup, status=UIStatus.DISENGAGED):
  events = {s: messaging.new_message(s) for s in SERVICES}
  builders = {s: getattr(e, s) for s, e in events.items()}
  setup(builders)
  # readers, as SubMaster hands out
  sm = FakeSM({s: getattr(e.as_reader(), s) for s, e in events.items()})
  monkeypatch.setattr(tamagotchi, "ui_state", SimpleNamespace(sm=sm, status=status, engaged=status != UIStatus.DISENGAGED))
  return widget._pick_anim(1000.)


@pytest.mark.parametrize("anim", SCENARIOS)
def test_pick_anim_disengaged(monkeypatch, widget, anim):
  assert pick(monkeypatch, widget, SCENARIOS[anim])[0] == anim


@pytest.mark.parametrize("experimental,anim", [(False, "jump"), (True, "scientist")])
def test_pick_anim_engaged(monkeypatch, widget, experimental, anim):
  def setup(m):
    moving(m)
    m["modelV2"].confidence = "green"
    m["selfdriveState"].experimentalMode = experimental
  widget._was_engaged = True  # past the "shades" greeting
  assert pick(monkeypatch, widget, setup, UIStatus.ENGAGED)[0] == anim


def test_low_confidence_is_drunk(monkeypatch, widget):
  def setup(m):
    moving(m)
    m["modelV2"].confidence = "red"
  widget._was_engaged = True
  assert pick(monkeypatch, widget, setup, UIStatus.ENGAGED)[0] == "drunk"


def test_no_car_state_yet_is_idle(monkeypatch, widget):
  sm = FakeSM({"carState": None})
  sm.valid["carState"] = False
  monkeypatch.setattr(tamagotchi, "ui_state", SimpleNamespace(sm=sm, status=UIStatus.DISENGAGED, engaged=False))
  assert widget._pick_anim(1000.) == ("idle", False)
