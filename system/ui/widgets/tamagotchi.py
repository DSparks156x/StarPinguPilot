import time
import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus

class TamagotchiWidget(Widget):
  def __init__(self, scale=1.0):
    super().__init__()

    self.frame_width = 1536 / 8
    self.frame_height = 1872 / 9

    self.set_rect(rl.Rectangle(0, 0, self.frame_width * scale, self.frame_height * scale))
    self._texture = gui_app.texture("pingu_spritesheet.png")

    self.frame = 0
    self.last_update = time.monotonic()
    self.fps = 6

    self.current_anim = 0
    self.num_frames = 6

  def update_animation(self):
    if not ui_state.sm.valid["carState"]:
      self._set_anim(0, 6)
      return

    cs = ui_state.sm["carState"]

    if cs.leftBlinker:
      self._set_anim(2, 8)
    elif cs.rightBlinker:
      self._set_anim(1, 8)
    elif ui_state.status == UIStatus.OVERRIDE:
      self._set_anim(5, 8)
    elif ui_state.status != UIStatus.DISENGAGED:
      if cs.vEgo < 0.1:
        self._set_anim(3, 4) # Waving when engaged but stopped
      else:
        self._set_anim(4, 5) # Jumping when engaged and moving
    else:
      if cs.vEgo >= 0.1:
        self._set_anim(7, 6) # Running when moving manually
      else:
        self._set_anim(0, 6) # Idle

  def _set_anim(self, anim_row: int, num_frames: int):
    if self.current_anim != anim_row:
      self.current_anim = anim_row
      self.num_frames = num_frames
      self.frame = 0

  def _render(self, _):
    self.update_animation()

    now = time.monotonic()
    if now - self.last_update > 1.0 / self.fps:
      self.frame = int((self.frame + 1) % self.num_frames)
      self.last_update = now

    src_rect = rl.Rectangle(self.frame * self.frame_width, self.current_anim * self.frame_height, self.frame_width, self.frame_height)
    dst_rect = rl.Rectangle(self.rect.x, self.rect.y, self.rect.width, self.rect.height)
    rl.draw_texture_pro(self._texture, src_rect, dst_rect, rl.Vector2(0, 0), 0.0, rl.WHITE)
