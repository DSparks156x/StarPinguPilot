#!/usr/bin/env python3
"""Pack Pingu animations into selfdrive/assets/pingu_spritesheet.png + .json.

Sources:
  base_spritesheet.png  the original hand-made sheet (8 cols x 9 rows of 192x208)
  strips/<name>.png     one generated animation per image, one row of frames on solid magenta

Every frame is first laid out in a CELL_W x CELL_H cell (original-art pixels) with Pingu's feet
on BASELINE and his body centered, so all animations line up. The original art is 192x208 with
Pingu standing 198 px tall, feet at y=202; cells are bigger so props (megaphone, Zzz, signs,
juggled snowballs) fit. Each cell is then scaled to SHEET_SCALE (the onroad UI never draws Pingu
bigger than that), trimmed to its pixels and packed tightly into the atlas. The .json gives each
frame's atlas rect and where that rect sits in the cell.

Usage: python pingu_sprites/pack_sprites.py [--preview out.png] [--calibrate out.png]
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parent / "selfdrive" / "assets"

# cell in original (192x208 sheet) pixel units
CELL_W, CELL_H = 320, 288
BASELINE = CELL_H - 6           # feet bottom, same 6 px gap as the original sheet
STAND_H = 198                   # standing Pingu height in the original sheet
SHEET_SCALE = 0.75
ATLAS_W = 2048
PAD = 2                         # transparent gap between atlas rects so filtering doesn't bleed

OLD_W, OLD_H = 192, 208
OLD_BASELINE = 202

# name: (row, frames, loop) in base_spritesheet.png
BASE_ANIMS = {
  "idle":        (0, 6, True),
  "walk_right":  (1, 8, True),
  "walk_left":   (2, 8, True),
  "wave":        (3, 4, True),
  "jump":        (4, 5, True),
  "look_around": (6, 6, False),
  "walk_front":  (7, 6, True),
  "giggle":      (8, 6, False),
}

# name: (file, frames, loop, ref_frame, ref_kind, ref_size)
#   ref_kind "h":     height of the biggest blob in ref_frame is ref_size * standing Pingu
#   ref_kind "w":     width of the biggest blob in ref_frame is ref_size * standing Pingu (lying down)
#   ref_kind "belly": height of the white belly in ref_frame is ref_size * the original belly
#                     (for frames with a prop sticking up from Pingu, like a sign)
STRIP_ANIMS = {
  "drunk":       ("drunk.png",       8, True,  0, "h", 1.0),
  "noot":        ("noot.png",        6, True,  0, "h", 1.0),
  "slide_right": ("slide_right.png", 6, True,  0, "w", 0.95),
  "slide_left":  ("slide_left.png",  6, True,  0, "w", 0.95),
  "melt":        ("melt.png",        8, False, 0, "h", 1.0),
  "sleep":       ("sleep.png",       6, True,  0, "h", 0.85),
  "shades":      ("shades.png",      6, False, 0, "h", 1.0),
  "look_right":  ("look_right.png",  4, True,  0, "h", 1.0),
  "look_left":   ("look_left.png",   4, True,  0, "h", 1.0),
  "toboggan":    ("toboggan.png",    6, True,  0, "w", 1.0),
  "carve":       ("carve.png",       6, True,  2, "belly", 1.0),
  "stop_sign":   ("stop_sign.png",   6, True,  0, "belly", 1.0),
  "scientist":   ("scientist.png",   6, True,  0, "h", 1.0),
  "reverse":     ("reverse.png",     6, True,  0, "belly", 1.0),
  "watching":    ("watching.png",    6, True,  0, "h", 1.0),
  "buckle":      ("buckle.png",      6, False, 1, "h", 1.0),
  "night":       ("night.png",       6, True,  0, "belly", 1.0),
  "high_beam":   ("high_beam.png",   4, False, 0, "h", 1.0),
  "fishing":     ("fishing.png",     8, False, 0, "h", 0.85),
  "fish_snack":  ("fish_snack.png",  6, False, 0, "h", 1.0),
  "snowman":     ("snowman.png",     8, False, 6, "h", 1.0),
  "juggle":      ("juggle.png",      6, True,  0, "belly", 1.0),
  "steer_fight": ("steer_fight.png", 6, True,  0, "h", 1.0),
  "speed_demon": ("speed_demon.png", 6, True,  0, "h", 1.0),
  "your_turn":   ("your_turn.png",   6, False, 0, "h", 1.0),
  "chill":       ("chill.png",       4, True,  0, "h", 1.0),
  "take_wheel":  ("take_wheel.png",  6, True,  0, "h", 1.0),
  "hold_up":     ("hold_up.png",     5, False, 1, "h", 1.0),
  "referee":     ("referee.png",     6, True,  0, "h", 1.0),
  "hockey_stop": ("hockey_stop.png", 6, False, 5, "h", 1.0),
  "green_light": ("green_light.png", 6, False, 0, "belly", 1.0),
}

KEY = np.array([250.0, 4.0, 252.0])


def label(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
  """8-connected components (no scipy on every machine). Returns labels (0 = background) and sizes[label]."""
  # union-find over horizontal runs, linked to overlapping runs on the row above
  parent: list[int] = [0]

  def find(i):
    while parent[i] != i:
      parent[i] = parent[parent[i]]
      i = parent[i]
    return i

  run_rows = []
  prev = []
  for y in range(mask.shape[0]):
    row = np.concatenate(([0], mask[y].astype(np.int8), [0]))
    d = np.diff(row)
    starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]
    cur = []
    for a, b in zip(starts, ends, strict=True):
      lab = len(parent)
      parent.append(lab)
      for pa, pb, plab in prev:
        if pa <= b and pb >= a:  # touches, diagonals included
          ra, rb = find(lab), find(plab)
          parent[max(ra, rb)] = min(ra, rb)
      cur.append((a, b, lab))
    run_rows.append(cur)
    prev = cur

  roots = np.array([find(i) for i in range(len(parent))])
  _, compact = np.unique(roots, return_inverse=True)
  labels = np.zeros(mask.shape, dtype=np.int32)
  for y, cur in enumerate(run_rows):
    for a, b, lab in cur:
      labels[y, a:b] = compact[lab]
  sizes = np.bincount(labels.ravel())
  sizes[0] = 0
  return labels, sizes


def key_magenta(path: Path) -> np.ndarray:
  """Solid magenta background -> straight-alpha RGBA float array."""
  rgb = np.asarray(Image.open(path).convert("RGB"), dtype=np.float32)
  magentaness = np.minimum(rgb[..., 0], rgb[..., 2]) - rgb[..., 1]
  alpha = np.clip((170.0 - magentaness) / 100.0, 0.0, 1.0)

  # Edge pixels are Pingu's color blended with magenta, which leaves a purple fringe on outlines
  # and thin white streaks. Next to the background, read any pink/purple tint as partial
  # transparency. Not deeper in, where it is real color (drunk cheeks).
  bg = magentaness > 120
  near_bg = bg.copy()
  r = 4
  padded = np.pad(bg, r)
  for dy in range(-r, r + 1):
    for dx in range(-r, r + 1):
      near_bg |= padded[r + dy:r + dy + bg.shape[0], r + dx:r + dx + bg.shape[1]]
  edge_alpha = np.clip(1.0 - (magentaness - 5.0) / 195.0, 0.0, 1.0)
  alpha = np.where(near_bg, np.minimum(alpha, edge_alpha), alpha)

  # drop specks left over from the key
  labels, sizes = label(alpha > 0.5)
  alpha[(sizes < 20)[labels] & (labels > 0)] = 0.0
  alpha[alpha < 0.05] = 0.0

  # unmix the magenta out of the edge pixels
  a = np.maximum(alpha, 1e-3)[..., None]
  color = np.clip((rgb - (1.0 - a) * KEY) / a, 0, 255)
  return np.dstack([color, alpha * 255.0])


def split_frames(rgba: np.ndarray, n: int) -> list[tuple[int, int]]:
  """Column ranges of the n frames: runs of non-empty columns, closest neighbours merged."""
  cols = (rgba[..., 3] > 127).sum(axis=0) > 0
  runs, start = [], None
  for x, filled in enumerate(cols):
    if filled and start is None:
      start = x
    elif not filled and start is not None:
      runs.append([start, x])
      start = None
  if start is not None:
    runs.append([start, len(cols)])
  # fold specks (a stray bubble, a bit of spray) into the closest run
  while len(runs) > 1:
    widths = [b - a for a, b in runs]
    i = int(np.argmin(widths))
    if widths[i] >= 0.02 * len(cols):
      break
    left = runs[i][0] - runs[i - 1][1] if i > 0 else np.inf
    right = runs[i + 1][0] - runs[i][1] if i < len(runs) - 1 else np.inf
    j = i - 1 if left <= right else i + 1
    runs[min(i, j)] = [runs[min(i, j)][0], runs[max(i, j)][1]]
    del runs[max(i, j)]
  if len(runs) < n:
    # frames touch: cut at the emptiest column near each even-pitch boundary
    coverage = (rgba[..., 3] > 127).sum(axis=0)
    pitch = len(cols) / n
    cuts = [0]
    for k in range(1, n):
      lo, hi = round((k - 0.15) * pitch), round((k + 0.15) * pitch)
      cuts.append(lo + int(np.argmin(coverage[lo:hi])))
    cuts.append(len(cols))
    return [(cuts[k], cuts[k + 1]) for k in range(n)]
  while len(runs) > n:
    gaps = [runs[i + 1][0] - runs[i][1] for i in range(len(runs) - 1)]
    i = int(np.argmin(gaps))
    runs[i] = [runs[i][0], runs[i + 1][1]]
    del runs[i + 1]
  return [tuple(r) for r in runs]


def biggest_blob_bbox(mask: np.ndarray) -> tuple[int, int, int, int]:
  labels, sizes = label(mask)
  ys, xs = np.nonzero(labels == int(np.argmax(sizes)))
  return xs.min(), ys.min(), xs.max() + 1, ys.max() + 1


def belly_height(rgba: np.ndarray) -> int:
  white = (rgba[..., 3] > 127) & (rgba[..., :3] > 200).all(axis=-1)
  _, y0, _, y1 = biggest_blob_bbox(white)
  return y1 - y0


def to_image(rgba: np.ndarray) -> Image.Image:
  return Image.fromarray(np.round(rgba).astype(np.uint8), "RGBA")


def strip_frames(name, file, n, ref_frame, ref_kind, ref_size, base_belly) -> list[Image.Image]:
  rgba = key_magenta(HERE / "strips" / file)
  ranges = split_frames(rgba, n)
  alpha = rgba[..., 3]

  ref = rgba[:, ranges[ref_frame][0]:ranges[ref_frame][1]]
  if ref_kind == "belly":
    scale = ref_size * base_belly / belly_height(ref)
  else:
    x0, y0, x1, y1 = biggest_blob_bbox(ref[..., 3] > 127)
    scale = ref_size * STAND_H / ((y1 - y0) if ref_kind == "h" else (x1 - x0))

  # one baseline for the whole strip so jumps and melting keep their vertical motion
  rows = np.nonzero((alpha > 127).any(axis=1))[0]
  top, bottom = rows.min(), rows.max() + 1
  # keep horizontal motion too: frames sit on an even pitch, anchor each to its slot center,
  # shifted so Pingu's black body (not a sign or snowman next to him) is centered in ref_frame
  pitch = rgba.shape[1] / n
  dark = (ref[..., 3] > 127) & (ref[..., :3] < 70).all(axis=-1)
  body_x = ranges[ref_frame][0] + np.nonzero(dark)[1].mean()
  shift = body_x - (ref_frame + 0.5) * pitch

  # shrink the whole strip if any frame would spill out of its cell
  wanted = scale
  for i, (a, b) in enumerate(ranges):
    center = (i + 0.5) * pitch + shift
    half_w = max(center - a, b - center)
    scale = min(scale, (CELL_W / 2 - 2) / half_w, (BASELINE - 2) / (bottom - top))

  frames = []
  for i, (a, b) in enumerate(ranges):
    center = (i + 0.5) * pitch + shift
    crop = to_image(rgba[top:bottom, a:b])
    w, h = max(1, round(crop.width * scale)), max(1, round(crop.height * scale))
    crop = crop.resize((w, h), Image.LANCZOS)
    cell = Image.new("RGBA", (CELL_W, CELL_H), (0, 0, 0, 0))
    cell.alpha_composite(crop, (round(CELL_W / 2 + (a - center) * scale), BASELINE - h))
    frames.append(cell)
  shrunk = f"  (shrunk {scale / wanted:.0%} to fit)" if scale < wanted * 0.995 else ""
  print(f"{name:12s} {n} frames  scale {scale:.3f}{shrunk}")
  return frames


def base_frames(sheet: Image.Image, row: int, n: int) -> list[Image.Image]:
  frames = []
  for c in range(n):
    old = sheet.crop((c * OLD_W, row * OLD_H, (c + 1) * OLD_W, (row + 1) * OLD_H))
    cell = Image.new("RGBA", (CELL_W, CELL_H), (0, 0, 0, 0))
    cell.alpha_composite(old, ((CELL_W - OLD_W) // 2, BASELINE - OLD_BASELINE))
    frames.append(cell)
  return frames


def pack(cells: list[Image.Image]) -> tuple[Image.Image, list[list[int]]]:
  """Scale, trim and shelf-pack the cells. Returns the atlas and per frame [x, y, w, h, ox, oy]."""
  trimmed = []
  for cell in cells:
    small = cell.resize((round(CELL_W * SHEET_SCALE), round(CELL_H * SHEET_SCALE)), Image.LANCZOS)
    bbox = small.getchannel("A").point(lambda v: 255 if v > 2 else 0).getbbox() or (0, 0, 1, 1)
    trimmed.append((small.crop(bbox), bbox[0], bbox[1]))

  order = sorted(range(len(trimmed)), key=lambda i: -trimmed[i][0].height)
  rects: list[list[int]] = [[]] * len(trimmed)
  x = y = shelf_h = 0
  for i in order:
    img, ox, oy = trimmed[i]
    if x + img.width > ATLAS_W:
      x, y, shelf_h = 0, y + shelf_h + PAD, 0
    rects[i] = [x, y, img.width, img.height, ox, oy]
    x += img.width + PAD
    shelf_h = max(shelf_h, img.height)

  atlas = Image.new("RGBA", (ATLAS_W, y + shelf_h), (0, 0, 0, 0))
  for (img, _, _), (rx, ry, *_) in zip(trimmed, rects, strict=True):
    atlas.paste(img, (rx, ry))
  return atlas, rects


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--preview", type=Path, help="also write every frame in a labelled grid here")
  parser.add_argument("--calibrate", type=Path, help="also write each animation's first frame next to idle here")
  args = parser.parse_args()

  base = Image.open(HERE / "base_spritesheet.png").convert("RGBA")
  base_belly = belly_height(np.asarray(base.crop((0, 0, OLD_W, OLD_H)), dtype=np.float32))
  cells: list[Image.Image] = []
  anims = {}
  for name, (row, n, loop) in BASE_ANIMS.items():
    anims[name] = {"start": len(cells), "frames": n, "loop": loop}
    cells += base_frames(base, row, n)
  for name, (file, n, loop, ref_frame, ref_kind, ref_size) in STRIP_ANIMS.items():
    anims[name] = {"start": len(cells), "frames": n, "loop": loop}
    cells += strip_frames(name, file, n, ref_frame, ref_kind, ref_size, base_belly)

  atlas, rects = pack(cells)
  if atlas.height > 4096:
    raise ValueError(f"atlas is {atlas.width}x{atlas.height}, raise ATLAS_W")
  atlas.save(ASSETS / "pingu_spritesheet.png", optimize=True)
  manifest = {
    "cell": [round(CELL_W * SHEET_SCALE), round(CELL_H * SHEET_SCALE)],  # atlas pixels
    "scale": SHEET_SCALE,              # atlas pixels per original art pixel
    "frames": rects,                   # [x, y, w, h, x in cell, y in cell]
    "anims": anims,
  }
  (ASSETS / "pingu_spritesheet.json").write_text(json.dumps(manifest, separators=(",", ":")) + "\n")
  print(f"{len(cells)} frames -> {atlas.width}x{atlas.height} ({atlas.width * atlas.height * 4 / 1e6:.1f} MB as a texture)")

  if args.preview:
    cols = 16
    grid = Image.new("RGBA", (cols * CELL_W, -(-len(cells) // cols) * CELL_H), (40, 44, 52, 255))
    for i, c in enumerate(cells):
      grid.alpha_composite(c, ((i % cols) * CELL_W, (i // cols) * CELL_H))
    grid.convert("RGB").save(args.preview)

  if args.calibrate:
    from PIL import ImageDraw
    names = list(STRIP_ANIMS)
    grid = Image.new("RGBA", (len(names) * CELL_W, CELL_H + 16), (40, 44, 52, 255))
    draw = ImageDraw.Draw(grid)
    for i, name in enumerate(names):
      # faint idle Pingu behind each animation's reference frame
      ghost = cells[0].copy()
      ghost.putalpha(ghost.getchannel("A").point(lambda v: v // 3))
      grid.alpha_composite(ghost, (i * CELL_W, 16))
      grid.alpha_composite(cells[anims[name]["start"] + STRIP_ANIMS[name][3]], (i * CELL_W, 16))
      draw.text((i * CELL_W + 4, 2), name, fill=(230, 230, 230))
    grid.convert("RGB").save(args.calibrate)


if __name__ == "__main__":
  main()
