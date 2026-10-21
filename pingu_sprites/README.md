# Pingu sprite wishlist

New animations for the onroad Tamagotchi (`system/ui/widgets/tamagotchi.py`).
Each one is tied to a real openpilot / StarPilot signal so it can be wired up later.

## What exists today

`selfdrive/assets/pingu_spritesheet.png` — 8 columns x 9 rows, each frame **192 x 208**, transparent background, played at 6 fps.

| Row | Animation | Frames | Used for |
|---|---|---|---|
| 0 | Idle blink | 6 | disengaged + stopped, no carState |
| 1 | Walk right (side view) | 8 | right blinker |
| 2 | Walk left (side view) | 8 | left blinker |
| 3 | Wave | 4 | engaged + stopped |
| 4 | Jump | 5 | engaged + moving |
| 5 | Sad / crying | 8 | gas override |
| 6 | Look around | 6 | **unused** |
| 7 | Front-facing walk | 6 | driving manually |
| 8 | Giggle | 6 | **unused** |

## How to use ChatGPT for this

1. Start **one new ChatGPT chat** for all the sprites, so it keeps the style in context.
2. Paste the **setup message** below and attach `pingu_reference.png` (in this folder). Also attach `selfdrive/assets/pingu_spritesheet.png` if you want.
3. Then paste **one animation prompt at a time**. Each one only needs the animation description; the rules come from the setup message.
4. If a result drifts (wrong size, different style, extra props), reply: *"Redo it, closer to the reference: same size, same outline, same proportions, feet on the same line in every frame."*
5. For an animation with a prop (bottle, goggles, sign), make the **key pose** first ("draw just frame 1"), then say *"now animate this exact image into N frames using the rules"*. That keeps the prop the same in every frame.
6. Save each result as `row_<name>.png` in this folder. Don't worry about exact pixel sizes — a packer script will remove the magenta, scale, line up the feet and add the frames to the real sheet.

### Setup message (paste once, with the reference image)

```
I'm making new animations for a pixel-art penguin sprite. The attached image shows the character in its exact style. I'll ask for one animation at a time. Follow these rules for every image:

- Same character exactly: round black head and body, white belly with a thin yellow inner outline, big red beak, white eyes with black pupils, short orange feet, chunky pixel art with a dark outline. Same proportions as the reference. No hair, clothes or accessories unless I ask.
- One single horizontal row of frames, left to right, in animation order.
- Solid flat magenta background (#FF00FF) everywhere. No gradient, no floor, no shadow, no transparency checkerboard.
- The penguin is the same size in every frame, centered in its frame, with his feet on the same horizontal baseline (unless the animation is a jump or fall).
- Leave clear magenta space between frames. Nothing may touch or overlap the next frame.
- Small props and effects (bubbles, Zzz, sweat drops, signs) must stay inside the frame and be in the same pixel-art style.
- No text, letters or numbers unless I ask for them.
- The frames should loop smoothly unless I say it plays once.

Reply "ready" and wait for the first animation.
```

### Animation prompts

Paste them as is. The frame count matters: at 6 fps, 6 frames = 1 second.

---

#### Top picks (make these first)

**1. Drunk — model confidence low** · `modelV2.confidence` yellow/red · 8 frames, loops
```
Animation: "drunk". 8 frames. The penguin holds a brown glass beer bottle in one flipper. He sways left and right, stumbles and almost falls over, his eyes are half-closed with a goofy smile, his cheeks are a little pink, and small white hiccup bubbles float up from his beak in some frames. In one frame he takes a sip from the bottle.
```

**2. Noot noot — following a car too closely** · lead time gap < ~1 s · 6 frames, loops
```
Animation: "noot noot". 6 frames, front view. The penguin gets annoyed, leans forward, and stretches his beak wide open into a trumpet/megaphone shape as he honks loudly. Show 2-3 short curved sound lines coming out of the beak on the honk frames. He shakes one angry flipper. His eyebrows are angry.
```

**3. Belly slide — lane change in progress** · `laneChangeState == laneChanging` · 6 frames, loops · make two: one facing left, one facing right
```
Animation: "belly slide right". 6 frames, side view facing right. The penguin lies flat on his belly with his flippers back and slides across ice to the right. His feet stick out behind him. Add small white ice sparkle/speed lines behind him. He looks happy.
```
Then: `Now the exact same animation mirrored, facing and sliding left.`

**4. Melting — device overheating** · `deviceState.thermalStatus` red/danger · 8 frames, plays once then holds the last frame
```
Animation: "overheating". 8 frames, plays once. The penguin is very hot: big blue sweat drops fly off him, he fans himself with one flipper, his tongue is out. Over the frames he slowly melts down into a black-and-white puddle, ending with only his eyes and red beak sticking out of the puddle.
```

**5. Falls asleep — stopped for a long time** · standstill > 30 s · 6 frames, loops
```
Animation: "asleep". 6 frames. The penguin sits down on the ground with his feet sticking forward, eyes closed, head nodding slowly forward and back. A small "Z" bubble floats up and gets bigger over the frames. (A single letter Z is allowed.)
```

**6. Puts on sunglasses — just engaged** · engage transition · 6 frames, plays once
```
Animation: "cool engage". 6 frames, plays once, front view. The penguin pulls black pixel-art sunglasses out of nowhere, lifts them up and puts them on, then the last 2 frames are him standing confidently with a smug smile and a tiny white sparkle on the sunglasses.
```

---

#### Driving

**7. Look over shoulder — about to change lanes** · `laneChangeState == preLaneChange` · 4 frames, loops · make left and right
```
Animation: "shoulder check right". 4 frames. The penguin turns his head over his right shoulder to look behind him, holds the look, and turns back, eyes squinting to check.
```

**8. Startled — blind spot car while signaling** · `leftBlindspot` / `rightBlindspot` + blinker · 5 frames, plays once
```
Animation: "blind spot scare". 5 frames, plays once. The penguin looks to the side, then his eyes go huge, he jumps back in fright with his flippers up, then stays frozen in a shocked pose.
```

**9. Faceplant — hard braking** · `carState.aEgo < -2.5` · 6 frames, plays once
```
Animation: "skid and faceplant". 6 frames, plays once, side view facing right. The penguin skids on ice with his feet out in front and his flippers flailing, little ice sprays at his feet, then he trips forward and lands flat on his face. Last frame: face down with small stars spinning above him.
```

**10. Tobogganing — highway speed** · `vEgo > ~30 m/s` · 6 frames, loops
```
Animation: "highway toboggan". 6 frames, side view facing right. The penguin rides fast on his belly wearing ski goggles, flippers pointed back, with white speed lines streaming behind him and a little snow spray.
```

**11. Skier lean — twisty road** · high lateral accel · 6 frames, loops
```
Animation: "carving". 6 frames, front view. The penguin leans hard to one side like a skier carving a turn, then back to center, then the other side, flippers out for balance, concentrated face.
```

**12. Stop sign — red light / forced stop** · `starpilotPlan.redLight` / `forcingStop` · 6 frames, loops
```
Animation: "waiting at red light". 6 frames. The penguin holds a small red octagon stop sign on a stick in one flipper and impatiently taps one foot, rolling his eyes in one frame.
```

**13. Scientist — experimental mode** · `selfdriveState.experimentalMode` · 6 frames, loops
```
Animation: "mad scientist". 6 frames. The penguin wears clear lab goggles and holds a round glass flask with bubbling green liquid. Green bubbles pop out of the flask, and he grins mischievously and peers into it.
```

**14. Fighting the wheel — steering maxed out / steer fault** · `steerFaultTemporary`, steer saturated · 6 frames, loops
```
Animation: "fighting the wheel". 6 frames. The penguin holds a big round brown steering wheel almost as big as he is, and struggles to turn it hard, straining with gritted teeth, sweat drops flying, leaning his whole body into it.
```

**15. Reverse — reverse gear** · `gearShifter == reverse` · 6 frames, loops
```
Animation: "reversing". 6 frames, front view. The penguin waddles backwards carefully, looking over his shoulder, holding a small yellow sign with a black arrow pointing down.
```

---

#### Driver monitoring and alerts

**16. I'm watching you — driver distracted** · `driverMonitoringState.isDistracted` · 6 frames, loops
```
Animation: "watching you". 6 frames, front view. The penguin points both flippers at his own eyes, then points one flipper straight at the viewer, squinting suspiciously, then repeats.
```

**17. Alarm clock — driver not paying attention for a while** · low `awarenessStatus` · 6 frames, loops
```
Animation: "wake up". 6 frames. The penguin holds up a red ringing alarm clock toward the viewer and shakes it, with shaking lines around the clock, mouth open shouting, eyebrows worried.
```

**18. Panic — critical "take control" alert** · `selfdriveState.alertStatus == critical` · 8 frames, loops
```
Animation: "panic". 8 frames. The penguin panics: he runs in a small circle with flippers on top of his head, eyes wide, mouth open screaming, sweat drops flying.
```

**19. Sulking — gas override (replacement for crying)** · `UIStatus.OVERRIDE` · 4 frames, loops
```
Animation: "sulking". 4 frames, front view. The penguin crosses his flippers, pouts with his beak pushed out, looks away to the side with his eyes half-closed, and taps one foot. Annoyed but not crying.
```

---

#### Car and device

**20. Seatbelt — seatbelt off / door open** · `seatbeltUnlatched` / `doorOpen` · 6 frames, loops
```
Animation: "buckle up". 6 frames. The penguin wags one flipper at the viewer disapprovingly, then pulls a small grey seatbelt strap across his belly and clicks it into place, then nods.
```

**21. Night owl — it's dark out (lightd)** · `starpilotLightState` dark / auto lights on · 6 frames, loops
```
Animation: "night time". 6 frames. The penguin wears a blue-and-white striped nightcap and holds a small flashlight with a pale yellow beam, slowly sweeping the beam left and right, sleepy eyes.
```

**22. Shades — high beams on (lightd HBA)** · high beam on · 4 frames, plays once
```
Animation: "bright lights". 4 frames, plays once. A bright light hits the penguin: he squints, raises one flipper to shield his eyes, then the last frame he has black sunglasses on and gives a thumbs up with his flipper.
```

**23. Speeding — over the speed limit** · `mapdOut.speedLimit` · 6 frames, loops
```
Animation: "speeding nervous". 6 frames. The penguin looks nervously over his shoulder, sweating, with small alternating red and blue light flashes in the background corner on alternate frames. He tugs at his "collar" nervously.
```

---

#### Idle variations (random when nothing is happening)

**24. Fishing** · 8 frames, loops
```
Animation: "ice fishing". 8 frames. The penguin sits on the ground with a small fishing rod over a round hole in the ice. He waits, the line twitches, and in the last frames he pulls up a small blue fish and looks delighted.
```

**25. Eating a fish** · 6 frames, loops
```
Animation: "fish snack". 6 frames. The penguin holds a small blue fish by the tail, tilts his head back and swallows it whole, then rubs his belly happily.
```

**26. Snowman** · 8 frames, plays once
```
Animation: "building a snowman". 8 frames, plays once. The penguin rolls a snowball, stacks a smaller one on top, adds a carrot nose, then steps back proudly next to the finished little snowman.
```

**27. Juggling** · 6 frames, loops
```
Animation: "juggling". 6 frames. The penguin juggles three white snowballs in an arc above his head, concentrating, tongue out.
```

## Adding a new animation

`strips/` (the generated source images, about 75 MB) is kept out of git because the car downloads this branch. Keep a backup of it: `pack_sprites.py` needs every strip to rebuild the sheet.

1. Save the strip (one row of frames on solid magenta) as `strips/<name>.png`.
2. Add a line to `STRIP_ANIMS` in `pack_sprites.py`: file, frame count, loop or not, a reference frame and how to size Pingu in it:
   - `"h", 1.0` standing (`0.85` sitting): the height of the biggest blob, i.e. Pingu plus anything touching him.
   - `"w", 0.95` lying down: the width of the biggest blob.
   - `"belly", 1.0`: the height of his white belly. Use this when a prop sticks up from Pingu (a sign, a nightcap, raised arms).
3. Run `python pingu_sprites/pack_sprites.py --calibrate calib.png --preview preview.png`. `calib.png` shows each animation's reference frame over a faint idle Pingu, so you can check the size. It rewrites `selfdrive/assets/pingu_spritesheet.png` and `pingu_spritesheet.json` from `base_spritesheet.png` (the original hand-made sheet) plus every strip.
4. Use the name in `_pick_anim()` in `system/ui/widgets/tamagotchi.py`.

Each frame is laid out in a 320 x 288 cell (original-art pixels, with room for props), with Pingu's feet on the same baseline and his black body centered. It is then scaled to 0.75, trimmed and packed tightly into a 2048-wide atlas. `.json` has each frame's atlas rect plus its offset in the cell, and each animation's first frame, frame count and whether it loops. Animations that don't loop stop on their last frame.

`strips/alt/` holds the duplicates and earlier versions that aren't used. That includes the scared versions (`*_scared.png`), which were replaced because Pingu should look confident when the driver takes over or something goes wrong. Swap one in by renaming it.

## Wired up

Checked top to bottom; the first match wins.

| Animation | When |
|---|---|
| melt | `thermalStatus` overheated or critical |
| take_wheel | critical alert (`selfdriveState.alertStatus`) |
| buckle | seatbelt unlatched or door open |
| reverse | reverse gear |
| hockey_stop | `aEgo` below -3 m/s² (plays 2.5 s) |
| referee / watching | driver monitoring alert level two or three / level one |
| slide_left / slide_right | lane change starting or finishing |
| look_left / look_right | pre-lane-change (blinker on, waiting for the nudge) |
| hold_up (mirrored for the right) | blinker on with a car in that blind spot |
| walk_left / walk_right | blinker on (no lane change) |
| steer_fight | engaged and lateral control saturated (held 1 s) or temporary steer fault |
| chill | gas override |
| your_turn | 2 s after disengaging |
| shades | 2 s after engaging |
| high_beam | 2 s after the high beams come on |
| green_light | 2.5 s after StarPilot's green light / lead departing alert, or when the red light it stopped for clears |
| drunk | engaged, moving, `modelV2.confidence == red` (held 2 s) |
| noot | lead under 0.8 s ahead above 5 m/s (held 2 s) |
| speed_demon | over the mapd speed limit by 10% + 1 m/s |
| stop_sign | `starpilotPlan.redLight` or `forcingStop` |
| sleep | stopped for 30 s |
| wave | engaged and stopped |
| night / idle | disengaged and stopped, low beams on / off |
| look_around, giggle, fishing, fish_snack, snowman, juggle | random idle routine every 15–40 s |
| carve | lateral accel over 2 m/s² (`controlsState.curvature` × v²) |
| toboggan | over 29 m/s (~65 mph) |
| scientist / jump | engaged and moving, experimental mode on / off |
| walk_front | disengaged and moving |
