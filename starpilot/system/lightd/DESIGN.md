# lightd: camera auto headlights + high beam assist (Audi TT Mk2)

StarPinguPilot. Status: low-beam estimator tuned offline; HBA prototype offline; vehicle path (lightd → card → mFLA_1 →
panda) implemented and unit-tested but not yet run on the car. See section 10.

The G397 rain/light sensor faces the cargo box, so AUTO lights are wrong all day. The comma four
already has two auto-exposed forward cameras on powertrain CAN. `lightd` turns them into:

1. a low-beam request + RLS-style brightness bucket that the patched J519 uses in place of G397, and
2. a high-beam-permitted decision that drives the stock FLA (Fernlichtassistent) path in the J519,

both carried in the factory HBA camera frame `mFLA_1` / 0x65A.

## 1. Facts this design rests on

### Car side (K-matrix `PQ35_46_ACAN_KMatrix_V5.20.6F`, CentElecRE artifacts)

`mFLA_1` 0x65A, DLC 8, sender FLA camera, receiver Gateway_PQ35. K-matrix: cyclic 1000 ms + on-change,
inhibit 20 ms, gateway timeout 5000 ms (DTC 0xD43 / U107000). No counter, no checksum.
Zero-based Intel bit positions (the K-matrix numbers bytes from 1):

| bit\|len | signal | values | J519 use (per CentElecRE RE) |
|---|---|---|---|
| 0\|8 | void in the matrix → **comma low-beam carrier** (below) | | patched J519 (byte-0 image) |
| 8\|4 | void | | |
| 12\|1 | FL1_KD_Fehler | 1 = DTC stored | |
| 13\|1 | FL1_FLA_aktiv | 1 = active | |
| 14\|1 | FL1_ISP_defekt | 1 = defective | `2CB8.6` → **sticky** fault `15AF.0` (until power cycle). Never set casually. |
| 15\|1 | FL1_Sensor_blockiert | 1 = blocked | `2CB8.7` → `15AF.1` → cluster `BSK_FLA_Sensor_blockiert` |
| 16\|8 | FL1_Abblendursache | bitmask: 1 oncoming, 2 preceding, 4 below speed, 8 town, 16 too bright, 32 fog, 64 overtemp, 128 delay condition, 255 fault | not read by J519 (diagnostic) |
| 24\|7 | FL1_Fernlicht_PWM | 0..100 %, 127 fault | `2CBA&7F`; state 3 output = 2×value; 0 blocks camera demand |
| 32\|6 | FL1_ISP_Lichtwert_vo | 0..61, 62 init, 63 fault | OEM field, we send init 62 |
| 40\|6 | FL1_ISP_Lichtwert_hi | 62 init | |
| 48\|5 | FL1_ISP_Himmelsrichtung | 30 init | |
| 53..55 | ISP_Taste / EC_aktiv / Kompass_aktiv | | |
| 56\|6 | FL1_Dimmwert_ASP | | |
| 62\|1 | FL1_FLA_Aufblenden | 1 = high beam requested | `2CBE.6`, required for camera demand |

Comma light carrier, **final layout** (`claude_handoff_06_finalize_byte0.md`): zero-based **byte 0**, void in the matrix.
- Bits: bit7 valid, bits 6..4 reserved 0, bit3 low beam requested, bits2:0 ambient code 0..7.
- Startup, expired or unavailable: the whole byte is 00.
- Invalid, or 0x65A stale (`10C0.0`), means the J519 falls back to the LIN G397 request with its timers and holds intact.
- The sender must clear valid itself when its data goes stale. The transport timeout only catches a sender that has stopped transmitting.
- **Image gate:** this needs a verified byte-0 J519 image. The reviewed SGO `ACCC6C5C…21B1A` (and the older `comma_light_patch`
  candidate) still read byte 4 and must not be used with this sender. The planned stale-latch patch removes the persistent
  HBA fault from a late comma start; it does not remove normal stale suppression.

J519 HBA: only state `15AE=3` (stalk-armed, AUTO), needs coding byte 11 bit 3 (currently clear),
camera fresh, `Aufblenden=1`, `PWM≠0`, low beam on. The stalk / flash-to-pass handling stays in the J519.

Observable on panda bus 1 (TT is `NO_EXT_CAN`): `Gate_Komf_1` 0x390 `GK1_Abblendlicht` (48), `GK1_Fernlicht` (49),
`GK1_LS1_Fernlicht` (37), `GK1_Wischer_vorn` (50), `GK1_Nebel_ein` (58); `Gate_Komf_2` 0x392
`GK2_Helligkeit` (16|3, ×714.286 lux, i.e. the 3-bit RLS bucket), `GK2_Sta_Lichtsensor`, `GK2_LS_def`.
The light switch position (AUTO) is **not** on A-CAN.

### Camera side (comma four = mici, OS04C10 on both road and wide)

- Narrow (road) cam: 1344×760, f=1141.5 px (~61°×37°). Wide cam: f=425 px (~115°×84°). Each camera runs its own AE loop.
- `FrameData` (`roadCameraState`, `wideRoadCameraState`): 20 Hz, logged to rlog (qlog 1 Hz). Fields used:
  `integLines`, `gain`, `measuredGreyFraction`, `targetGreyFraction`, `exposureValPercent`, `frameId`, `timestampEof`, `sensor`.
- `exposureValPercent` is **linear** in EV between min_ev=2 and max_ev=19992 (`camera_qcom2.cc:243`).
  That puts 1 % ≈ 200 EV units, so all of daylight lives in the bottom percent. Don't threshold it directly,
  and don't use the UI `light_sensor` (`ui_state.py:203`) either.
- `measuredGreyFraction` is the **median** Y over the AE rect (gamma-encoded). Wide rect is a band around/below the horizon
  (y≈227..622 of 760); road rect is nearly the whole frame. No histograms are published.
- OS04C10 gamma: y = 10x/(1+9x), so the inverse is x = y/(10−9y).

## 2. The core measurement: scene brightness in stops

The camera is a light meter once you undo the AE:

```
EV_applied = 2 + exposureValPercent/100 * (19992 - 2)      # EV actually used for this frame (cur_ev[frame_id%3])
lin        = g / (10 - 9 g),   g = measuredGreyFraction
B_cam      = log2(lin) - log2(EV_applied) + K_cam           # stops; +1 = 2x brighter scene
```

- `integLines * gain` is the EV commanded one frame earlier; use the inverted percent to stay aligned with the frame.
- The result stays valid when AE is pinned at max EV at night (grey keeps falling) or at min EV in sun (until grey clips).
  Flag `aeSaturatedLow/High`.
- `K_cam` is a per-camera constant that makes `B` ≈ log2(lux). Fit it once (section 7).

### First replay (42 segments, 2026-04-15 16:40–17:25 MDT, sun 25–32°, daylight only)

- Wide `exposureValPercent` median 0.05 % (p99 0.10 %). The UI light sensor would read a flat ~100.
- AE: integLines 3–17, gain 1.0 → about 11 stops of headroom below this light. Dusk switching levels (~1000 lux) sit well inside the AE range.
  Exposure is quantized to whole lines (0.15–0.4 stop steps), and the grey term absorbs that.
- `B_wide` swings ~3.5 stops with sun/shade. `narrow − wide` differs by up to ±1.5 stops in normal daylight, so the tunnel cue
  has to be relative (rate of change), not absolute.
- The estimator never requested lights. The darkest shade sat ~4.7 stops above the day threshold. G397 read bucket 7 the whole time.
- `EV(perc) / (integLines·gain)` ≈ 1.31, a constant ratio that `K_cam` absorbs.
### Summer replay (108 TT routes, 1428 segments, 2026-05-23 … 06-13, cargo box off, G397 glass cal set low)

G397 is only a one-sided reference: its cal factor was lowered for the cargo box, so it switches on early. If lightd
turns on before G397, that's a lightd bug. If lightd turns on later, it isn't necessarily wrong.

- With K=23, `B_wide` (τ 1 s, moving) vs sun elevation is tight and monotonic. Medians:

  | sun elevation | `B_wide` median |
  |---|---|
  | 30–90° | 15.3–16.3 |
  | 10–15° | 11.5 |
  | 4–6° | 9.9 |
  | 0–2° | 8.5 |
  | −4..−2° | 6.4 |
  | −6..−4° | 4.2 |
  | night | 2.2–4 |

  Dusk p5–p95 spans about 2 stops. At night AE is pinned at max EV and B floors near 2.2.
- G397 switched on around sun 15–22° (`bf`, `ce`, `ef`) and in storm-dark afternoons (`df`, `e0`–`e3`, `f0`, sun 20–35°,
  slow `B` 9.4–10.6). Most "off" edges are key-off.
- Daytime dips:
  - one real tunnel (`b8`): −8.2 stops to 7.9, 20 s, 321 m;
  - one 8 s / 275 m dark stretch (`bc`);
  - highway underpasses: −6 stops, 0.4–1.0 s, 16–34 m;
  - ordinary shade: never pulls a 20 s average below ~11.5 on a clear day.
- camerad AE needs ~5 s after start to converge (reads ~14 at dusk at first).
- `GK2_Helligkeit` buckets 1–6 span only ~1.2 stops of camera B (9.3–10.5). With the altered cal it's not a lux scale.

## 3. V1: low-beam estimator (no pixels), as implemented in `ambient.py`

Two channels share one absolute threshold `T = 9.8 + sensitivity` with hysteresis 0.8 stop:
- **slow** (EMA τ 20 s, frozen while the tunnel channel is latched): dusk, dawn, storms. Shade never lasts long enough to pull it down.
  On clear evenings it crosses T at sun ≈ +5°, about 30 min before sunset. G397 crossed at ~15–22°.
- **fast** (EMA τ 0.3 s down, 1 s up): tunnels and garages. It latches only when fast < T **and** fast ≤ slow − 3 stops, held for
  min(2 s, 60 m). That ignores underpasses and dusk shadow flicker but catches the logged tunnel within ~2 s. It releases after 1 s above T + H.
- **night latch**: sun below 0° (sunset to sunrise, Colorado law) means dark, whatever the camera says.
- Not valid for 6 s after camerad starts (both channels track raw while AE converges). The J519 stays on G397 until then.

Replay results:
- clear daytime routes: 0 activations;
- night: 100 %;
- `b8` tunnel and the `bc` stretch: on;
- dusk routes at sun ≤ 4.5°: 100 %; 5–7° is the edge;
- storm afternoons: off. `sensitivity = +0.5` would catch them, if that's wanted.

Original V1 plan, kept for reference (superseded where it conflicts with the above):

Inputs at 20 Hz: `B_wide`, `B_narrow`, solar elevation `e` and azimuth, `vEgo`, GPS bearing, `lowBeamOn` (car), validity.

Inputs at 20 Hz: `B_wide`, `B_narrow`, solar elevation `e` and azimuth, `vEgo`, GPS bearing, `lowBeamOn` (car), validity.

1. **Solar position** (new `solar.py`, NOAA algorithm, offline, no network). Lat/lon come from
   `gpsLocation(External)` (via `common/gps.py`) or the cached `LastGPSPosition`; time comes from the system clock
   when `system_time_valid()`. A position off by a few hundred km only moves sunset by minutes, which is fine.
2. **Regime** from `e` (all thresholds tunable):
   - `e < −4°`: **night latch**. Dark is requested no matter what the camera says, so gas stations and streetlights can't turn lights off.
   - `−4° ≤ e ≤ +8°`: **twilight**. The camera decides.
   - `e > +8°`: **day**. Thresholds shift darker by `D` stops, so it takes a tunnel, garage or real storm.
   - No solar data: camera only, lower confidence.
3. **Filter** in log space, asymmetric: toward dark τ≈0.3 s (tunnels), toward light τ≈1.5 s. Keep it fast. The J519 hold
   (`2589`×10) and Audi's delays sit downstream.
4. **Decision with hysteresis**: on when `Bf < T_on(regime)`, off when `Bf > T_on + H` (H≈1 stop). Per the RE,
   the J519 takes our on/off bit as the decision (like G397's `34B2.0`) and only adds hold time. So the
   hysteresis has to live here.
5. **Self-illumination**: when `lowBeamOn`, subtract a learned δ. Each time the lights toggle we get a free
   measurement of δ. Start with δ=0; at the dusk switching level it's small, but it matters in garages.
6. **Bucket** for byte 0 bits 2:0: `clamp(floor(lux_est / 714.286), 0, 7)`. The 714 lux/step scale comes from
   `GK2_Helligkeit`, so it's a hypothesis: verify against GK2 on a box-off drive. The J519 only uses the bucket for coming/leaving-home vs `2586`.
7. **Validity → `valid` bit**: both camera states alive and fresh (<200 ms, frameId advancing), sensor known,
   AE settled (>2 s after camerad start), process healthy. If the wide camera is disabled (`DisableWideRoad`), fall back
   to narrow only. Anything else clears valid and the J519 uses G397.
8. **Free tunnel cue**: `r = B_narrow − B_wide`. The narrow center sees a tunnel mouth before the wide band does.
   V1 only logs `r`; V2 uses it.

Also log, without using it yet: the sun's azimuth relative to heading (driving into a low sun reads brighter than
a skyward RLS would), and `GK1_Wischer_vorn`.

## 4. V2: pixel ROI statistics + tunnel prediction

`VisionIpcClient` on WIDE_ROAD and ROAD (template: `starpilot/system/speed_limit_vision.py:1162`, `adj_spot_monitor_vision.py`).
Use a strided Y view (`y[::4, ::4]`) at 5–10 Hz and nice=19. Cost is well under a ms per frame.

- ROIs: **sky/upper** from wide (closest analog to the RLS, and not lit by our own lights), **road/center** from narrow,
  **whole scene**. Check one real frame first so the sky ROI excludes the windshield frame, roofline and cargo-box lip.
- Features per ROI: median, p5/p95, clipped fraction, then convert each to stops with the same EV math.
- Tunnel/garage: a fast drop of the narrow road ROI relative to the wide sky ROI, with a dark-core blob near the
  projected path's vanishing point, gives a `tunnelScore` that pre-arms dark before the portal.
  Plain shade drops everything together; a tunnel shows strong `forward/global` contrast first.

## 5. V3: HBA, a separate classifier

Output: `highBeamPermitted` + reason mask, mapped 1:1 onto `FL1_Abblendursache`.

Gates first, all cheap:
- **too bright (16)**: V1 says not dark, or `e > −4°`.
- **below speed (4)**: `vEgo` under ~60 km/h on / 45 km/h off.
- **town (8)**: streetlight density, plus mapd speed limit ≤ 50 if available.
- **overtemp (64)**: device thermal or lightd throttled.
- **error (255)**: invalid inputs.

Detection:
- **Preceding (2)**: `modelV2.leadsV3[0].prob` gives a lead out to ~100+ m almost for free, and it's trained at night.
  For farther taillights, look for red blobs (high V in NV12) in the narrow camera.
- **Oncoming (1)**: the model has nothing for this, so it's custom. Detect bright blobs (`cv2.threshold` + `connectedComponentsWithStats`)
  in narrow (far, 0.05°/px; a 1.5 m headlight pair at 600 m is about 3 px) and wide (near and periphery).
  Search only a **path-projected corridor**:
  - project `modelV2.position` / `laneLines` / `roadEdges` (to ~190 m) with `liveCalibration` (`rpyCalib`, `wideFromDeviceEuler`, `height`)
    and `DEVICE_CAMERAS[("mici","os04c10")]`;
  - copy the math from `selfdrive/ui/mici/onroad/augmented_road_view.py:942`;
  - extend the corridor along the curvature and widen it on the oncoming side (a param for left/right-hand traffic) and on crests.
- **Tracking**: a small nearest-neighbour tracker. Static lights (streetlights, reflectors) move exactly as ego-motion
  predicts from `vEgo` and image position; vehicles don't. Reflectors and signs brighten when *our* high beam turns on, so that gives a cue too.
- **Fog (32)**: later. Global low-contrast brightening when our high beam turns on means backscatter.
- **Asymmetric state machine**:
  - Drop immediately (about 2 frames) on any plausible detection or invalid input.
  - Restore only after the scene has been clear for T_clear (~2 s, scaled by speed). While waiting, send reason 128 (delay condition).
  - Never assume clear when data is missing.
- **Blocked**: `FL1_Sensor_blockiert=1` only on sustained evidence (frozen or uniform image, huge narrow/wide disagreement).
  Never set `FL1_ISP_defekt` (sticky fault in the J519).

Later, ML: crop blob patches from logged video and train a tiny classifier (oncoming, tail, street, reflector, sign).

### Prototype (`hba.py`), first offline pass

Data: the only night routes with video are `dc`, `dd`, `de` (2026-05-27, sun −8°, about 8.5k narrow-camera frames). They're forest
and residential roads, never above 47 km/h, with no manual high-beam use. So this checks detection only. The speed gate keeps the
beam low for the whole drive, and the 60+ km/h behaviour is untested.

What the pass covered:
- **Geometry.** The corridor follows the model lane lines, extended straight past the model horizon. It runs from 15 m out to 600 m, and
  vertically from 0.2 m to 1.6 m above the road. Starting at 8 m with a 2.2 m top made it cover half the frame.
  Projection is K · view_from_device · R(rpyCalib) · (x, y, z + height), with model y pointing right. That matches the UI.
- **Colour.** Red is a hue test (≤ 8 or ≥ 172) plus a bright core (V ≥ 190). A plain Cr threshold also took yellow diamond
  signs and warm house windows. Without the core requirement, red paint lit by our own low beams got through.
- **Monocular scale.** Distance comes from assuming the lamp sits 0.7 m above the road. A reflector on a 1 m post therefore gets an
  inflated distance and a speed off by roughly 2–3×, so motion alone can't mark it static. Beyond ~70 m one pixel row is tens of metres,
  so there's no usable motion at all.
  - Fix 1: the off-path *angle* (lateral/distance) is scale-free. Small lights more than 3.3° to the right (our side) are roadside:
    reflectors, parked cars, driveway markers. Big white blobs are exempt, because they're cars pointing at the road.
  - Fix 2: motion labels (static, oncoming, moving away) apply only under 70 m. Anything farther stays a candidate, which dips.
- **Persistence.** Weak blobs need 8 frames (0.4 s) before they count, the same window the motion fit needs. Big or saturated blobs
  dip immediately. Restoring needs 2 s clear.
- **Result above 36 km/h:** threat frames are 0.8 % (`dc`), 1.6 % (`de`) and 17 % (`dd`, which had real traffic: a driveway car with
  its lights on, a car parked on the left shoulder, distant lights at the end of the road). Parked cars and reflectors on our side
  are labelled static.

### Second pass (2026-10-17): replay harness, hand labels, fixes

**Harness.** `tools/replay_hba.py` replays pingulink route dirs (`rlog--N.zst` + `fcamera--N.hevc`) through the classifier at
`HBA_PERIOD_S`. It feeds the model, calibration, speed, `livePose` gyro and `GK1_Fernlicht` at each frame and decodes the video to
NV12 with ffmpeg. It writes per-frame and per-track CSVs (each track carries the rule that set its kind, `why`), and annotated
JPEGs at every dip/restore, every `--jpg` s or `--at seg:t0-t1`. `--labels` scores against hand labels
(`route,seg,t0,t1,vehicle|none|ambiguous`): vehicle-time threat rate, glare (permitted while a vehicle is present), missed runs
≥ 1 s, and nuisance (threat while nothing is there). `--clf other_hba.py` compares against a saved copy. The hindsight auto-labels
(near-range motion) are weak: a car followed at 200 m never resolves, so trust the hand labels and the JPEGs.

**New data.** `c9` (2026-05-27 03:42Z, sun −5°, 70–88 km/h, two-lane mountain road) segments 3, 5, 6, hand-labelled at 2 s:
80 s with a vehicle, 70 s with none. Lots of delineators, lit green/yellow signs, orange chevrons, a following car most of seg 3.
`c5` (sun +2°) also has video. `dc/dd/de` stay the slow regression set.

**Bugs found and fixed:**
- **The corridor sat 1.2 m too low whenever lanes were visible.** `laneLines` z is already road level (~+1.14 m below the camera
  at 20 m); only `position` (the device's own path) needs `+ height`. The UI does the same (lanes z_off 0, path z_off height). At 50 m
  that was ~27 px, so real lamps sat on the corridor's top edge. `road_centerline` now returns road-level z.
- **Frame-count timing.** confirm/motion/dip/missed were frame counts written for 20 Hz; the worker runs at ≤ 10 Hz, so confirm took
  0.8 s. Everything is in seconds now (`confirm_s`, `motion_s`, `dip_s`, `max_missed_s`, `pair_window_s`).
- **Wet/glare-smeared far headlights were "posts".** The far oncoming car at c9/5 6–8 s was two tall blobs, so each one got marked a
  delineator and static. Tall blobs may now pair (within 5× their height), and a paired blob is never a post.
- **Lit yellow signs passed the white test** (saturated yellow has luma ≈ 226), and orange chevrons passed the red test. Blobs whose
  pixels are saturated yellow/green, or that have more saturated yellow around them than their own area, are dropped.

**Changes:**
- Range comes from the ray's first crossing of lamp height over the model's road profile, in either direction, so lamps above
  the camera horizon on an uphill road are ranged. The flat plane is gone. Lights whose ray never passes through the lamp band
  between 15 and 600 m, or only within 30 m while small, are `high` (buildings, signs and streetlights let in by the margin).
- Past the model horizon the corridor widens by 0.02 m/m sideways and 0.01 m/m up and down.
- A red-only band one lane wider on the far side when the model sees a lane there (`laneLineProbs[3]`). Red lights skip the
  roadside rule when far (> 70 m, where the path lags curves) or when that lane exists; close by on a two-lane road they're parked cars.
- A big white blob is "a car pointing at the road" only within 8 m of our lane centre; farther out it's a lit sign.
- A white light "moving away" sits lower than a lamp (pavement markers, low reflectors): static.
- Static voting: a track that looked static for half of the last second stays static unless it shows vehicle evidence (oncoming
  motion, a strong lamp pair). Single frames flip between rules, and each flip to a threat was a dip.
- Restore backoff: a restore cut short within 3 s by weak evidence doubles the clear time (2 → 4 → 8 s). It resets after 10 s
  permitted. A model lead, a strong lamp or oncoming motion doesn't count.
- Gyro (`livePose.angularVelocityDevice`): tracks are predicted through the rotation since the last frame. The sign was checked
  on c9: median prediction error 4.0 px with it, 4.8 px without, 6.8 px flipped. (This pass also rotated the projection by a
  high-passed gyro integral. That was wrong and is gone, see the first drive below.)
- High-beam feedback (`starpilotCarState.highBeamOn`): a track whose brightness rises ≥ 1.8× within 0.15–0.6 s of our high beam
  switching on (or falls as much after it switches off) is a retroreflector for good. So is an unpaired light that first appears
  within 0.4 s after switching on. Untested on real data: none of the logs has the high beam on.

**Results** (c9 hand labels, speed gate off; baseline is the first-pass classifier):

| | baseline | now |
|---|---|---|
| vehicle present: threat | 91.2 % | 95.9 % |
| vehicle present: high beam permitted (glare) | 2.2 % | 0.0 % |
| vehicle present: missed runs ≥ 1 s | 3.8 s | 0 s |
| nothing present: threat (nuisance) | 37.6 % | 20.4 % |
| nothing present: high beam permitted | 31.6 % | 42.1 % |

`dc/dd/de` (slow, unlabelled): threat frames 14.7 → 12.1 %, dips 3.4 → 2.8 /min, permitted 66.5 → 69.9 %. Every restore moment was
checked by eye: no vehicle in view. Desktop cost is ~2 ms/frame, about the same as before.

**Tried and dropped:** the brightness-vs-distance exponent (lamps ∝ d⁻², reflectors lit by us ∝ d⁻⁴). A constant range-scale
error doesn't change the slope, but near the horizon the per-frame range within one track swings 50–400 m, and the fitted exponents
on c9 spread from −1 to 2.5 with no separation. It would need a much better range (pairs, or a longer baseline).

**PWM:** `FL1_Fernlicht_PWM` doesn't dim the TT's bi-xenon high beam (it's a shutter, per the owner), so there's no soft fade-in.
Send 100 or 0.

Known gaps and next steps:
- 70 s of labelled empty road and 80 s with vehicles is a small set. Nuisance is still 20 %, mostly retroreflectors on our side
  past the model horizon (curves), lights at junctions and town edges, and far lights with an unreliable range. More labelled
  night drives at speed (pingulink on the device) are the bottleneck, not compute.
- The high-beam feedback cue needs logs with the high beam on: shadow mode plus manual high beams.
- Track vehicles as lamp pairs rather than single lamps, then a learned per-track score (GBDT on the CSV features) once there are
  enough labels.
- Town gate: see below.
- The wide camera only feeds a glare guard for curves (see "Wide-camera glare guard" below).
- Fog/backscatter needs data with the high beam on.

### Town gate (2026-10-17)

`REASON_TOWN` (8) blocks the high beam like the speed gate. Two cues, OR'd:
- **Streetlight density.** Each frame counts bright (Y ≥ 180), unsaturated lights outside the corridor above the row of a
  point 3 m over the road 150 m out. Above that row a light is higher than 3 m or nearer than 150 m, so it isn't a
  vehicle's lamp. Lit signs that get in (near ones) are dropped by colour: yellow is saturated, and green has hue 38–100.
  The band is labelled 2×2 max-pooled (a light's peak survives it), then each light is measured at full resolution.
  That's ~1 ms on desktop with 30 lights in view and much less on an empty road. Town starts when the mean over 4 s is
  ≥ 2 lights per frame, and ends 10 s after that mean drops below 0.5.
- **Posted limit ≤ 50 km/h** from `mapdOut.speedLimit`, only when mapd matched the way (`waySelectionType` current or
  extended). lightd reads `mapdOut` itself, because `starpilotPlan.slcMapSpeedLimit` is only filled while SLC is on.
  Untested: every log with video predates StarPilot, so none has `mapdOut`.

Data (light counts per frame): the c9 mountain town peaks at 6.6, the open road in c9 segs 5–6 at 1.0 (one lone yard
light or streetlight at a time), and c9 seg 4 (a stop and a junction at the town's edge) at 1.2. Height alone
doesn't separate them: lit green guide signs reach 5–12 m on the 150 m line, while the town's windows are at 6–7 m.
Colour does: town lights are warm and pale (hue 11–35, saturation 20–75).

| replay (speed gate off) | before | after |
|---|---|---|
| c9 3–6: town | — | 10 s, c9/3 50–60 s only |
| c9 3–6: vehicle present: threat / glare | 95.9 % / 0.0 % | same |
| c9 3–6: nothing present: permitted | 42.1 % | 37.3 % (c9/3 51–60 is town, 12.9 % of "none") |
| dc/dd/de: town | — | 113 of 428 s (lit residential streets) |
| dc/dd/de: permitted | 69.9 % | 54.5 % |

The replay starts a new classifier per segment, so the town state doesn't carry from c9/3 into c9/4.

Known gaps:
- The moon, or a lit town across a valley, counts like a streetlight. One light alone stays under the threshold, but it
  plus a lone yard light gets close. Static far lights could be rejected by their lack of parallax (gyro-predicted
  position), if it shows up in data.
- An unlit village doesn't register at all, which is why the mapd cue matters.
- The thresholds rest on one town and three residential drives.

### First drive (2026-10-17, route `00000046`, 34 min at night, mode 3)

HBA worked but flashed several oncoming cars at close range, mostly in turns, and stayed low for reflective signs.
Video + rlogs are in `carstuff/logs/00000046--c6859cdf72` (flat layout for `replay_hba.py`).

**`starpilotLightState` wasn't logged at all.** `cereal/services.h` is generated but committed, and it was stale (no
`starpilotLightState`), so loggerd skipped it. Regenerated; `starpilot/system/tests/test_services_h.py` now fails if it drifts.
Until the device rebuilds loggerd, the drive can only be read from what we sent (0x65A in `sendcan`) and `GK1_Fernlicht`.

**Turns: the corridor was swung off the road.** The second pass subtracted a high-passed (1.5 s) integral of the gyro from
the calibration before projecting. On a bump that's a short pitch blip, but in a sustained turn the yaw integral settles at
yaw rate × 1.5 s, about 13° at 0.15 rad/s, and the corridor slid to the outside of the curve, away from oncoming cars. The model's
road is already in the current frame's calibrated coordinates, so there is nothing to correct. The gyro now only predicts tracks.

**Halogen headlights were dropped as yellow signs.** The "saturated yellow around a white blob" sign test (second pass) also
matches the warm halo of a halogen lamp (yellow ≈ 1.7× the core's area). For white blobs it now needs 4× (`sign_frac_white`).

**Lamps bloom, retroreflectors don't.** Lit white signs, route markers and post reflectors have cores as neutral as headlights
(saturation 14–86), so colour can't split them. Their surroundings can: the median V of a blob-sized ring around the core is
13–24 for signs and reflectors on route 46 and 41–128 for headlights. White blobs of 16 px or more with a ring median under 32
are dropped (`crisp_ring_v`). Smaller (far) blobs overlap, so they're left alone.

**A big white lamp on the oncoming side is a car.** Motion and votes misjudged close oncoming lamps (two lamps swapping, wet-road
reflections) as static. A strong, unpaired-or-paired, non-post white track no more than 1° to our side of the path can't be
static any more (`why = "glare"`). The "born right after our high beam came on" reflector rule skips strong blobs.

**Measuring glare without labels.** `replay_hba.py` now records the biggest blob per frame that is saturated, near-neutral, blooms
(ring median ≥ 40) and sits in the left two thirds of the band, ignoring the classifier ("obvious lamp", ≥ 60 px). It's a
crude stand-in for "an oncoming car is right there"; time with the high beam permitted while one is in view is the glare number.

| route 46 segs 9–21 (13 min, replay) | as driven | now |
|---|---|---|
| high beam permitted while an obvious lamp is in view | 7.0 s of 50 s | 0.8 s |
| high beam permitted | 36.8 % | 41.1 % |
| dips per minute | 4.2 | 3.3 |

| c9 hand labels | second pass | now |
|---|---|---|
| vehicle present: threat / glare | 95.9 % / 0.0 % | 96.0 % / 0.0 % |
| nothing present: threat / permitted | 20.4 % / 42.1 % | 14.4 % / 45.9 % |

What's left of the 0.8 s is the first frame a car appears, plus one 0.3 s case of a car entering at the edge of the road camera
in a tight curve. The wide camera (115°) sees those earlier; it's the next step for turns. Most remaining long dips at speed are
taillights 150–300 m ahead, which is correct.

### Wide-camera glare guard (2026-10-17)

In a tight curve an oncoming car is outside the narrow camera (61° across), or cut by its edge, until it is close. On
route 46 seg 14 the wide camera has the car at 25.3 s, 2.5 s before it reaches the narrow view's left edge. The guard
(`wide_lights()`, `HbaClassifier._wide_update()`) has no tracks or motion. Every `WIDE_PERIOD_S` (0.2 s), in the dark, it
looks at the newest wide frame (`VISION_STREAM_WIDE_ROAD`, within 2 frames of the narrow one), and any hit dips at once
with `REASON_ONCOMING`:
- **Geometry:** `liveCalibration.wideFromDeviceEuler` on top of `rpyCalib`, with the wide intrinsics treated as a pinhole
  like openpilot does (1344×760, f = 567 × 3/4).
- **Where:** a corridor from the lane centre 7.5 m towards oncoming traffic, 6–300 m out, at lamp height, minus the narrow
  view inset by 40 px. Inside the narrow view the narrow pipeline knows better: it has 2.7× the resolution, tracks and motion.
- **What:** Y every other row and column (the strided mask is projected directly, so nothing is computed at full
  resolution). A light counts if it has Y ≥ 225, at least 6 strided px, chroma |U−128|+|V−128| ≤ 40, h/w ≤ 2.5, and blooms:
  the ring median Y is ≥ 32, the same test as `crisp_ring_v`.
- **Reach ≥ 10 m:** the farthest distance at which its ray is at lamp height inside that band must be at least 10 m.
  Close by, the corridor's lamp band reaches up to the horizon row, where a yard light 200 m off to the side sits too.
  Those, and cars already passing beside us (our beam doesn't reach their driver), have a reach of 7–10 m. Cars in curves
  have 12–18 m (the model's road stops short of where they are).
- **Hold:** a hit holds the threat for 0.3 s, so the 10 Hz decision doesn't flicker between looks.

Replay: `replay_hba.py` decodes `ecamera--N.hevc` alongside the narrow video and checks it against `wideRoadEncodeIdx`.
It writes every light the guard looked at, with its tests, to `<out>_wide.csv`, and saves a JPEG with the wide frame
alongside at every onset (`_wide`). `--no-wide` replays without the guard.

Cost: 0.85 ms per look on desktop (p95 1.2 ms, seg 14 and 20), 3.0 ms on the device's A55, five times a second. `DisableWideRoad` turns it off, and
so does a missing `wideFromDeviceEuler`. Without wide frames the HBA is unchanged.

| route 46 replay, same narrow pipeline | wide guard off | on |
|---|---|---|
| high beam permitted while an obvious lamp is in view, whole route | 1.0 s | 0.6 s |
| ...at ≥ 50 km/h | 0.5 s | 0.3 s |
| ...segs 9–21 | 0.5 s | 0.3 s |
| high beam permitted at ≥ 50 km/h (24 min dark) | 305.7 s | 301.8 s |
| dips, whole route | 41 | 39 |

Without the reach test, glare was 0.3 s but the guard cost 7 s of high beam, mostly on yard lights in seg 20. The 0.3 s
left at speed is the first frame of three cars appearing inside the narrow view (seg 9 21.8 s, seg 11 7.4 s, seg 13 37.5 s,
the last next to a lit yellow sign), which the wide camera can't help with. The 0.3 s below 50 km/h is a parking lot
(seg 0 51.8 s, 7 km/h), where the speed gate keeps the high beam off anyway. Most of the 3.9 s cost is real cars: one in seg 14
seen earlier, and cars passing at 12–15 m whose dip now lasts until they're beside us. One yard light in seg 20 (reach 11 m)
is the rest.

c9 has no wide video (`ecamera`), so it's unchanged: vehicle threat 96.0 %, glare 0.0 %, nuisance 14.3 %. Route 46 is the
only night drive with wide video so far, so the thresholds rest on its 34 minutes.

## 6. Interfaces, as implemented

Data flow:
```
camerad ─┐
modelV2, liveCalibration, GPS ─┼─> lightd ──starpilotLightState (20 Hz)──> card ──CS.fla_command──> carcontroller ──mFLA_1 10 Hz──> panda ──bus 1──> J533 ─> J519
carState, starpilotCarState ───┘                       (freshness + mode)          (fla.py packs it)          (TX allowlist + checks)
```

### cereal
- `StarPilotLightState` sits in the reserved slot `CustomReserved12` (id kept), as event `starpilotLightState @138`.
  Service: `(True, 20., 20)`, so rlog gets 20 Hz and qlog 1 Hz.
- Fields cover:
  - the low-beam decision and its inputs (all log2 lux);
  - sun position;
  - the HBA decision with reason, counts, and processing time;
  - camera frame ages.
- `StarPilotCarState` gained `lowBeamOn`, `highBeamOn`, `highBeamStalk`, `frontWipersOn`, `lightSensorBucket`
  (G397 code from `GK2_Helligkeit`, 255 = n/a) and `lightSensorFault`. They're filled in `carstate.update_pq` only for
  `NO_EXT_CAN` (the TT), because `Gate_Komf_2` is a lazily added message.

### lightd (`starpilot/system/lightd/lightd.py`)
- **Loop:** polls `wideRoadCameraState`, or `roadCameraState` when `DisableWideRoad` is set. Camera samples only count while
  `frameId` advances, and the estimator drops anything older than 200 ms.
- **Clock:** the camera messages' own `logMonoTime`.
- **Sun:** live GPS, falling back to `LastGPSPosition` up to a week old, plus the wall clock once `system_time_valid()`.
- **HBA:** runs on its own thread (`HbaWorker`) at up to ~10 Hz on the newest narrow VisionIPC frame. The loop hands it
  the newest context and reads the newest result without waiting, so a slow HBA never slows the low beam.
  - The frame stays NV12. Only the corridor's bounding box is labelled, and only small boxes inside it are converted to
    BGR/HSV: around 2x2 blocks whose brightest Y reaches a per-chroma table (the lowest Y at which any pixel with that
    chroma pair passes the white or red test, built once from OpenCV's own conversion, ~0.7 s on the A55), each grown by
    its own size for the halo/ring tests. The rest stays 0, which passes nothing, so the blobs are the same as converting
    the whole box. OpenCV runs on one thread.
  - CPU (comma four, A55 core, 240 real night frames from route 46): ~24 ms/frame before the 2026-10-17 optimisation pass,
    now 9.2 ms median on rural road with oncoming cars (seg 9, was 18.6), 13.3 ms in town (seg 2, was 21.7; p90 21 ms, was
    43), 6.1 ms on curves (seg 14, was 14.7). The wide guard: 3.0 ms per look (was 6.4). Decisions unchanged.
  - The HBA thread pins itself to cores 0-3, next to locationd/paramsd, so its ~10-14 % of one A55 never competes with
    card/controlsd (4), plannerd/radard (5) or modeld (7) on the big cores.
  - Message `valid` is alive + valid of the poll camera. Its average-rate check is ignored, because it measured this
    loop: at 100 ms per inline HBA frame the loop fell to ~9 Hz, `valid` went false, and card dropped the low beam too.
  - It runs in mode 1 (shadow) and mode 3. It needs `calStatus == calibrated`, `modelV2` within 5 frames of the image, and `carState` alive.
  - It also gets `livePose.angularVelocityDevice` (gyro, for tracking and pitch), `starpilotCarState.highBeamOn` (the
    reflector cue) and `mapdOut.speedLimit` (the town gate). Without them it runs as before: no compensation, no beam
    cue, town from streetlights only.
  - In the dark, every 0.2 s it also pulls the newest wide VisionIPC frame (a second client) for the glare guard, unless
    `DisableWideRoad` is set. It only uses that frame if it's within 2 frames of the narrow one and the calibration has
    `wideFromDeviceEuler`. The wide camera never affects `hbaValid`.
  - `hbaValid` drops 0.3 s after the last processed frame, or at once if the worker thread died.
- **Process:** `PythonProcess("lightd", ..., run_lightd, nice=10)`. It runs onroad when `volkswagen_lightd_mode >= 1`,
  and that toggle is only non-zero on PQ cars.

### card (`selfdrive/car/card.py`)
- `lightd_tx` is true when the PQ safety config latched `PQ_LIGHT_TX` at car start. Only then does card subscribe to `starpilotLightState`.
- Each control step it builds the two commands:
  - `LightCommand(valid, headlight_on, brightness)` when mode ≥ 2 and `lowBeamValid`;
  - `HbaCommand(active, permitted, reason, blocked)` when mode ≥ 3 and `hbaValid`.
- If the message isn't alive (0.5 s for a 20 Hz service) or isn't valid, both commands are revoked.
- Lowering the mode mid-drive takes effect at once.

### CAN (`opendbc/car/volkswagen/fla.py`, `vw_pq.dbc`, carcontroller)
- **DBC:** `BO_ 1626 mFLA_1` holds the K-matrix OEM fields as raw values.
- **Low-beam carrier:** packed after the DBC fields by `apply_light_carrier()`, into byte 0 (final layout). Bytes 1..7 are
  left exactly as packed. `LightCommand` rejects non-bool flags and brightness outside 0..7. card sends invalid
  rather than raising if lightd ever produces a bad code.
- **HBA fields:**

  | state | `FL1_FLA_Aufblenden` | `FL1_Fernlicht_PWM` | `FL1_Abblendursache` | `FL1_FLA_aktiv` |
  |---|---|---|---|---|
  | permitted | 1 | 100 | 0 | 1 |
  | dipped | 0 | 0 | lightd's reason bits | 1 |
  | off or stale | 0 | 0 | 255 | 0 |

  `FL1_ISP_defekt` is never set. `FL1_Sensor_blockiert` follows lightd (nothing sets it yet).
- **ISP mirror fields:** "no mirror": values at init (62 / 62 / 30), EC/compass/button flags 0, `FL1_Dimmwert_ASP` 0.
- **Sending:** the carcontroller sends every 10 frames (10 Hz) whenever `PQ_LIGHT_TX` is latched.
  - Without a command from card it sends invalid / HBA-off, so the J519 drops back to G397 immediately instead of after the 0x65A timeout.
  - Like all openpilot CAN, nothing is sent before controls start or in dashcam mode. The J519 then times out to G397.

### panda (`opendbc/safety/modes/volkswagen_pq.h`)
- `FLAG_VOLKSWAGEN_PQ_LIGHT_TX` is bit 9 (512). Python sets it in `interfaces.py` only for `NO_EXT_CAN` cars whose
  `VolkswagenLightdMode` is ≥ 2 at car start.
- With the flag, the bus-1 TX tables add `{0x65A, bus 1, len 8, check_relay = false}`. With relay checking on, a factory FLA
  camera sending 0x65A would kill steering.
- The TX hook blocks frames with `FL1_ISP_defekt` set or `FL1_Fernlicht_PWM > 100`.
- Tests are in `test_volkswagen_pq_pingu.py::TestVolkswagenPqLightSafety`. The bus-1 suite without the flag still proves 0x65A is blocked.
- `common.py` gets one exception: 0x65A falls inside ELM327's diagnostic range.

### params / UI
- `VolkswagenLightdMode` (INT, default 0): 0 Off, 1 Shadow (log only), 2 Low Beam, 3 Low + High Beam.
- It's a PQ selector next to the HCA ones in both settings UIs (via `fla.LIGHTD_SETTINGS`) and is listed in `SAFE_MODE_MANAGED_KEYS`.
- **Going from 0/1 to 2/3 needs a car restart**, because the panda bit is latched at start. Lowering it is live.

### code layout
```
starpilot/system/lightd/
  lightd.py        daemon (IO)
  ambient.py       AmbientEstimator (pure)
  hba.py           HbaClassifier (pure + cv2)
  solar.py         NOAA sun position
  tools/replay_lightd.py
  tools/replay_hba.py     HBA over logged video (+ wide video for the glare guard), hand-label scoring, classifier-independent
                          glare check, annotated frames
  tools/bench_hba.py      per-stage timing on the device
starpilot/system/tests/test_lightd_{ambient,hba,solar}.py
opendbc/car/volkswagen/fla.py (+ tests/test_volkswagen_fla.py)
```

## 7. Logging, labels, tuning

- Everything above goes into `starpilotLightState` at 20 Hz. Raw CAN is logged too, so GK1/GK2 can be decoded offline as well.
- **Use what's already on disk.** `carstuff/logs` has 42 rlog segments with `fcamera`/`ecamera` video. Camera states +
  CAN + GPS are enough to develop V1 entirely offline, and the video covers V2/V3 prototyping.
- **Box-off calibration drive**: dusk/dawn plus a tunnel, AUTO on, no cargo box. Then G397 is unobstructed:
  `GK2_Helligkeit` + `GK1_Abblendlicht` are Audi's own labels and thresholds. Use it to fit `K_cam`, `T_on`, `H` and the bucket map.
- **Free HBA labels**: whenever the driver uses the high beam manually (`GK1_LS1_Fernlicht` / `GK1_Fernlicht`), the dips mark real threats.
- **Explicit labels**: a tap target on the onroad UI ("lights should be on/off") written into the label fields, plus the
  existing bookmark-via-button (`BUTTON_FUNCTIONS["BOOKMARK"]`). Do bulk labeling offline in a notebook over qcamera.
- **Mismatch log**: if we request on while `GK1_Abblendlicht` stays off past the hold, either the switch isn't in AUTO or the J519 path is broken.
- Offline: `replay_lightd.py` on LogReader for quick iteration, plus a `process_replay` config for regression.

## 8. Fallbacks (non-negotiable)

- Low beam: any doubt → `valid=0` → the J519 uses G397. If openpilot dies, there's no TX, and the 0x65A timeout falls back too.
- HBA: any doubt → `Aufblenden=0, PWM=0`. Never "assume clear".
- Startup: invalid until both cameras have settled. The J519 starts on G397 (the patch marks 0x65A stale at boot).

## 9. Open items / risks

1. **Gateway routing on the TT.** The K-matrix routing sheet doesn't list `mFLA_1` for the TTNF column. Bench-test before
   anything else: inject 0x65A on A-CAN and watch the comfort CAN / J519 measuring values.
2. **Gateway passage of byte 0.** The final carrier is in byte 0, which is void in the matrix. If J533 rebuilds 0x65A from
   K-matrix signals instead of copying the frame, byte 0 arrives as 00 = invalid. That fails safe (the J519 stays on G397),
   but the feature would silently never engage. Bench check: does byte 0 reach the J519 intact?
3. The 0x65A stale timeout is 500 ticks of `7DE490`. 5 s is likely (it matches the K-matrix value), but that's unproven.
   The planned stale-latch patch only removes the persistent HBA fault after a late comma start; normal stale suppression
   (comma dies, so the J519 returns to G397 and HBA output stops) stays. Don't depend on the comma starting within 4.5 s.
4. FLA coding (byte 11 bit 3) may raise cluster warnings / DTC U107000 while the comma is booting or off.
5. Instrument dimming (0x457) still follows the obstructed G397. Out of scope for now.
6. A fogged or iced windshield blinds both the camera and G397 → lightd invalid → fallback (same failure as stock).
7. Camerad's AE histogram indexes by width instead of stride (`camera_common.cc:84`; mici 1344 vs 1408). That slightly skews
   `measuredGreyFraction`. Harmless, since `K_cam` absorbs it.
8. **Mirror fields in 0x65A** (`fla.ISP_DEFAULTS`). Nothing on the TT sends 0x65A today; the car has a standalone compass mirror.
   We send "no ISP mirror". Nobody has checked whether the cluster, the RNS-E or the gateway shows or logs anything for it.
   If something complains, the fallback is the K-matrix init values (EC/compass active = 1) or real data
   (GPS bearing → `FL1_ISP_Himmelsrichtung`, 12° per count).
9. **Upstream slot collision.** `StarPilotLightState` uses the reserved `CustomReserved12 @138`. If StarPilot upstream claims
   that slot, the merge conflicts and old logs decode with the wrong struct. Move to another reserved slot rather than
   reuse an id.
10. **Panda param bit 9** (`PQ_LIGHT_TX`) is ours. An upstream VW safety flag on 512 would collide; keep the C and Python
    constants in sync.
11. **J519 image vs sender layout.** The sender now writes byte 0 (final). A byte-4 J519 image (reviewed SGO `ACCC6C5C…21B1A`,
    old `comma_light_patch`) would see 0x3E (OEM init 62) in byte 4. Its valid bit is clear, so the J519 would quietly stay on
    G397: not dangerous, just non-functional. The handoff still forbids pairing them. Any future move must change
    `fla.apply_light_carrier()`, its test and the J519 image together.
12. **Not run on the device yet.** `lightd.py`, the card plumbing and the carcontroller send are unit- or compile-tested
    only. The first on-car step is mode 1 (shadow): check `starpilotLightState` in the logs, plus the CPU cost of the HBA
    (`hbaProcessingMs`).
13. **Mode 2/3 needs a restart to enable TX** (the safety bit is latched). Mode 2 with a stock J519 is harmless (nothing reads
    byte 0), but still check the cluster for FLA messages (item 8).
14. **Startup flicker.** Measured over 107 drive starts with the comma already awake: cameras at ~1 s, lightd valid at ~7 s, and the
    first CAN frame at 6.6–12.4 s (typically 9–10 s), because nothing can be sent before openpilot sets `ControlsReady`. A cold
    comma boot adds tens of seconds. Until then the J519 uses G397, which under the cargo box says dark, so the lights come on and
    then drop out after the comma's first OFF plus the J519 hold. The comma can't win that race. The fix belongs in the J519
    patch: a startup grace window (ignore G397 ON until the first valid comma frame or ~20 s, optionally still accepting G397
    when its code is 0).
15. **Native changes need a CI rebuild.** The comma runs the committed binaries and loggerd only logs services compiled
    into it: the upstream-built loggerd silently dropped `starpilotLightState` until the 2026-10-18 rebuild. Changes to
    `cereal/services.py`, the capnp schema or other native code ship only via `.github/workflows/build_pingu.yaml`; read
    its header comment, especially after an upstream merge.

## 10. Order of work

| step | deliverable | car changes | status |
|---|---|---|---|
| 0 | `solar.py`, `ambient.py`, `replay_lightd.py` run over existing segments; plots of B_wide/B_narrow vs GK2/GK1/sun | none | done (offline) |
| 1 | cereal struct, lightd process, TT car-state light fields, shadow mode on device | none (log only) | code done, not run on device |
| 1b | box-off calibration drive, then fit constants | none | done with the summer logs (low G397 cal) |
| 2 | bench: does 0x65A reach the J519, and is byte 0 intact? | bench | open |
| 3 | DBC + `create_fla_1` + panda TX + mode 2, after a verified byte-0 J519 image is flashed | J519 flash | code done, byte 0 final |
| 4 | V2 pixel ROIs + tunnel | — | open |
| 5 | V3 HBA in shadow mode (compare against manual high-beam use), then mode 3 + FLA coding | J519 coding | prototype offline; needs highway night data |

## 11. V4: an HBA that can be measured, then learned (proposed 2026-10-18)

Written after drive 2 (route `0000005a`, HBA_HANDOFF.md §5). Two drives of the rule stack (§5 above) ended the same way:
the replay numbers improved, the car got worse. Before changing another rule, this section says why, what a production
HBA actually does, and what to build instead.

### Why the rule stack cannot converge

- **Most tracks are never classified.** On route 5a the replay produced 58k track-frames. 23.7k are `candidate/colour`
  and 16.7k `preceding/colour`: the fallback when no rule fired, which is "unknown, so dip". Only 168 track-frames were
  ever `oncoming/motion`, the one rule with real vehicle evidence. The HBA is driven by what it cannot classify.
- **The decision has no notion of its own effect.** Our high beam lights a retroreflective sign 5-10x brighter; it
  saturates and blooms, passes every lamp test, is a "strong" track, dips on the first frame, skips the backoff, and the
  cycle repeats every 2.6 s (§5 of the handoff, seg 24). The glare override then flips tracks already marked reflector.
  These are not tuning errors: the architecture has no state for "this light exists because I switched on".
- **There is no metric a rule change can be judged by.** Replay is open loop (the recorded scene was lit by the old
  decisions), hand labels cover 150 s of dusk, and the "obvious lamp" proxy only catches near blooming lamps. So each
  pass optimised what it could see, and the car measured something else: toggles per minute 2.24 -> 2.94, median
  on-time 3.7 s -> 1.9 s, with no oncoming car missed on either drive.
- **~90 interacting parameters, each introduced for one drive.** That is the signature of fitting, not modelling.

### What the 2010 systems did (Mobileye AHC / Hella / Gentex, as used in VW FLA)

They are not magic, but they had three things this design lacks, and we should copy the two we can:

1. **A camera they controlled.** Alternating or very short exposures so headlights stay unsaturated point sources while
   reflectors (lit by the car's own beams, falling off ~d^-4) all but vanish; some used a red/clear filter for taillights.
   We cannot: camerad's exposure serves the driving model. Appearance is therefore our weakest cue, and gates on
   brightness and bloom ("lamps bloom, reflectors don't") are the ones that fail first under our own high beam.
2. **A trained spot classifier over tracked features**, not thresholds: colour, size, growth, pair geometry, motion
   against the ego flow, position relative to the horizon, and a time history. Trained on fleet data with labels.
3. **A deliberately slow, asymmetric actuator policy.** Dip fast (0.2-0.5 s), restore slowly (seconds), never flash:
   lights that appear with the restore are suspect by construction, and anything short of vehicle evidence does not dip
   again at once. Staying dipped at a sign for 5 s is a non-event for the driver; flashing is a defect.

Point 2 and 3 are the design. Point 1 we replace with what they did not have: our own beam state on CAN at 10 ms, the
driving model's road geometry and leads, a gyro, mapd, and roughly a thousand times their compute.

### Architecture

Three layers with one direction of data flow, and a metric that does not depend on the rules under test.

**A. Candidates and tracks (keep, simplify).** Blob extraction in the road corridor, nearest-neighbour tracking with gyro
prediction, and track stitching across short gaps (a far lamp flickers; today each gap makes a new `tid` and loses the
history). Every gate that currently *rejects* a blob (crisp ring, sign colour, post shape, pairing, height) becomes a
*feature* on the track instead. Rejection happens once, in C, not nine times in sequence.

**B. A learned per-track classifier.** Inputs are per-track temporal features over the last 1-2 s: area, peak, flux
trend against ego travel, saturation and hue, ring and bloom profile, aspect, pair persistence, image velocity minus the
predicted static flow (gyro + ego speed + road geometry), angle to the path and its trend, height class from the
ray, position relative to the model's lanes and leads, and **our beam state: time since the last switch and the flux
ratio across it**. Output p(vehicle). Model: gradient-boosted trees or a tiny MLP, well under 1 ms for all tracks on
the A55; a 32x32 patch CNN for appearance is a later option if the features plateau. Training labels come free from
**hindsight**: a track that resolves at short range (oncoming motion, model lead, passes us as a static roadside light,
follows our beam switch) labels its whole far history, and stitched tracks carry that label back to 300 m.
`replay_hba.py` already does a first version of this per `tid`; the labels on 5a are 20k vehicle / 4.7k static /
33k unknown track-frames, and stitching plus the beam cue should resolve much of the unknown set.

**C. A decision policy that knows what it did.** Dip when the max p(vehicle) over live tracks exceeds a low threshold,
or a model lead or wide-guard hit; fast. Restore after the corridor has been clear for `clear_s`. Then the two rules
that stop flashing: (1) tracks born within ~1 s of our own switch-on start with a reflector prior and need vehicle
evidence (pairing, motion, lead, p above a *high* threshold) to dip again; (2) every re-dip inside `short_permit_s`
lengthens the next clear time, whatever the confidence, unless the evidence was a lead or a wide hit. Town and speed
gates stay as they are, but the streetlight town gate only applies below ~70 km/h (nobody drives through a town at 110).

### The metric (build this first, it is needed under any design)

Per drive, three numbers, each computed twice: for the car's logged beam (`GK1_Fernlicht`) and for the replay's permit,
and the replay numbers conditioned on frames where the recorded beam state equals the replay's, so the open-loop lie is
at least visible:

1. **glare seconds**: high beam on while a hindsight-vehicle track is live in the corridor (the safety number);
2. **nuisance seconds**: dipped at speed in the dark with no hindsight vehicle, no lead, no town (the benefit lost);
3. **toggles per minute and the on-period histogram** (< 1 s, < 2 s) (the defect the driver notices).

Plus the label audit: how many track-frames are still `unknown`, and a sample of chains checked against the JPEGs.
Hand labels (c9, and whatever the user reports from drives) validate the hindsight labels, not the classifier directly.

### Data: there is very little video with the high beam on

Route 5a has ~5 min of high beam at speed, route 46 ~3.7 min, c9 none. Everything else is low beam. That matters in
three ways, and the design has to live with it rather than wish it away.

1. **Labels do not need the high beam.** Hindsight resolution (motion at short range, model lead, passing a roadside
   light, pairing as a car approaches) works under either beam, so the vehicle/static training set comes from all night
   video. Only the *reflector response to our switching* needs real switches, and every switch we have (about 100 across
   both drives, half of them the flashes themselves) is a sign lit by the high beam: a small but clean set.
2. **The classifier must not learn "the scene under low beam".** Appearance features (peak, bloom, ring, saturation)
   shift when our beam comes on; motion, geometry, colour hue, pairing and time do not. So: beam state and time since the
   last switch are inputs, appearance features are used with that conditioning, and the leave-one-drive-out test must
   report the high-beam-on frames separately. If the high-beam-on score is poor, the policy leans on the
   beam-invariant features there (dip on the unknown, as today) until there is data.
3. **Get the data cheaply: drive with the stalk, not the HBA.** In shadow mode (`VolkswagenLightdMode` 1) the HBA logs
   but the J519 ignores it, and the driver works the high beam manually and generously on empty roads. Each manual dip
   is a driver label "vehicle appeared" (stalk and `GK1_Fernlicht` are on CAN), each long manual on-period is minutes of
   signs, posts and delineators under the high beam with no vehicle, and the camera sees the closed loop the driver
   closes. One or two such drives on the usual mountain roads give more high-beam-on video than everything so far, with
   no risk to anyone's eyes. This needs `starpilotLightState` in the rlogs (the loggerd rebuild) so the shadow decisions
   can be scored against the driver's.
4. **Later: mode 3 drives are labels too**, once flashing is gone, because the user's notes ("flashed a sign at X") and
   every switch become training data. Until then, do not train on mode 3 drives' switches as if they were truth: half of
   them are the bug.

### Step 1 findings (2026-10-18, `hba_tools/score.py` on routes 5a and 46)

- **The car's nuisance time is 30 % of the dark time at speed on 5a (356 s of 19.7 min) and 24 % on 46**: dipped with
  no vehicle chain, no lead, no town. The replay understates it by 20-25 % (287 s and 176 s), in the direction the
  open-loop argument predicts. Conditioning on frames where the recorded beam equals the replay's permit keeps 83-87 %
  of frames and removes most of the short on-periods from the replay's count: the flashes are mostly closed-loop.
- **Hindsight "vehicle" labels were mostly receding tracks.** Of the motion-resolved vehicle chains, 5a had 11
  approaching vs 244 receding, 46 had 22 vs 466; most receding ones are off the lane: delineators lit by the high beam,
  exactly what `hba.py` itself calls "white moving away = low reflector". The three longest "glare" episodes on 5a were
  2-15 px dots at 30-70 m on the shoulder. Oncoming cars almost never resolve by motion: they are inside 70 m for under
  a second and bloom to NaN range before that. So the first glare numbers (8.5 s / 8.7 s) are label noise; without the
  receding chains they are 2.7 s and 0.9 s, and even those are unaudited. Labels now use the sign and the side, and an
  oncoming car is resolved by its **pass event** (area explodes, moves to the oncoming edge, wide guard sees it).
- **Stitching helps less than hoped**: tids to chains 4711 -> 3013 (5a), unknown track-frames down 11 % on 5a and 25 %
  on 46, median track life 0.2 s -> 0.6 s. Far lights really do flicker in and out at this exposure.
- **The beam cue reaches few tracks**: only 44 of 130 switches on 5a have any chain with flux on both sides, because the
  lit sign is born at the switch (the same reason `_beam_check` never fires on the car). The switch-off side is biased
  (the dip was caused by the track, and every strong blob shrinks when our beam goes off) and is dropped. "Born within
  a second of our switch-on" is the usable form of the cue, as a feature and as a policy prior.
- **Oncoming inventory (labels v2, pass events + sign/side motion, audited 14/15 right on each drive):**

  | | 5a | 46 |
  |---|---|---|
  | oncoming cars (chains merged within 2 s) | 50, 39 at >= 60 km/h | 85, 53 at >= 60 km/h |
  | first track before the pass, median (p25-p75) | 8.2 s (3.9-13.0) | 4.3 s (2.0-6.7) |
  | wide guard saw the lamp within 1 s of the pass | 78 of 94 pass chains | 130 of 157 |
  | `hba.py` threat on the car's frames, >200 / 120-200 / 70-120 / <70 m | 76 / 74 / 80 / 74 % | 49 / 77 / 87 / 74 % |
  | logged beam on with a car in view | 1.7 s over 4 cars | 6.0 s over 6 cars |
  | nuisance (dipped, nothing in view, no town) | 366 s | 265 s |

  The misses are 0.5-0.8 s reaction time at the pass, plus one 2.1 s case on 46 (seg 9 at 50 s) where the replay dipped and
  the car did not. So the safety side of the current HBA is near its floor; everything to gain is nuisance and flashing.
  Rural, fast, clean oncoming cars: about 25 on 5a and 30 on 46, about 7k oncoming track-frames per drive. Enough for a
  tree model with leave-one-drive-out, not for anything that learns appearance from pixels. Remaining label caveats: the
  inverse-depth speed fit is garbage in dense scenes (|v| > 40 m/s on 1k near frames on 46; those are ignored), receding
  chains with strong oncoming-kind frames (35 / 71) are kept out of training as `ambiguous`.
- Audited frames: signs under our beam labelled reflector were right; wrong vehicle labels were tiny shoulder dots
  resolved as "moving away"; one flux-ratio reflector was probably a headlight. Label quality, not model choice, is the
  risk in step 2.

### Step 2 and 3 results (2026-10-18, evening)

**Step 2, the per-track classifier (`hba_tools/train/`):** 61 causal features per (chain, frame), gradient-boosted trees,
trained on one drive and tested on the other. Verdict: it matches the rules, it does not beat them yet.

| held-out drive | AUC oncoming vs static | oncoming recall at 5 % FPR | at the rules' own FPR (38-44 %) | rules |
|---|---|---|---|---|
| 5a (trained on 46) | 0.73 | 8 % | 75 %, lead 6.6 s | 74 %, lead 6.9 s |
| 46 (trained on 5a) | 0.66 | 9 % | 67 %, lead 4.0 s | 73 %, lead 3.8 s |

Why, in order: (1) three quarters of oncoming frames are the far history where a car is one white dot, and the label is
the chain's hindsight label, so per-frame the problem is unlabelable there; (2) two drives: the top features were drive
fingerprints (ego speed, time since the last switch, chain age), and dropping them raised oncoming AUC to 0.80 / 0.74;
(3) the negatives were 7-11k near-range delineator frames, while the 29-33k `unknown` far dots the HBA actually dips
for were never in training. The one clean result: the signs born at our own switch-on sit at the bottom of the score at
any threshold (median p 0.005 / 0.023), from one feature, time since our beam came on. So that mechanism went into the
policy (step 3) and the classifier waits for more drives; the pipeline (labels, features, fit, leave-one-drive-out) is
in place, so every new night drive is a training set.

A third pass (fingerprint features out, four physics features in: predicted static angular rate from v, dist and
lateral, the observed rate, their ratio, and flux normalised to the chain's low-beam level; evaluation per chain as the
max p over the last second; dc/dd/de added for negatives) moved it to "a cleaner version of the rules": at the
false-positive rate `hba.py` actually runs at (its threat flag fires on 57-58 % of the labelled static/reflector
chains) the model sees 98 % / 90 % of oncoming frames vs the rules' 85 % / 87 %, misses no car, same lead time, and
keeps 100 % / 80 % of the born-at-switch signs down vs 65 % / 52 %. At a usable 2-5 % chain FPR it still misses 30-48
of 47-70 cars before the pass; the 46 oncoming AUC of 0.72 is a transfer ceiling (town, four-lane, cars at 10-14 m
lateral that rural 5a never showed). The far dots stay unlabelled: a "static exit" rule, the complement of the pass
event, converts only 1.4-1.8 % of them, because far dots die at the horizon rather than exiting. The angular-rate
consistency of a static point is not separable on its own (AUC 0.55-0.57; the lamp-height ray overestimates the range of
low roadside reflectors by ~1.5x), so it is a veto, not a feature that carries the class. Every growth of the training
set moved the held-out numbers; the label and feature pipeline reruns unattended in ~30 s per drive.

**Step 3, the policy layer (`hba.py`, 2026-10-18):** three changes, all "the system knows what it just did":
1. a light without a partner blob born within `beam_born_s` = 1.2 s of our high beam coming on is a reflector whatever
   its size (the rule used to cover only blobs under `strong_area`, and the flashing signs were big); it stays one until
   it shows a real lamp pair on 3 frames or measured oncoming motion (`Blob.partner`, new, is a second blob on the row;
   `paired` also includes wide blobs, and a lit sign is wide);
2. the glare override no longer flips a track marked reflector back to oncoming;
3. a re-dip within `short_permit_s` of a restore doubles the next clear time whatever the confidence, unless a model
   lead, a wide-camera lamp or measured oncoming motion caused it (a strong lone lamp right after a restore is most
   often the sign we just lit). `_motion_fit` is factored out so reflector tracks can be checked for motion.

Replay (open loop, so the born-at-switch rule only acts where the recorded beam came on when the replay restored):

| replay permit, dark >= 60 km/h | 5a before | 5a after | 46 before | 46 after |
|---|---|---|---|---|
| on-periods | 52 | 43 | 31 | 30 |
| of which < 1 s / < 2 s | 13 / 21 | 4 / 11 | 2 / 10 | 3 / 10 |
| toggles per minute | 2.69 | 2.24 | 1.95 | 1.90 |
| glare s (oncoming or preceding chain live) | 3.0 | 2.4 | 2.4 | 2.0 |
| nuisance s | 293 | 302 | 172 | 199 |
| high beam on | 31.9 % | 31.1 % | 27.2 % | 24.5 % |

Of the 11 short on-periods left on 5a, 6 are a brand-new strong blob at the far end of the corridor while the recorded
beam was already on (the sign was lit by the car's beam, not by a switch the replay can see), 3 are far red dots, 1 is
a real oncoming car. On the car the born-at-switch rule covers the first group, so the real gain should be larger than
the replay shows; the nuisance cost of the backoff is the 9-27 s.

What this does not touch: the ~300 s of nuisance per drive. Half of it is the clear and backoff delay after dips for
far unknown dots (238 s of 5a's dipped time is reason "delay" alone), the rest the dips themselves. That is the
classifier's job once there is data, and the town gate's (51 % of dipped time on the car involved the town flag; the
streetlight gate at 75-110 km/h on 5a segs 30-34 is a yard-light cluster, not a town, but c9's town was driven at
70-88 km/h, so a speed condition needs the density distributions of real towns vs rural clusters first).

Not verified: CPU on the device (`_motion_fit` now also runs for reflector tracks, a handful per frame), and nothing has
been driven. Shadow mode first.

### Order of work

| step | deliverable | status |
|---|---|---|
| 1 | `hba_tools/score.py`: stitching, extended hindsight labels, the three numbers for 5a and 46, label audit | done 2026-10-18; labels being fixed (sign/side, pass events) |
| 2 | feature extraction per track into a training table; GBDT trained on 5a + 46 (+ dc/dd/de for static), tested leave-one-drive-out | done 2026-10-18: matches the rules, waits for more drives |
| 3 | layer C policy in `hba.py` behind the classifier, replay on all night drives, CPU check on the device | policy done 2026-10-18 (born-at-switch, backoff); device CPU open |
| 4 | shadow drive (mode 1), then mode 3; the user's drive notes become hand labels | open |

What this does not fix: a far oncoming car at 300 m is a single white dot and so is a delineator. No feature separates
them on one frame; time, pairing as it approaches, and our beam response do. The policy therefore errs to dipping on the
unknown far dot, as today, but it stops re-dipping on the dot it created itself.

## 12. Evidence policy after Gentex SmartBeam (2026-10-19)

Source: static RE of the Audi C6 SmartBeam mirror firmware (`carstuff\HBACam\SMARTBEAM_NOTES.md`). What it does that
`hba.py` did not, mapped to our failures:

| SmartBeam | what we had | our failure it addresses |
|---|---|---|
| each tracked light carries its own 0-31 index; every 200 ms frame its type steps it down (far taillight 1, bright close headlight 4-8, saturated: to 0); the global index is the minimum; bi-xenon closes below 25, i.e. after **7 points of evidence** (far light under 64 km/h ~1.4 s, close bright 0.4 s, saturated at once); steps double/triple above 64/137 km/h | any threat on two frames dips; a "strong" track dips on its first frame, skipping `confirm_s` | first-frame dips on lit signs; far dots that live 0.6 s each costing a dip and a clear delay |
| classification thresholds are 24 points **lower while dipped** and full while the beam is on, and a light must be older than one frame to count while on | brightness and bloom count *more* under our high beam (a lit sign becomes "strong") | the 2.6 s sign flashing on route 5a |
| restore by cause: fast return 0.4-1.6 s after an oncoming car passes at the near edge; 3 s hold for headlights/taillights that vanish (6 s taillights > 130 km/h); holds arm only after 2 frames; leaky seen-counters (+2/-1, cap 5); 2-5 s minimum low-beam time after every dip | one `clear_s` for everything, doubling backoff | restores between two cars (done, below) |
| village: "prevent switch-on" variant; thresholds x2.3 count / x1.5 brightness above 90 km/h | streetlight density, same at any speed | not done yet (idea 4) |

### What is implemented (`HbaPolicy` in `hba.py`, `evidence=True`)

The decision layer is now its own class fed with `TrackView`s (kind, why, strong, near, pairing, reflector), so it can
be rerun on replay csvs without video (`hba_tools/policy_sim.py`; `evidence=False` is the old decision and reproduces
the replay's own permit on 99.9 % / 99.7 % of 5a / 46 frames). Each track's evidence grows at a rate per second
(1 = dip; > 10/s is one 10 Hz frame):

| what the track shows | rate | time to dip |
|---|---|---|
| model lead, wide-camera lamp | - | at once (not per track) |
| measured oncoming motion | 12 | one frame |
| bright (area >= 25 or saturated) within 70 m, either beam | 12 | one frame |
| bright, farther or no range, our high beam **off** | 6 | two frames |
| bright, farther or no range, our high beam **on** | 2.5 | 0.4 s |
| weak within 70 m | 6 | two frames |
| weak, far, lamp pair over 0.5 s | 4 | 0.25 s |
| weak, far, lone dot | 0.5 | 2 s (the rules never dipped for these) |
| classified static / reflector | -2 | drains |
| lost (a re-detection within `gate_px` in 1 s inherits it) | -3 | drains |

A track at >= 0.5 holds the clear timer. Restore and backoff are unchanged.

Why the high-beam discount is **far only**: on 5a / 46, the strong non-static lights the replay saw under our high
beam that hindsight calls reflectors are far and big (area median 164 / 184 px); the oncoming cars among them are far
and small (24 / 30 px) or within 70 m (34-160 px). A first version discounted every bright light under our beam and
put real cars at 14-45 m 0.4-0.9 s late on 46 (`policy_sim.py --diff`); splitting by range removed all of them.
(`sep_stats.py` prints the table.)

### Results (open loop; policy_sim on the step-3 replays `runs/new5a`, `runs/new46`, dark >= 60 km/h)

`glare<` counts only vehicle chains within 200 m (distance-weighted glare is the number that matters to the other
driver; the plain glare column counts a car at 400 m like one at 80 m).

| | 5a legacy | 5a evidence | 46 legacy | 46 evidence |
|---|---|---|---|---|
| high beam on | 31.2 % | 35.7 % | 24.3 % | 26.5 % |
| glare s | 2.4 | 1.5 | 2.0 | 2.0 |
| glare< 200 m s | 1.5 | 0.9 | 1.7 | 1.7 |
| nuisance s | 304 | 250 (-18 %) | 203 | 181 (-11 %) |
| toggles / min | 2.24 | 2.19 | 1.90 | 1.78 |
| on-periods < 2 s | 11 | 6 | 11 | 9 |

Sweep (one parameter at a time from the first defaults, then combinations; full output in the session, rerun with
`policy_sim.py --var ...`): slowing *near* or *pair* evidence or lowering `clear_s` buys nuisance with glare on 46;
dropping the backoff (`clear_max_s=2`) buys 40-60 s of nuisance but adds toggles, and the backoff is the closed-loop
anti-flash, so it stays; `ev_hold` and the speed gain made no difference at these rates; `ev_lone=0.5` lowered glare
on both drives (far lone dots that persist are often cars).

Checked on the real pipeline: a full video replay of the new `hba.py` (`runs/ev5a`, `runs/ev46`) agrees with the
simulated policy on 99.9 % of frames on both drives, and its own permit gives the numbers above (5a: glare 1.5 s,
nuisance 247 s, 6 on-periods < 2 s; 46: glare 2.0 s, nuisance 180 s). Replay time per frame on the desktop is within
noise of before (the policy is a dict update per track); device CPU not re-measured. 69 lightd tests pass (four new:
far bright light under our beam is slow, near bright light is instant either way, a lone far dot must persist and keeps
its evidence through a flicker, the legacy policy is still selectable).

What open loop cannot show: the high-beam rate only acts on frames where the *recorded* beam was on (~5 min per drive),
and the flashing it targets is closed loop (our beam lights the sign). Its value was chosen from SmartBeam's ratio
(far headlight vs bright close one, 1 : 4-8) and the switch-to-sign latency on 5a (0.3-1.0 s), not fitted. It needs
the shadow drive (section 11, step 4) and then a mode 3 drive; the first number to check there is on-periods < 2 s.

### Restore by cause (`restore_by_cause=True`, 2026-10-19)

Every track that held the corridor is recorded when it stops: **pass** (lost on the oncoming half of the image, not red,
within 70 m or >= `pass_area` px), **far** (lost while far: a bend or a crest), **other** (still there but static now or
drained: a sign); a model lead or a speed/town/dark gate counts as other, a wide-guard hit as wide. Only what ended in
the `cause_window_s` = 1 s before the corridor went clear says what cleared it (the car's other lamp or a far dot that
died earlier doesn't matter, SmartBeam only asks that nothing is there now). Then:

- cleared by a pass (plus the wide guard seeing the same car): restore after `clear_pass_s` = 0.5 s;
- cleared by far losses only: `clear_far_s` = 2 s (3 s, SmartBeam's hold, cost 7-12 s of nuisance for no glare);
- anything else: `clear_s` = 2 s; the doubling backoff still wins whenever it is above that;
- and every dip stays low at least `min_low_s` = 3 s at >= 70 km/h, rising to 5 s at 28 km/h.

| policy_sim, `runs/ev5a` / `runs/ev46` | 5a evidence | 5a + cause | 46 evidence | 46 + cause |
|---|---|---|---|---|
| high beam on | 35.7 % | 35.4 % | 26.5 % | 26.6 % |
| glare s | 1.5 | 1.0 | 2.0 | 1.5 |
| glare< 200 m s | 0.9 | 0.8 | 1.7 | 1.2 |
| nuisance s | 250 | 253 | 181 | 179 |
| toggles / min | 2.19 | 2.13 | 1.78 | 1.60 |
| on-periods < 1 s / < 2 s | 3 / 6 | 4 / 5 | 3 / 9 | 0 / 5 |

Against legacy (the rules before section 12) that is, on 5a: glare 2.4 -> 1.0 s, nuisance 304 -> 253 s, on-periods < 2 s
11 -> 5; on 46: glare 2.0 -> 1.5 s, nuisance 203 -> 179 s, toggles 1.90 -> 1.60 / min, on-periods < 2 s 11 -> 5.

What did the work: the minimum low time (3 s), not the fast return. Only 4-6 restores per drive are cleared by a pass
alone (most cars pass while something else still holds the corridor), so the 0.5 s return saves 3-5 s of nuisance per
drive. The minimum time removes restores between two cars of a group and the shortest on-periods: 2.5 s did half of it,
4 s cost another 10-20 s of nuisance for nothing. The policy-only change was not re-run through a full video replay (the
replay matched policy_sim on 99.9 % of frames before it); 71 lightd tests pass.

### Next, from the same source

4. **Town gate**: the "prevent switch-on only" variant and SmartBeam's speed split of the thresholds (90 km/h).
5. **Exposure**: `hba.py` thresholds are raw pixel values and never see `exposureValPercent` or gain. If auto exposure
   reacts to our own high beam, every brightness test moves with our switching. Log both into the track csv and check.
6. Speed band: SmartBeam stays on down to 28 km/h once on (switch-on 58); ours drops at 50.
