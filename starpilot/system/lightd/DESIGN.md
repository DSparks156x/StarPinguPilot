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
