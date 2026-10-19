# HBA handoff (2026-10-18)

A brief for an agent picking up the camera high beam assist (HBA) in StarPinguPilot. Read this first, then
`DESIGN.md` §5 (the long history) and `hba.py`.

**Status in one line:** three offline passes made the replay numbers look good. The second on-car test (2026-10-18,
route `0000005a`) was bad: it stayed dipped for reflective signs or flashed the high beam at them, it toggled the high
beam far more than before, and it was no better at getting the high beam up. The analysis (§5) pins the flashing on
our own high beam lighting distant signs, which then trigger first-frame dips with no backoff. It also found too much
dipped time spent in "delay" and in a town flag that fires on rural roads with houses. No oncoming car was confirmed
flashed on that drive.
**Since then (same day, §9):** a hindsight scoreboard exists, a first learned classifier was tried (matches the rules,
needs more drives), and `hba.py` got a policy layer that treats lights born at our own switch-on as reflectors and
backs off after every early re-dip. Not driven yet.

## 1. The car and the goal

- Audi TT Mk2 (VW PQ35), comma four ("mici", OS04C10 cameras), openpilot fork StarPilot, branch `pingu` in
  `C:\Users\raccoon\Documents\carstuff\StarPinguPilot`. The user does the body-computer (J519) reverse engineering in
  `carstuff/CentElecRE`.
- The TT has **bi-xenon** headlights. The high beam is a shutter: on or off. `FL1_Fernlicht_PWM` does nothing, so
  there is no dimming or fade.
- The factory high beam assist (FLA) path in a patched J519 is driven over CAN: lightd → `starpilotLightState` → card
  → `mFLA_1` 0x65A (`FL1_FLA_Aufblenden`, `FL1_Abblendursache` reason bits) → panda → J519. The J519 only uses it
  with the stalk in AUTO, the low beam on and the camera fresh. Command to beam takes 0.1 s median, 0.3 s max
  (measured: Aufblenden change → `GK1_Fernlicht` on CAN 0x390 byte 6 bit 1).
- `VolkswagenLightdMode`: 0 off, 1 shadow (log only), 2 low beam, 3 low + high beam. The test drives were mode 3.
- **Safety rule:** anything uncertain means no high beam. Never set `FL1_ISP_defekt` (it latches a fault in the J519).
- Right-hand traffic, Colorado: mountain two-lane roads, lots of retroreflective signs, delineator posts, chevrons,
  small towns.

## 2. How the code works now (`starpilot/system/lightd/hba.py`, ~1000 lines)

`HbaClassifier.update()` runs on its own thread (`lightd.py` `HbaWorker`, pinned to cores 0–3, nice 10, ≤10 Hz) on
the newest narrow-camera NV12 frame:

1. **Corridor.** The model's lane lines (road level; `position` is the device path and needs `+height`) are projected
   15–600 m out, 0.2–1.6 m above the road, our lane + 4.5 m on the oncoming side + 1.5 m on ours, widening past the
   model horizon (~192 m), with a 10 px margin. A red-only band adds an adjacent lane if the model sees one.
2. **Blobs** inside it: white (BGR-gray ≥ 225) and red (HSV hue ≤ 8 / ≥ 172, S ≥ 110, bright core). They're rejected
   if they're lit yellow/green signs (saturation, yellow surround), or white ≥ 16 px with a dark ring around them
   (`crisp_ring_v`: "lamps bloom, retroreflectors don't"). Then pairing and the post/delineator shape test. Only
   candidate regions found from the NV12 planes are converted (exact, see the CPU notes).
3. **Tracks:** nearest neighbour, predicted through the gyro's frame-to-frame rotation. Range is where the ray
   crosses lamp height (0.7 m) over the model's road profile. Object speed comes from the inverse-depth slope minus
   vEgo, under 70 m only.
4. **Per-track rules** set kind = static / oncoming / preceding / candidate. Each rule records `why` = post, high,
   roadside, motion, colour, vote, glare or beam:
   - static voting over 1 s;
   - a "glare" override: a strong white lamp on the oncoming side can't be static;
   - high-beam feedback: tracks that brighten ≥ 1.8× when our beam comes on are reflectors;
   - the roadside angle rule (> 3.3° to our side);
   - "white moving away = low reflector";
   - the "high" rule: the ray is never at lamp height.
5. **Wide-camera guard** (115° ecam, every other px, 5 Hz): big neutral blooming lamps on the oncoming side outside
   the narrow view dip at once.
6. **Town gate:** streetlight density above the corridor, plus mapd limit ≤ 50.
7. **Decision:**
   - a threat dips after 2 frames, or on the first frame if it's "confident" (model lead, strong lamp, oncoming
     motion, wide hit);
   - restore needs `clear_s` = 2 s clear;
   - backoff doubles the clear time (up to 8 s) when a restore is cut short within 3 s by weak evidence;
   - speed gate 60 on / 50 off km/h.

About 60 tunable params sit in `HbaParams`. The design is a stack of hand rules that interact, and every fix so far
has been another rule. That's probably part of the problem.

## 3. What has been tried, and what each pass found (details in DESIGN.md §5)

| pass | data | main changes | offline result |
|---|---|---|---|
| prototype | dc/dd/de (slow residential) | corridor, blobs, NN tracks, angle rule | detection only |
| 2nd (10-17) | c9 hand labels (150 s) | corridor 1.2 m too low (lane z), timing in s, post/pair, sign colour, gyro, voting, backoff, beam cue, town gate | c9: vehicle threat 91→96 %, glare 2.2→0 %, nuisance 38→20 % |
| drive 1 (route 46) | 34 min night, mode 3 | **regression found:** gyro-rotated projection swung the corridor ~13° in turns (removed); halogen halos read as yellow signs; crisp-ring sign filter; glare override; first-frame dips; wide guard; services.h fix | route 46 replay: "obvious lamp" glare 7.0→0.3 s; c9 nuisance →14 % |
| CPU | device profile | candidate regions, vectorized corridor, etc. | ~2x faster on A55, bit-identical decisions |
| **drive 2** | **see §5** | — | **bad on the car** |

Tried and dropped:
- Brightness-vs-distance exponent (d⁻² lamps vs d⁻⁴ reflectors): range within one track swings 50–400 m near the
  horizon, so the fitted exponent is noise.
- Gyro correction of the projection: wrong, because the model's road is already in the current frame.
- An analytic chroma bound for the red candidate test: too loose.

## 4. Why the offline numbers didn't predict the car (start here)

- **Replay is open loop.** `replay_hba.py` feeds recorded video, and in that video the high beam was on or off
  according to *the old code's* decisions. On the car, our own high beam lights the scene. A retroreflective sign
  at 50–200 m is maybe 5–10x brighter under the high beam than under the low beam, so it can saturate and bloom.
  Then it passes the "lamps bloom" test and becomes a strong, confident white lamp. That gives a first-frame dip,
  the sign goes dim and crisp again, the corridor is clear for 2 s, the beam comes up again, and so on. That would
  produce exactly "flashing at signs" plus "lots more flashing". Changes that made dips faster (first-frame confident
  dips) and the "glare can't be static" override make this loop tighter. **Verify on drive 2 by looking at sign
  blobs when the car's high beam was on vs off** (CAN `GK1_Fernlicht` is in every rlog).
- The beam-feedback cue (reflectors brighten when our beam switches) was never tested on real switches until drive
  2. Drive 1 had real switches too (route 46: 48 high-beam-on runs).
- Hand labels exist only for c9 (80 s with a vehicle, 70 s without, dusk at sun −5°, high beam never on). The
  "obvious lamp" proxy in `replay_hba.py` only catches big near-white blooming blobs; it says nothing about nuisance
  or flashing.
- The device lags the replay: route 46 shows the real beam-off 0.4 s after the replay's dip (median), with a tail of
  1–3 s (route 5a's replay instead runs ~0.2 s ahead of the device, 88.7 % agreement). lightd's own decisions and
  `hbaProcessingMs` aren't logged on either drive: `cereal/services.h` was stale (fixed in 718bd501), and the device's
  prebuilt loggerd binary still hasn't been rebuilt against it.

Suggested directions: superseded by `DESIGN.md` §11 (V4: measure first, then a learned track classifier, 2026-10-18).
The bullets below are kept as the original list:
- Evaluate in closed loop, at least per sign: does the blob change with `GK1_Fernlicht`? Most of the information
  about reflectors is in our own switching.
- Minimum dwell times (stay up ≥ N s unless a confident *vehicle*, stay down ≥ M s) to stop flashing, and a
  per-location memory ("this blob appeared when we switched on").
- Fewer, better-validated rules, or a small learned classifier over the per-track CSV features. There's now enough
  data with logged beam state to label tracks semi-automatically (reflector = follows our switches).
- More hand labels on the new drive(s), including what the user saw ("flashed a sign at X").

## 5. Newest drive: route `0000005a--cedbf8229c` (2026-10-18, ~36 min night, mode 3)

**What ran:** the 718bd501 classifier, confirmed by md5 on the device (`/data/openpilot` at build `29cc7a0f`).

**Still no `starpilotLightState` in the rlogs.** The device's `services.h` lists it, but the prebuilt `loggerd` binary
wasn't rebuilt (it doesn't contain the string). The HBA's own reason, counts and `hbaProcessingMs` are therefore
unknown; the analysis used `sendcan` 0x65A, CAN 0x390 and `starpilotCarState`. Rebuilding loggerd in the prebuilt tree
is the fix. lightd CPU from procLog: 13–32 % of a core per segment, 51 % peak in town, so no starvation.

**Metrics** (dark, ≥ 60 km/h; route 46 in brackets):

| | route 5a | route 46 |
|---|---|---|
| time | 19.7 min | 17.0 min |
| high beam actually on | 26.7 % | 21.6 % |
| on→off toggles per min | 2.94 | 2.24 |
| on-periods | 58 | 38 |
| of which < 1 s | 21 | 5 |
| of which < 2 s | 30 | 9 |
| median on-time | 1.9 s | 3.7 s |
| longest on-time | ~31 s | — |

- Off-periods: median 4.3 s.
- Command → beam: 72–76 ms median, max 126 ms (one 0x390 cycle).
- Reasons while dipped:
  - delay (128): 32 %
  - town (8) involved: ~51 %
  - oncoming (1, no town): 10 %
  - preceding (2, no town): 6 %
- **No confirmed case of the high beam being on with an oncoming car in view.** Every candidate checked was a lit sign
  or reflector.

**Replay vs the car:**
- The replay of the same code agrees with the logged command 88.7 % of the time, at 0.2 s device lag.
- Replay permits 31.9 % vs the car's 26.9 %.
- The route-46 classifier (`hba_tools/hba_drive.py`) on segs 10–34 permits 28.2 % vs 34.1 % now, but with fewer
  flashes: 14 on-periods < 2 s vs 21.
- Replay is open loop, so it understates the flashing.

**Failure modes, with evidence** (frames in `carstuff\logs\hba_tools\runs\r5a\jpg`, sheets C1/C2/c24/c22 for flashes):

1. **Signs lit by our own high beam dip at once, and nothing backs off.** This is the flashing. In 26 of 29 threat
   tracks at short-on dips, a brand-new "candidate/colour" strong white blob (area ≥ 25 or peak 255) appears near the
   far end of the corridor, distance mostly NaN. Example: seg 24 at 37.8, 40.3, 42.9 and 45.5 s, four 0.2–0.3 s
   flashes every ~2.6 s, each when the high beam lit a white square sign at 155–303 m. Code paths, all from 718bd501:
   - `Track.is_threat`: a strong track is a threat immediately, skipping `confirm_s` and the far-light pair test.
   - "Confident" (any strong track) dips on the first frame.
   - The beam-born reflector rule now requires `area < strong_area`, so big signs that appear on switch-on aren't
     caught.
   - `_beam_check` needs a "before" sample, which a newly lit sign doesn't have.
   - The doubling backoff is skipped for confident dips, so clear stays 2 s. That matches the 2.6 s flash period.
2. **The glare override beats reflector detection.** `_classify` turns any strong, non-red, non-high, non-post static
   track on the oncoming side (or with NaN angle) into ONCOMING "glare", even one already marked `reflector`
   (seg 22:5.5, 23:57.2, 26:31.3). Also, `sign_frac_white = 4.0` lets yellow signs with an overexposed white middle
   through as white lamps (20:22.6, 24:45.5).
3. **Time lost dipped without a threat.**
   - Delay is 32 % of dipped time; each nuisance dip costs ≥ 2 s, up to 8 s after backoff.
   - The town flag is set in ~51 % of dipped time at speed. Replay: town for 397 s at ≥ 60 km/h (mean 2.7 lights per
     frame), only 27 s of it from a posted limit. Segments 30–34 (75–110 km/h) are rural road with scattered houses,
     not a town; segment 11 is a real town edge.

**Episode index:**

| type | segment:second (video frame / 20) |
|---|---|
| (b) long dips at speed with no vehicle | 13:48–60, 20:48.5–60, 21:35–48, 22:5.7–14, 25:47–60; town false positives at 30:3–14 and 31:32–50; a real town edge at 11:0–15 and 11:49–60 |
| (c) flashes | 24:37.8–45.5, 22:5.5, 15:29.4, 23:57.2, 20:22.6, 23:41.7–51.0, 28:20.0; 26:31.3 might be a real distant car |

Undetermined: the HBA's internal reason and timing (not logged), whether 26:31.3 and 14:34.5 were distant cars.
`hba_tools/runs/r5a/` holds both timelines (5a and 46), the full replay CSVs, the old-classifier replay and the
episode JPEGs.

The open-loop hypothesis in §4 is confirmed for signs: the high beam lights a distant retroreflective sign, the sign
becomes a strong blob, and the beam dips on the first frame with no backoff. A fix has to use our own beam state, and
evaluation has to be closed-loop (or at least condition on `GK1_Fernlicht`).

## 6. Data: where everything is

| what | where | notes |
|---|---|---|
| **drive 2, route `0000005a--cedbf8229c`** | `C:\Users\raccoon\Documents\carstuff\logs\0000005a--cedbf8229c\` | 37 segs, night, mode 3, rlog + road + wide video, 3.0 GB, flat layout; see §5 |
| short hops before drive 2 | `carstuff\logs\00000047--974c057ff9`, `00000049--62eb666993`, `0000004e--a61141122e`, `00000050--c466f78c1f`, `00000054--aabd0f5b4e`, `00000055--ea56a34fe2` | 3–5 segs each, dusk/low speed, none ≥ 60 km/h; only 49 has 2.8 s of high beam |
| drive 1, route `00000046--c6859cdf72` | `C:\Users\raccoon\Documents\carstuff\logs\00000046--c6859cdf72\` | 34 segs, night, mode 3, rlog + road + wide video, 3.1 GB; no `starpilotLightState` (use `sendcan` 0x65A + CAN 0x390) |
| c9 (hand-labelled dusk, 70–88 km/h) | NAS `\\SHITNAS\appdata\pingulink\uploads\de6bbd2ebc68925e\000000c9--4c6e2f9e25` | rlogs segs 3–6, video 0–9 (qlogs for the rest). Labels: `carstuff\logs\hba_tools\labels_c9.csv` (route,seg,t0,t1,vehicle/none/ambiguous) |
| c5 (dusk, sun +2°) and other older routes with video | same NAS folder: `000000a6`, `000000ae`, `000000be`, `000000c2`, `000000c3`, `000000c5`, `000000c9` | mostly daylight; c5/c9 dusk |
| dc, dd, de (night, slow residential, ≤ 47 km/h) | `C:\Users\raccoon\Documents\carstuff\pinguverse\pingulink-local\uploads\de6bbd2ebc68925e\000000dc--…`, `dd--…`, `de--…` | regression set for town/residential; video only on a few segs |
| summer daylight drives (108 routes, 1428 segs, rlogs only) | `carstuff\pinguverse\pingulink-local\uploads\de6bbd2ebc68925e\` and the NAS | used for the low-beam estimator, not HBA |
| 42 older segments with video | `C:\Users\raccoon\Documents\carstuff\logs\00000000--2fe68aac58--*` (segment-dir layout) | daylight, April |
| live device | `ssh -i ~/.ssh/id_ed25519_comma -o IdentitiesOnly=yes comma@192.168.10.82`, routes in `/data/media/0/realdata/` | only cores 0–3 from SSH; `/usr/local/venv/bin/python`, `PYTHONPATH=/data/openpilot`; profiling corpus in `/data/tmp/hbaprof/` |

## 7. Tools

- `starpilot/system/lightd/tools/replay_hba.py <route dir> [--segs 3-6] [--labels csv] [--jpg N] [--at seg:t0-t1] [--clf other_hba.py] --out prefix`:
  replays video through the classifier and writes `_frames.csv`, `_tracks.csv` (one row per track per frame, with
  `why`), `_wide.csv` and annotated JPEGs. It prints hand-label scores, the obvious-lamp glare proxy and town spans.
  `--clf` replays a saved copy of hba.py, e.g. `hba_tools/hba_drive.py` = the code as driven on route 46.
- `starpilot/system/lightd/tools/bench_hba.py`: synthetic per-stage timing on the device.
- `C:\Users\raccoon\Documents\carstuff\logs\hba_tools\` (README inside):
  - `timeline.py`: 0x65A + beam state per route;
  - `events.py`: road + wide frames around dips;
  - `contact.py`, `sheet.py`, `crop.py`: contact sheets for hand labelling;
  - `blobdbg.py`, `halo.py`, `lampdbg.py`: per-blob gate stats on one frame;
  - `prof_*.py`, `equiv*.py`, `dump_corpus.py`: device profiling and equivalence checks;
  - `sync.sh`, `test.sh`, `run.sh`, `run_par.sh`: the WSL helpers;
  - for drive analysis: `metrics.py`, `latency.py`, `onlist.py`, `dipwhy.py` (which tracks caused each dip),
    `episodes.py`, `compare.py` (replay vs logged command), `replay_onstats.py`, `proccpu.py`, `copy_routes.sh`
    (pull routes off the comma).
- **Tests run in WSL** (Windows Python has no capnp):
  - clone at `~/spp`, venv `~/spp-venv`;
  - copy changed files over with CR stripped (`sync.sh`);
  - `PYTHONPATH=~/spp:~/spp/opendbc_repo ~/spp-venv/bin/python -m pytest -n0 -o addopts= --noconftest -W error -W ignore::pytest.PytestConfigWarning starpilot/system/tests/test_lightd_*.py starpilot/system/tests/test_services_h.py` (80 tests);
  - background jobs must run inside one `wsl.exe -e bash -lc '...; wait'`;
  - Git Bash heredocs eat backslashes: write scripts to files.
- **CPU on the device** (A55 little core, real frames): classifier 6–14 ms median per frame, wide guard 3 ms per
  look. Keep it there.

## 8. Conventions

- Changes are tagged "StarPinguPilot" and kept small for upstream merges. The Windows checkout is CRLF with
  autocrlf. Use `Edit` replacements, not whole-file rewrites.
- `cereal/services.h` is committed: regenerate it when `services.py` changes (`test_services_h.py` enforces this).
- The device builds from `origin/pingu` and pushes "build" commits (panda binaries). Merge them, never force-push.
- Commit or push only when the user asks. The user prefers that heavy work runs in subagents.

## 9. After drive 2: what changed on 2026-10-18 (read with `DESIGN.md` §11)

The user's direction after drive 2: no more rule patches tuned to the last drive; build a measured, principled HBA, any
architecture, a learned model on the device included. The result so far:

- **A scoreboard that does not depend on the rules under test:** `carstuff\logs\hba_tools\score.py <timeline.csv>
  <runs prefix>` stitches tracks into chains, labels them in hindsight (oncoming cars by their *pass event*, preceding by
  red in lane or a model lead, static by near-range motion with sign and side, reflector by our beam, plus `ambiguous`),
  and prints glare s / nuisance s / toggles per minute for the car's logged beam, the replay, and the replay restricted
  to frames where the recorded beam equals the replay's permit. Inventory: 46 oncoming cars on 5a, 65 on 46; the car's
  real glare is 3.4 s / 7.2 s, all reaction time at the pass; nuisance 360 s / 236 s. `train/` has the feature table and
  the classifier (`features.py`, `fit.py`, README); scikit-learn is installed in `~/spp-venv`.
- **The classifier waits for more drives** (DESIGN.md §11, step 2 results): it matches the rules, not beats them, with
  two drives. Every new night drive with rlogs + video is a training set now: run `run_par.sh`, then `score.py`, then
  `train/features.py` and `train/fit.py`.
- **`hba.py` got the policy layer** (step 3): lights born within 1.2 s of our switch-on are reflectors until they pair
  or approach, the glare override respects that, and every early re-dip backs off unless a lead, a wide lamp or
  measured motion caused it. Replay on 5a: on-periods under 2 s 21 -> 11, under 1 s 13 -> 4, glare 3.0 -> 2.4 s,
  nuisance +9 s. 65 lightd tests pass (two new). Not driven, CPU on the device not re-measured.

Next, in order:
1. Rebuild loggerd in the prebuilt tree so `starpilotLightState` is logged (the user has an agent on this). Without it the
   HBA's own reasons and `hbaProcessingMs` stay invisible.
2. Shadow mode (mode 1) drive with the stalk worked manually and generously on empty roads: closed-loop data and driver
   labels (DESIGN.md §11, "Data"). Then mode 3.
3. After each drive: `copy_routes.sh`, `timeline.py`, `run_par.sh`, `score.py`; compare the three numbers with the table
   in DESIGN.md §11. If toggles per minute did not fall on the car, the born-at-switch timing (`beam_born_s`) is the first
   thing to check against the real switch-to-sign latency in the new logs.
4. Town gate: measure streetlight density in real towns vs rural light clusters across c9, 5a, 46 before touching it.
5. Retrain the classifier at 4-5 night drives; the acceptance test is leave-one-drive-out oncoming recall per distance
   band at a fixed false-positive rate, and lead time before the pass, against the rules' own numbers.
