# Short-exposure frames for lightd (comma 4 / OS04C10 stagger HDR)

Status: investigation only, no code. Verdict below is **medium-hard**, so per the brief nothing is implemented yet.
Date: 2026-10-24. Tree: `pingu` @ f9f5507015.

## 1. The question

At night camerad's AE runs the road and wide OS04C10s near max: about 2270 of 2352 lines, gain 1.1-1.7. Headlights clip and
bloom, pairs merge, colour is lost and the lamps smear. lightd wants a second, short exposure (about 1-2 ms) of the narrow camera,
and of the wide one if it costs nothing. It should come every frame, or at least at 1-2 Hz, as raw data.

The one hard rule: **openpilot's processed frames must not change**, and nothing may change when the param is off.

## 2. Short answer

| Layer | Can it do long → pixel path + short → RAW at the same time? | Effort |
|---|---|---|
| Kernel / Spectra ISP (CSID + IFE, cam_isp UAPI) | **Yes.** It's a supported, standard config. | easy |
| camerad plumbing (2 in-ports, 2nd output, fences, delivery) | Yes. About 200-300 lines in `spectra.cc` / `camera_qcom2.cc`, all behind a param. | medium |
| **OS04C10 sensor mode** | Only through the vendor *staggered HDR* mode. Every known table for it is **RAW10** with **different PLL / analog / HTS / VTS** settings, so the frames openpilot sees **would change**. | **hard / blocked** |

The ISP is not the blocker. The sensor is: the only way to get a second exposure in the same 50 ms frame is the stagger mode.
At 20 fps and full resolution, that mode only fits on MIPI as RAW10 (§5.2). Switching road and wide to RAW10 changes the model's
input at night, which is exactly when it matters. Getting a RAW12 stagger mode, or proving RAW10 is harmless to the model,
needs the datasheet or an OmniVision FAE table, plus device work.

## 3. Kernel / ISP evidence

Kernel: `commaai/agnos-kernel-sdm845` master @ `4a6a2ab8e50f`, `drivers/media/platform/msm/camera/`.
Line numbers below refer to that commit.

1. **Several in-ports per ISP acquire.** `cam_isp_context.c:2216-2248` copies `cmd->num_resources` `cam_isp_resource`s and passes
   them as `num_acq`. `cam_ife_hw_mgr.c:1568` (`cam_ife_mgr_acquire_hw`) loops over each `cam_isp_in_port_info` and calls
   `cam_ife_mgr_acquire_hw_for_ctx()` (`:1407`) for it. Each in-port has its own `vc` / `dt` / `format`
   (`third_party/linux/include/media/cam_isp.h:185-212`). camerad's `device_acquire()` already takes `num_resources`
   (`system/camerad/cameras/spectra.cc:85-93`). Today it is always 1.
2. **Pixel and RDI in one context.** `cam_ife_mgr_acquire_hw_for_ctx` acquires a CSID IPP path if the in-port has pixel outputs
   and a CSID RDI path if it has RDI outputs (`cam_ife_hw_mgr.c:1434-1452`). The context is "RDI only" only if no pixel port was
   acquired (`:1617-1620`). Mixed contexts are the normal case.
3. **Same PHY is required, and it is allowed.** The root resource must have the same `res_type` (PHY) for every in-port
   (`cam_ife_hw_mgr.c:1356-1376`). On a CSID that is already reserved, a new CID must match the PHY and lane config
   (`cam_ife_csid_core.c:643-668`). Two VCs from one sensor meet both checks.
4. **Different VC/DT means a different CID on the same CSID.** The CID acquire first tries the CSID the context already holds
   (`cam_ife_hw_mgr.c:1001-1043`). `cam_ife_csid_cid_get()` (`cam_ife_csid_core.c:296-338`) reuses a CID only when vc/dt match,
   otherwise it takes a free one. There are `CAM_IFE_CSID_CID_RES_MAX = 4` per CSID (`cam_ife_csid_core.h:21`). Each path gets
   its own `vc`/`dt` (`cam_ife_csid_core.c:893`), and RDI paths are separate from IPP (`:822-858`).
5. **The RDI output lands on the same IFE.** The VFE-in for RDI is reserved on `ife_devices[csid hw_idx]` (`cam_ife_hw_mgr.c:854-945`),
   and the RDI out port follows it (`:640-718`).
6. **Resources on SD845.** CSID0/IFE0, CSID1/IFE1 and CSID-lite/IFE-lite, each with 1 IPP and up to 4 RDI. Today wide = PHY0 and
   road = PHY1 each hold a full CSID IPP, and the driver camera (PHY2, `ISP_BPS_PROCESSED`, which is RDI only) sits on its own CSID.
   Adding RDI_0 to the road and wide contexts uses CSID RDI paths nobody uses now. The driver camera is not touched.
7. **Request completion.** The ISP context counts buf-done per output fence (`cam_isp_context.c:415-437`). With FULL + RDI_0
   in one request, both fences must signal before the request retires. camerad only *waits* on the FULL fence, so a slow or missing
   RDI fence does not delay openpilot's frame. A *missing* RDI buf-done would leave requests stuck in the kernel's active list,
   though (risk R5).

So, for lead 3: **yes.** A second `cam_isp_in_port_info` on the same PHY with `vc = <short VC>`, `dt = 0x2b`,
`format = CAM_FORMAT_MIPI_RAW_10` and one out port `CAM_ISP_IFE_OUT_RES_RDI_0` sits next to today's in-port. Today's in-port moves
from `vc = 0` to the long VC. It is the same HW topology camx uses for PDAF or embedded-data VCs. No kernel change is needed.

## 4. What exists for the sensor side

### 4.1 Current mode (this tree)

`system/camerad/sensors/os04c10_registers.h` uses the vendor table
`OS04C10_AA_00_02_17_wAO_2688x1524_MIPI728Mbps_Linear12bit_20FPS_4Lane`: linear, RAW12 (`dt 0x2c`), HTS `0x85c` = 2140,
VTS `0x938` = 2360, 0x3821 = 0x00, MIPI 728 Mbps/lane. That gives a row time of 50 ms / 2360 = 21.19 µs and
`exposure_time_max` = 2352 (`os04c10.cc:57`). The IFE downscales 2688x1520 to 1344x760 (`out_scale = 2`).
These values match upstream's mici `ife_downscale_override_array_os04c10` (PR #36806 base).

Existing details that matter here:
- `0x3510-0x3512` (short exposure = 64 lines) and `0x350c/0x350d` (short gain) are already written, but do nothing in linear mode.
  `OS04C10::getExposureRegisters` (`os04c10.cc:111-120`) writes the long gain into the short gain every frame.
- `0x4813 = 0xe4` looks like a 4-entry VC map (frame0→VC0 … frame3→VC3). This is a guess: datasheet Q3.

### 4.2 Upstream draft PR commaai/openpilot#36806 (open, draft, Adeeb, last commit 2025-12-17)

The last real commit message says "works with vc, 20fps and staggered hdr". The diff against upstream:
- `hw.h`: **driver camera removed** from `ALL_CAMERA_CONFIGS`. Because the init table is per sensor *class*, the driver OS04C10
  also gets the stagger table, and its BPS blobs (`bps_cfg[sensor->num()]`) and RAW12 settings no longer match.
- `spectra.cc`: in-port `vc = 0x1`, so the IFE pixel path takes the **long** frame and the short VC (0) is dropped by the CSID.
- `os04c10.cc`: `CAM_FORMAT_MIPI_RAW_10`, `CSI_RAW10`, `frame_stride = w*10/8`. `bits_per_pixel` stays 12 and black level 48,
  which is wrong for RAW10 (see §5.1).
- Register changes (the same set as the 2024 table):

| reg | linear (now) | stagger | likely meaning (needs datasheet) |
|---|---|---|---|
| 0x0301 | e4 | 84 | PLL divider / config |
| 0x0305 | b6 | 61 | PLL multiplier |
| 0x0325 | 62 | 7a | PLL2 multiplier |
| 0x370a | 02 | 00 | analog timing |
| 0x3748/4a/4c/4e | 02 | 00 | analog timing |
| 0x4090 | 04 | 14 | BLC / HDR-related |
| 0x3022 | 61 | 01 | ? |
| 0x3621 | b0 | 90 | analog |
| 0x380c/d HTS | 08 5c (2140) | 04 2e (1070) | line length, halved in stagger |
| 0x380e/f VTS | 09 38 (2360) | 09 db (2523) | frame length |
| 0x3821 | 00 | 04 | bit 2 = stagger HDR enable (guess) |

### 4.3 comma's 2024 stagger HDR, `cc3550df2a` ("camerad: OS HDR", #32112)

Table `DP_2688X1520_NEWSTG_MIPI0776Mbps_30FPS_10BIT_FOURLANE`, run at 20 Hz through VTS. It has the same PLL / HTS / VTS / 0x3821
as above, plus more analog changes (0x366a, 0x3685, 0x3698-0x36a1, 0x3706/0x370b, 0x3741-0x374f, 0x3790/91, 0x4305, 0x4507, 0x484b,
0x4837 = 0x14, 0x5001 = 0x00, …). Short exposure was fixed at 64 lines (`0x3512 = 0x40`) with short gain = long gain.

That camerad took the *whole* stream raw through RDI and merged it in OpenCL (`process_raw.cl`, `os04c10_cl.h`):
`frame_height = 1520*2 + hdr_offset`, `hdr_offset = 64*2 + 8`. So in that era long and short came in one interleaved frame on a
single VC. #36806 splits them onto two VCs instead. The VC split register is unknown (datasheet Q3). RAW10 black level was 64
then (`BLACK_LVL 64` in `os04c10_cl.h`). The HDR path was later removed (`03fa24ee82` "use os binning", upstream #33838
"disable os HDR"), and nobody has documented image-quality validation of it since.

## 5. Why it is not "easy": impact on openpilot's frames

### 5.1 RAW10 vs RAW12

- The IFE works in a 14-bit domain: black level is applied as `black_level << (14 - bits_per_pixel)` (`ife.h:99`,
  `spectra.cc:596,1297`). For RAW10, `bits_per_pixel = 10` and the black level have to be re-measured. 48 is a RAW12 value, and
  2024's RAW10 value was 64.
- Quantization: one RAW10 step is 4 RAW12 steps. The gamma LUT `10x/(1+9x)` has slope 10 near black. One RAW10 code near black
  becomes about 10/1023 × 255 ≈ **2.5 output codes** in 8-bit Y, against about 0.6 today. Read noise at gain 1.1-1.7 dithers
  some of it, but night shadows and the road surface (the model's main night input) get coarser and posterize.
- No IFE-processed camera in comma's tree has ever run RAW10. ar0231, ox03c10 and os04c10 are all RAW12. The linearization LUT,
  CCM and gamma were tuned on RAW12.

### 5.2 Why RAW10 is forced (MIPI budget)

Each stagger row period carries one long row and one short row. If #36806 really runs 20.0 fps with VTS = 2523, the row period
is 50 ms / 2523 ≈ 19.8 µs. Per lane, over 4 lanes:

- linear RAW12 today: 2688 × 12 b / 21.19 µs / 4 ≈ **380 Mbps** of 728 (52 %).
- stagger RAW10: 2 × 2688 × 10 b / 19.8 µs / 4 ≈ **678 Mbps** of 776 (87 %).
- stagger RAW12: 2 × 2688 × 12 b / 19.8 µs / 4 ≈ **814 Mbps**. That is more than 776 before any protocol overhead.

RAW12 stagger at full resolution and 20 fps would need a lane rate of about 950 Mbps or more, so a new PLL table from
OmniVision (datasheet Q1, Q2). The SD845 CSIPHY handles that rate, but `MIPI_SETTLE_CNT` and `data_rate` in `spectra.cc` would
need recomputing (`camera_freqs.py`).

### 5.3 Timing changes

- **Row time / rolling shutter.** With the numbers above, the row time goes from 21.19 to about 19.8 µs, so long-frame readout
  skew drops by about 6 %. `readout_time_ns` (11 ms, "set from logs") and the EOF estimate would need re-measuring. The model sees
  a slightly different rolling-shutter shear. That is small, but not zero.
- **Exposure units.** In OmniVision stagger modes the exposure registers may count in rows or in row-pairs (datasheet Q4). If they
  count row-pairs, AE's `ev_scale`, `exposure_time_max` and every lightd EV→lux calibration (`DESIGN.md` §7, low-beam calibration)
  shift by 2x.
- **Max long exposure.** It becomes about VTS − short − margin. With VTS 2523 and short 64 that is still at least 2352, so the
  AE ceiling can be kept. Needs confirming (Q5).
- **Analog settings differ** (table in §4.2): column / ADC timing and BLC registers. Fixed-pattern noise, black level and
  response near saturation can all change.
- **Cabin camera and sync.** All three sensors must SOF within 0.2 ms at startup (`spectra.cc:1592-1633`) and stay locked (FSIN
  regs 0x3002/0x3822/0x3832…). Road and wide must both run stagger with identical timing. The driver camera must **keep the
  linear RAW12 table** (BPS blobs, DMS model), so the init table has to be chosen per `camera_num`, not per sensor class.
  Whether a linear and a stagger sensor stay FSIN-locked at 20 Hz is unknown (Q6, and the device test).

**Bottom line:** with any table we have, enabling the short exposure changes the long frame that modeld, encoderd, the UI and
logging consume, at night. That breaks the "processed frames unchanged" goal. It could still be done, but it would need an A/B
model-quality check on night drives (§8, phase 3), and that is not "easy".

## 6. Options considered

| Option | Short frame | openpilot frames | Verdict |
|---|---|---|---|
| A. Stagger RAW10 (#36806 table), long VC → IFE pixel, short VC → RDI_0 | full-res RAW10 every frame | **change** (RAW10, timing, analog) | medium-hard; needs night A/B validation |
| B. Stagger RAW12 with a faster PLL (OmniVision table) | full-res RAW12 every frame | small timing change only, if row time is kept | best target; **blocked on datasheet / FAE** |
| C. Alternate long/short frames in time (linear mode) | 1 per N frames | a dark frame every N → modeld breaks | rejected |
| D. Same-VC RDI copy of the long frame | raw long frame | unchanged | no help: still clipped |
| E. Shorter long exposure at night | n/a | changes AE for the model | rejected |
| F. IFE stats (BHIST/HDR_BE) | histograms only | unchanged | no lamp structure; rejected |

## 7. What it would take (implementation plan, option A or B)

All of it gated by a new bool param `CameraShortExposure` (default off), read once at camerad start. When off, every code path
below is skipped and the register tables and acquire structs stay byte-identical to today.

1. **Sensor (`os04c10.cc`, `os04c10_registers.h`)**
   - Add `stagger_patch_os04c10[]` (the §4.2 diff; for option B, the vendor RAW12 table). In `SpectraCamera::openSensor()`,
     after the probe and before the `init_reg_array` write, apply it only when `param && cc.camera_num != 2`.
   - For those cameras: `mipi_format = CAM_FORMAT_MIPI_RAW_10`, `frame_data_type = CSI_RAW10`, `bits_per_pixel = 10`,
     `frame_stride = 2688*10/8 = 3360`, measured `black_level`, re-measured `readout_time_ns`, `exposure_time_max` from the
     new VTS. Leave the linearization LUT in the 14-bit domain.
   - `getExposureRegisters`: stop copying the long gain into 0x350c/d. Write a **fixed** short exposure (0x3510-0x3512, e.g.
     47-94 lines, about 1-2 ms) and a **fixed 1x** short gain. A fixed short exposure makes the frame photometric, so lightd
     can tell lamps from reflectors by absolute brightness.
2. **ISP acquire (`SpectraCamera::configISP`)**: build `cam_isp_in_port_info in[2]`. `in[0]` is today's struct with
   `vc = LONG_VC`. `in[1]` has the same PHY and lanes, `vc = SHORT_VC`, `dt = 0x2b`, `format = RAW_10`, `line_start = 0`,
   `line_stop = 1519`, `height = 1520`, and one out port `{RDI_0, CAM_FORMAT_MIPI_RAW_10, 2688, 1520}`. Then
   `device_acquire(m->isp_fd, session_handle, isp_resource_array, 2)`.
3. **Buffers and fences**: allocate `ife_buf_depth` raw buffers of 3360 × 1520 bytes (as `camera_bufs_raw`) and map them like
   `camera_map_bufs()` does for BPS. Add `sync_objs_rdi[MAX_IFE_BUFS]`, created in `enqueue_frame`, destroyed in
   `destroySyncObjectAt`.
4. **Per-frame IFE packet (`config_ife`)**: `num_io_configs = 2` (FULL as today, plus RDI_0 pointing at the raw buffer with its
   own fence). Add one more `cam_buf_io_cfg` to the size. Set the HFR blob to `num_ports = 2` with a second
   `port_hfr_config` for RDI_0. The BW blob already votes `rdi_vote[0]`. RDI write traffic is about 5.1 MB × 20 Hz ≈ 102 MB/s.
5. **Frame handling (`handle_camera_event`)**: unchanged wait on the FULL fence. Then a **non-blocking** check on the RDI fence
   (timeout 0-5 ms). On success, and when a decimation counter hits (default every frame, configurable to 2 Hz), hand the raw
   buffer to the publisher. On RDI failure, count and log it, and never touch the openpilot path.
6. **Delivery to lightd**, preferred first:
   - **VisionIPC** stream `VISION_STREAM_ROAD_SHORT` / `VISION_STREAM_WIDE_SHORT`: add to the enum in
     `msgq_repo/msgq/visionipc/visionbuf.h` before `VISION_STREAM_MAX`. Serve the raw RAW10 buffers zero-copy, no CPU copy.
     Not registered with encoderd/loggerd, so it is not logged.
   - Optionally a small cereal message in `CustomReserved13` (`custom.capnp:375`): frameId, timestampSof, short integLines,
     gain, black level, plus an 8-bit log-encoded 2x2-binned **upper-half crop** at 1-2 Hz. Mark it `should_log = False` in
     `services.py`.
7. **lightd**: a new reader module. `hba.py` is not touched; another agent owns it. It unpacks RAW10, does a 2x2 Bayer bin, and
   finds blobs above a fixed threshold.

Rough effort: 1-2 days of code, plus parked bring-up, plus night validation drives. Option B also waits on OmniVision.

## 8. Device test plan (to run once the code exists; all parked first)

Prereq: an SSH session, the car parked at night, ignition on, `openpilot` stopped (`tmux kill-session -t comma`, or
`sudo systemctl stop comma`). Run camerad by hand. **Do not set `LOG_RAW_FRAMES`**: it dereferences an unset `cur_camera_buf` for
IFE-processed cameras.

Phase 0, baseline with the param off:
1. `echo -n 0 > /data/params/d/CameraShortExposure; cd /data/openpilot/system/camerad && ./camerad`
2. In another shell, check rates with a small `SubMaster` loop (`cereal.messaging.SubMaster(['roadCameraState',
   'wideRoadCameraState', 'driverCameraState'])`, printing `sm.freq`, `frameId` and `timestampSof` once a second). All three
   should be at 20.0 Hz. Check `dmesg | grep -i -E "csid|ife|cam_isp"` is clean. Save a `system/camerad/snapshot.py` capture
   as the reference.

Phase 1, sensor bring-up (param on):
1. `echo -n 1 > /data/params/d/CameraShortExposure`, then start camerad as above. Expect `camera N synced on frame_id_offset` for
   **all three** cameras. Driver sync is the critical check.
2. Rate and timing: for 60 s log `timestampSof` from all three states. Pass: 20.00 ± 0.05 Hz each, road-wide and road-driver SOF
   delta < 0.2 ms that does not drift, no `frame ID skipped` / `sync failure` / `requests skipped` in the camerad log.
3. Kernel: `dmesg` has no CSID `ERROR_*`, `CPHY_PKT_HDR`, `FIFO overflow` or `unbounded frame` messages. Read the CSID IRQ status
   for the RDI path if available (`cat /sys/kernel/debug/camera_*` on AGNOS, if present).
4. RDI frames: the new VisionIPC stream arrives at 20 Hz. Dump one buffer and check its size is 3360 × 1520. Unpack RAW10 and view
   it: a correct short exposure looks about 30-40x darker than the long one, with streetlights unclipped.
5. Which VC is which: if the "short" stream is bright and the openpilot image dark, swap `LONG_VC` / `SHORT_VC`.

Phase 2, image parity for openpilot (param on vs off, same scene, parked):
1. Lens covered: average black level for long (YUV) and short (raw). Set `black_level`.
2. A static night scene with the param off, then on. Compare `snapshot.py` road/wide images: mean Y, Y histogram in the lowest
   10 % (posterization), noise σ on a flat patch, colour of a known lamp. Also compare `integLines`/`gain`/
   `exposureValPercent` at steady state. If the exposure-unit factor (Q4) is 2x, it shows up here.
3. The driver camera image and `driverStateV2` look normal with the param on.

Phase 3, driving (only after phases 1-2 pass):
1. Short night drive with lateral and longitudinal disengaged (or no engagement at all), param on, `rlog` recording.
   Check modeld `frameDropPerc`, `modelV2` lane-line / path confidence against a param-off drive on the same road, and
   lightd's blob detections on the short stream against the human-labelled HBA events.
2. No engagement with the param on until a model A/B over several night routes shows no regression.

## 9. Risks

- R1: RAW10 posterization at night degrades modeld (§5.1). This is the main risk and why this is not "easy".
- R2: driver-camera FSIN sync breaks when only road/wide run stagger. That would drop DMS, which is a safety function.
- R3: the exposure unit changes in stagger, which breaks AE scaling and lightd's lux calibration.
- R4: analog register set copied from a 30 fps vendor table run at 20 fps. Unvalidated FPN/BLC behaviour.
- R5: a missing RDI buf-done leaves requests stuck in the ISP active list, which can stall the FULL path too. Needs a watchdog:
  turn the RDI port off after N misses and requeue.
- R6: `0x350c/d` is written with the long gain today. Changing it is harmless in linear mode but alters stagger behaviour.
- R7: upstream itself has never shipped this (#36806 is still a draft). No quality reference exists.

## 10. Open questions for the OS04C10 datasheet / OmniVision FAE

1. Is there a **staggered 2-exposure RAW12** mode at 2688×1520 and 20 fps? What MIPI lane rate does it need, and what is the
   sensor's max D-PHY rate per lane?
2. PLL register semantics (0x0301-0x0307, 0x0323-0x0325): SCLK and MIPI clock for both tables. Can stagger keep the
   **same row time (21.19 µs)** as linear?
3. VC assignment in stagger mode: is it 0x4813 (0xe4)? Which exposure goes on VC0 and which on VC1? Can the short frame use
   a different DT or VC only?
4. Units of 0x3500-0x3502 / 0x3510-0x3512 and of HTS/VTS in stagger mode (rows or row-pairs). Constraints on short exposure:
   min, max, the relation to 0x3832 or similar.
5. Max long exposure in stagger: VTS − short − N? What is N?
6. FSIN slave behaviour when the master is linear and the slave stagger, or the other way round, at equal frame period.
7. Can the short exposure be windowed (top half only) or vertically subsampled to cut bandwidth, so RAW12 fits at 776 Mbps?
8. Black-level target registers (0x4002/0x4003, 0x4090) and whether BLC behaves differently in RAW10 vs RAW12.
