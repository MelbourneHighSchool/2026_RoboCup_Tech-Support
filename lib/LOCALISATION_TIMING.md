# LIDAR timing and motion compensation

The game, dashboard localisation session, and `tests.localisation` use the
timestamped hardware feed. Angular speed no longer gates scan updates in any
mode. Timing, motion-history and observation-validity checks still apply.
`scan_updates_enabled()` remains available for compatibility and returns true.

## Build and run

Rebuild **both** native extensions on the machine that will run the robot:

```bash
.venv/bin/python lib/setup.py build_ext --inplace
```

`full` is the default in the game and dashboard. Use these environment variables
before starting either application:

```bash
SOCCER_DESKEW=off .venv/bin/python main.py
SOCCER_DESKEW=rotation .venv/bin/python main.py
SOCCER_DESKEW=full SOCCER_LOCALISATION_RECORD=motion.jsonl .venv/bin/python main.py
```

`off` disables intra-scan correction only: it still uses actual beam bearings,
timestamped prediction, historical IMU yaw, and the configured mount transform.
`rotation` uses gyro motion and rotates the sensor mounting offset; `full` also
uses wheel translation. These modes share filtering and map; none gates updates by angular speed.

The existing interactive localisation test exposes the same controls:

```bash
.venv/bin/python -m tests.localisation --no-move --deskew full --record-motion stationary.jsonl
.venv/bin/python -m tests.localisation --deskew full --max-speed 200 --record-motion moving.jsonl
```

The second command uses the test's existing interactive driving controls.
It prints `[DESKEW]` lines alongside the existing diagnostics. Capture files are
created exclusively; choose a new filename for each run. Capture is optional.
Do not run a second hardware-owning application alongside the game/dashboard.

For a non-centred LIDAR, set the measured mount displacement from the robot's
rotation centre. Millimetres are positive **forward** and **left**; yaw is
clockwise-positive degrees, added to the sensor's reported bearing:

```bash
SOCCER_LIDAR_FORWARD_MM=40 SOCCER_LIDAR_LEFT_MM=15 SOCCER_LIDAR_YAW_DEG=3 \
  .venv/bin/python -m tests.localisation --no-move --record-motion mount-check.jsonl
```

Those values are an example, not the robot's measured calibration. Defaults are
zero. A wrong mount offset can create translation-like distortion during turns.

## Compare the same recorded inputs

```bash
.venv/bin/python -m lib.replay_localisation moving.jsonl --output comparison --plot-every 10
```

This runs `off`, `rotation`, and `full` with the same recorded sensor stream,
configuration and random seed. It writes one CSV per mode, `summary.json`, and
SVG raw/compensated hit-endpoint plots every ten processed scans. Use
`--mode rotation` to run one mode or `--plot-every 0` to omit plots. Output files
are not overwritten; use a fresh directory for a second comparison.

Replay controls the native estimator's clock, so speed of the replay machine
does not change the filter. Scans wait for bracketing motion samples. The replay
uses the captured event order and is repeatable, but does not reproduce the
live threads' exact scheduling or initial particle population. Compare replay
modes to each other, not bit-for-bit against the live pose trace.

The capture includes raw beams (including genuine misses and low-quality hits),
beam times, wheel samples and their read duration, IMU yaw/gyro histories,
reference/reset epochs, available classified floor samples, and drop counters.
Floor readings are recorded when forwarded through the Python feed; the capture
is not a trace of native floor/scan lock interleaving. Asynchronous recording is
bounded; failures/drops are reported. An absent footer means shutdown was not
completed. Treat a capture with drops as incomplete when comparing timing.

## Read the diagnostics

`lidar.get_deskew_status()` supplies the latest scan attempt:

- `duration_s`, `age_s`: acquisition duration and midpoint age at processing.
- `max_translation_mm`, `max_rotation_deg`: largest beam-to-midpoint motion
  used, not total motion over the entire scan. Translation excludes the mount lever arm.
- `raw_residual_mm`, `corrected_residual_mm`: mean absolute hit range errors
  against map ray casts at the **same predicted midpoint pose**, before the
  scan correction. `-1` means no acquired pose was available for that comparison.
- `history_ok`: timestamped history passed the checks for the selected mode.
- `accepted`: scan likelihood was applied; this does not promise a confident pose.
- `reason`: `accepted`, `waiting_motion`, `missing_motion`,
  `missing_yaw`, `invalid_scan_time`, `insufficient_observations`, or `not_started`.
- `processing_ms`: native processing cost after taking the estimator lock.

The dashboard's localisation state includes the same dictionary as `deskew`.
CSV pose jumps include actual robot motion; they are not pure correction errors.
Wall residuals depend on an estimated pose and dynamic obstacles; they are **not
independent position error**. Confidence and smoothness alone do not prove success.

Validate stationary poses first, then both rotation directions, straight and
sideways travel, arcs, acceleration/braking, blocked wheels and interruptions.
Use surveyed positions/headings or overhead video for independent accuracy.
During severe wheel slip, rotation-only may outperform full compensation.
Use the motor-driven rotation test below to measure tracking at higher rates.

## Timing and invalid-data policy

- The SDK returns acquisition-ordered samples with the first sample's monotonic
  timestamp. Each beam time is reconstructed using that timestamp and the active
  scan mode's sample interval, **before** any angular selection. This assumes
  regular sampling within the returned scan; it is not a per-node hardware clock.
  Full SDK buffers and scans ending in the future are rejected. Packet-loss timing
  within a scan is a remaining limitation of this SDK interface.
- Angular bins retain the chosen return's actual angle and timestamp. Zero range
  is an explicit no return. Nonzero low-quality/out-of-range returns are unknown
  and omitted, rather than converted into missing-wall evidence.
- SH-2 report timestamps are aligned from their wrapping microsecond domain to
  the host monotonic clock. They include SH-2 report-delay correction. The HAL
  uses host poll arrival rather than a hardware interrupt timestamp, so transport
  and polling uncertainty still need checking on the Pi. Future or >250 ms old
  report times are excluded from the compensation history.
- Sequential wheel QDR reads use their host midpoint and expose their full read
  duration. Reads taking >50 ms are excluded. Wheel interpolation will not cross
  gaps >250 ms; yaw/gyro interpolation will not cross gaps >100 ms. Neither
  interpolation extrapolates beyond its available samples.
- Prediction integrates timestamped wheel/gyro intervals, independent of Python
  call delay, using constant-twist steps. Delay rewind/replay uses the same motion
  model. Motion gaps invalidate the current pose and are never bridged by an
  invented constant velocity. New valid scans can reacquire it.
- IMU reset and explicit re-zero increment an epoch and discard timing history.
  The game re-zeros once per pause/recovery sample batch, rather than repeatedly
  invalidating scan history while paused. Pause again to collect a new reference.
- Legacy `predict_odometry(vx, vy, omega, dt)` remains available for synthetic
  and LIDAR-only utilities. It keeps contiguous approximate history, but cannot
  supply per-beam motion compensation; those callers should leave deskew off.

## Offline checks

```bash
.venv/bin/python -m pytest -q tests/test_localisation_motion.py tests/test_localisation_replay.py tests/test_line_sensors.py tests/test_hardware_controller.py
.venv/bin/ruff check
```

Native tests use known simulated trajectories for compensation error, rather
than only checking the filter's own confidence. Coverage includes actual beam
bearings, unknown versus missing observations, mount offsets, both turn signs,
lateral motion, slip, delayed processing, angle wrapping, history gaps, resets,
historical yaw scoring and scan acceptance during high-speed rotation. Capture/replay tests check
repeatability, CSV/SVG output, mode equivalence when stationary and loss reporting.

## Motor-driven rotation recording

```bash
.venv/bin/python -m tests.localisation_rotation --motor-rpm 100 --duration 5 --record-motion rotation-100.jsonl
.venv/bin/python -m lib.replay_localisation rotation-100.jsonl --output comparison-rotation-100 --plot-every 10
```

After acquiring a pose, the test records 3 seconds stationary, 5 seconds clockwise,
3 seconds stationary, 5 seconds anticlockwise, and 3 seconds stationary. It uses
zero commanded translation and no dribbler. `--direction cw|ccw|both` selects turns;
`--duration` sets seconds per turn (maximum 30). `--motor-rpm` sets yaw wheel RPM
(maximum 400), **not body degrees per second**; the printed gyro rate is the
measured body rate. Increase RPM between recordings to exercise higher rates.
`--deskew` defaults to rotation and supports off/full for comparison.
The test stops on Ctrl+C, stale raw scans or unavailable/stale IMU data. Pose loss
is reported but does not stop the timed turn, so recovery can be observed. It does
not hold position against physical translation. Capture files must be new paths.

## Hardware validation so far and next steps

These are scan-to-map residuals at estimated poses, not surveyed position errors.
The operator also observed visibly straighter walls in compensated SVGs.

- Straight 200 mm/s: whole-run means were 32.72 mm off, 32.75 rotation,
  32.59 full. No meaningful benefit demonstrated.
- Diagonal 500 mm/s: on 30 moving scans (>5 mm midpoint displacement),
  full-mode paired raw/corrected means were 27.87/28.51 mm; 8 improved.
  Translation compensation slightly worsened the paired residual.
- Diagonal commanded 2000 mm/s: on 16 moving scans, paired means were
  40.99/39.56 mm; 11 improved. Full replay still fit worse than off on those
  scans (39.56 vs 38.50 mm). Peak midpoint displacement was 64.47 mm in about
  103 ms, suggesting roughly 1250 mm/s measured translation during steady
  motion, not proof of actual physical speed.
- Manual rotation: on 112 accepted turning scans (>0.5 degrees midpoint
  rotation), rotation-mode paired means were 42.01/40.32 mm; 77 improved.
  This recording predates removal of the rotation gate.
- Motor rotation at 400 wheel RPM: all 102 fast-turning scans (>5 degrees
  midpoint rotation) improved, with paired means 162.01/37.70 mm (76.7%).
  Whole-run mean was 108.21 mm off, 38.12 rotation, 38.07 full; p95 was
  211.17/44.50/44.13 mm. All 194 scans were accepted and poses reported valid.
  Maximum midpoint rotation was 33.63 degrees over a 115 ms scan, roughly
  586 deg/s if steady. Full measured at most 1.72 mm midpoint translation.
  The capture was complete without drops. These results strongly support
  rotation compensation, but validity alone did not identify the bad off fit.

The off/rotation/full modes, capture health reporting, synthetic geometry tests,
CSV diagnostics, optional SVG output, and bounded motor rotation trial are retained
for repeatable regression testing. Generated `/comparison-*/` folders are ignored
by Git and preserved locally. Keep raw JSONL under `recordings/` for future replay;
comparison CSVs cannot reconstruct the original beam or motion histories.
Replaying old captures after code changes measures the new implementation and
need not reproduce their original summaries, particularly after gate removal.

Next work, in order:

1. **Repeat rotation validation.** Record clockwise/anticlockwise trials at
   100, 200 and 400 wheel RPM at several field positions, including near walls.
   Compare each direction and speed separately. Confirm stationary behaviour,
   pose continuity and repeatable residual/visual improvement. Record a measured
   position/heading or overhead video to check actual pose errors.
2. **Resolve translation accuracy.** Capture forward, backward, left and right
   runs at 500 mm/s, followed by higher commands. Use raw wheel/gyro history,
   physical travel/time and command/RPM limits to explain the apparent speed
   plateau and test lateral signs, wheel scale and timestamp offsets. Change one
   variable at a time; do not tune against aggregate residual alone.
3. **Combine motion.** Test arcs and diagonal travel while turning, then starts,
   stops and reversals. Full should improve moving scan geometry without worsening
   measured tracking relative to rotation-only. If that fails repeatedly, use
   rotation-only operationally while translation is investigated; default full
   has not been changed by this cleanup.
4. **Evaluate confidence and recovery.** Use known-position checks, partial wall
   visibility and deliberate sensor interruptions. Measure pose error, correction
   age and recovery time alongside residuals. The off high-speed run reporting
   valid poses despite large residuals warrants confidence calibration; do not
   treat accepted scans or `valid=True` as independent proof of accuracy.

For each trial retain the raw capture, software revision, deskew mode, mounting
settings, commanded motion, measured motion, capture/drop status and reference
measurements. Use the same recording for all replay modes. Compare paired
raw/corrected residuals and moving-only mean/p95; require repeatable improvements
across trials before declaring a feature validated.
