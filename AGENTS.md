Any time you don't understand something about how this project works, try and figure it out. If you still don't understand, ask for help. Once you figure it out/get an answer, add an explanation of the problem and how to solve it to this file, so you don't run into the same problem again. Do not just save every change you make here, only add to this file if you didn't understand something and you had to spend time working it out.
Whenever you finish writing code, lint with `.venv/bin/ruff check` (or `.venv/bin/ruff check <path>`). If that does not work (for example `.venv` is missing or the command fails), tell the user. CI uses Ruff **0.16.2** (pinned in `.github/workflows/ruff.yml`); install the same version locally (`pip install 'ruff>=0.16.2'`) so local results match GitHub Actions. Ruff 0.15.x has fewer default rules and will pass checks that 0.16 fails.

## Legacy Python motor / movement tools

**Movement module (`legacy/movement.py`)** – retained for calibration and old utilities:

- **`init_motors(i2c_addresses, calibration_file="calibration_data.json")`** → `(motors, motor_modes)`. Creates up to 8 `PowerfulBLDCDriver` instances, sets PID/limits, loads saved calibration from the JSON file, applies `set_ELECANGLEOFFSET` / `set_SINCOSCENTRE`, and puts motors in FOC speed mode (command mode 12). Exits with an error if the calibration file is missing or has fewer motors than requested. Use this for normal operation.
- **`get_motors_for_calibration(i2c_addresses)`** → `(motors, motor_count, normalized_addresses)`. Creates drivers and sets PID/limits only (no calibration, no FOC). Used by `calibration/motors.py`.
- **`calibrate_motors(motors, motor_count, i2c_addresses, calibration_file="calibration_data.json")`**. Puts each motor in calibration mode, runs physical calibration, reads `ELECANGLEOFFSET` and `SINCOSCENTRE`, and writes them to the JSON file. Call this only when you need to (re)calibrate; use `calibration/motors.py` as the entry point.
- **`MovementController.move(direction, speed, rotation, rotation_speed, yaw, dribbler=0)`**. Non-blocking: only updates command targets. A background drive thread (50 Hz) owns accel ramping, yaw correction, and I2C speed writes. `direction` / `rotation` / `yaw` use the project frame where `0` = startup-forward and `90` = startup-right (clockwise positive). `rotation_speed` is a 0.0-1.0 scalar for yaw-correction strength.
- **`imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)`** → `float`. Convert raw IMU yaw into the project's startup-relative heading frame. The upside-down IMU reports clockwise turns as decreasing yaw, so the correct conversion is `wrap_angle_deg(startup_yaw - imu_yaw)`.
- **`_prompt_i2c_addresses()`** → list of I2C addresses. Interactive prompt (number of motors, then each address). Used by scripts that don't hardcode addresses.

**Problem:** Accel and yaw correction used to run inside `move()`, so a stalled control loop (slow LIDAR/camera/strategy) produced a huge `dt` (accel jumps) and paused heading correction between calls.

**Solution:** `MovementController` runs a fixed-rate drive thread. `move()` only stores targets under a lock; the thread applies `MAX_VELOCITY_CHANGE_PER_SEC` with a capped `dt`, recomputes yaw correction, and writes motors. Callers can still stall without distorting the ramp or freezing yaw hold (yaw stays at the last value passed to `move()` until the next call).

**External driver:** `steelbar_powerful_bldc_driver.PowerfulBLDCDriver(i2c_bus, address)` (e.g. from `git+https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver`). Driver firmware version must be 3. Important methods: `set_ELECANGLEOFFSET`, `set_SINCOSCENTRE`, `get_calibration_ELECANGLEOFFSET`, `get_calibration_SINCOSCENTRE`, `set_speed`, `get_speed_QDR`, `update_quick_data_readout`, plus PID/limits and mode config.

**Calibration workflow:** Run `python calibration/motors.py` once (or after hardware change) to create/overwrite `calibration_data.json`. After that, `init_motors(...)` loads that file and does not run physical calibration. Calibration file format: `{"motors": [{"address": <int>, "elecangleoffset": <int>, "sincoscentre": <int>}, ...]}`.

## Movement: speed (mm/s) to motor RPM (`legacy/movement.py`)

**Problem:** What are `a_speed`, `b_speed`, `c_speed`, `d_speed`, and `max_trans_rpm` when `speed` is a given value (e.g. 500)?

**How it works:**
- `speed` is in mm/s. `direction`, `rotation`, and `yaw` use the project heading frame where `0` = startup-forward and `90` = startup-right. The IMU does **not** natively use this sign convention; convert raw IMU yaw with `imu_yaw_to_relative_yaw(imu_yaw, startup_yaw)` before passing it into `move()`.
- The legacy Python controller and native C++ controller convert the global translation heading into the robot's local frame with `local_direction = yaw - direction + 45` (degrees). The `+45` rotates into the wheel basis because the wheels sit on the diagonals, leaving the front edge clear.
- Wheel velocities in mm/s use the current code signs: `a_value = -sin(local_direction)*speed`, `b_value = +cos(local_direction)*speed`, `c_value = +sin(local_direction)*speed`, `d_value = -cos(local_direction)*speed`.
- Conversion to RPM uses wheel diameter (e.g. `WHEEL_DIAMETER = 50` mm in `defence.py`): `mmps_to_rpm = 60 / (diameter * π)`.
- Motor speeds in RPM: `a_speed = a_value * mmps_to_rpm`, and similarly for b, c, d.
- Yaw correction uses `yaw_error = wrap_angle_deg(rotation - yaw)`, so positive error means "turn clockwise to match the target heading". That RPM is **subtracted** from every wheel (`rpm - yaw_correction_rpm`). Adding it spun the robot anticlockwise (`tests/dribbler_rotate.py` with `rotation=90`, `yaw=0`). The requested yaw RPM is limited to `max_yaw_rpm`, then the translation RPMs are scaled down if needed to leave headroom for yaw while preserving the requested movement direction.
- With diameter 50 mm: `mmps_to_rpm ≈ 0.382`. Example: speed 500, `direction = yaw = 0` gives `local_direction = 45°`, which corresponds to local forward in the current wheel basis.

**Problem:** QDR wheel odometry had the correct forward speed (`vx`) but the opposite sign on `vy`.

**Solution:** This project's yaw is clockwise-positive and field `+y` is right (`90°`). Body `vy` is defined as **left**, so convert global velocity with `body_vy = gx * sin(yaw) - gy * cos(yaw)` (the left axis is `(sin yaw, -cos yaw)`). The ROS/CCW formula `−gx sin + gy cos` is the **right** axis here and will flip `vy`. `loc_predict_odometry` uses the same left-positive body frame; rebuild the C++ extension after changing it.

## LIDAR localization (`lib/lidar_module.cpp` + `lib/localisation.cpp`)

**Solution — 3-DOF Monte Carlo localization (MCL) in C++:**

- Particles track `(x, y, yaw)` on the known pitch map (outer walls + goal hardware segments).
- Scan thread keeps explicit no-return bearings (`hit=false`) as well as valid hits. MCL bins them into 2° angular bins before scoring.
- Likelihood is incidence-aware: ray casting returns range + wall normal; near-normal walls are expected to return, grazing walls (`|cos θ|` ≲ 0.25) are expected to miss. Hits use a Gaussian+outlier mixture with incidence-scaled σ; misses get a soft expected/unexpected miss probability instead of free-space penalties.
- Confidence combines inlier quality, particle spread, and visible wall-normal diversity (one wall alone cannot claim a strong along-wall pose). Resample only when ESS drops below 50% of the particle count so partial scans keep diversity.
- `predict_odometry(vx, vy, omega, dt)` propagates particles between scans (call from Python each control loop). Pass IMU gyro z as `omega_deg_s` (clockwise positive). `vx`/`vy` are body-frame mm/s with **vx = forward** and **vy = left**.
- Estimation runs in a background thread; Python reads the latest pose.
- Fast rotation gate: when `|omega|` exceeds 50 deg/s, MCL skips LIDAR scan updates and dead-reckons via predict only. Scan updates resume after `|omega|` stays below 25 deg/s for 150 ms. `lidar.scan_updates_enabled()` reports whether scans are currently accepted.

**Problem:** LIDAR often drops walls at extreme incidence angles. The old hit-only Gaussian treated every surviving return as a perfect first-wall match and ignored missing bearings, so partial scans could under-constrain or destabilize the pose.

**Solution:** Model visibility from incidence angle; score explicit misses; lower confidence when geometry is under-constrained. Offline check: `python test_grazing_localisation.py` (uses `lidar.test_mcl_*` hooks, no hardware).

**Python API** (import with `from lib import lidar`):

1. `lidar.init(port, baudrate)` — start scan thread.
2. `lidar.start_coordinates(pitch_x, pitch_y)` — start MCL thread and build pitch map.
3. `lidar.set_imu_yaw(yaw_deg)` — feed startup-relative IMU yaw as a soft MCL yaw prior (call each control loop).
4. `lidar.predict_odometry(vx_mm_s, vy_mm_s, omega_deg_s, dt_s)` — propagate particles between scans (also drives the fast-rotation gate from `omega_deg_s`).
5. `lidar.get_pose()` → `(x, y, yaw_deg, confidence)` — last estimate (None if not confident).
6. `lidar.get_coordinates()` → `(x, y)` — backward-compatible confident position only.
7. `lidar.get_coordinates_info()` → `(x, y, yaw_deg, confidence, ok)` — diagnostics.
8. `lidar.is_coordinates_ready()` → `bool` — true once first confident pose exists.
9. `lidar.scan_updates_enabled()` → `bool` — false while MCL is pausing LIDAR updates during fast rotation.
10. `lidar.shutdown()` — stops localization, scan thread, and motor.

**IMU in MCL:** `HardwareController` enables calibrated gyroscope and game rotation vector without the magnetometer. Call `set_startup_yaw()` after sampling `get_raw_imu_yaw()`, then feed `get_yaw()` directly to `lidar.set_imu_yaw()` and clockwise-positive `get_gyro_z_deg_s()` to `predict_odometry()`. MCL applies a soft Gaussian yaw prior (σ = 45°) on every scan update so LIDAR still owns fine, drift-free heading while the IMU breaks the 180° field symmetry. Init/recovery particles sample yaw around the IMU when a reading exists, otherwise over ±180°.

**Stuck on "Waiting for first pose estimate...":** While waiting for `is_coordinates_ready()`, still call `predict_odometry(0, 0, omega, dt)` each loop (even when stationary). Predict applies translation/yaw process noise; without it, particles collapse after the first resample and often never reach the confidence threshold. Also print `get_coordinates_info()` / `get_scan_count()` during the wait so low confidence vs empty scans is visible.

**Rebuild after changing `lib/lidar_module.cpp` or `lib/localisation.cpp`:** `python lib/setup.py build_ext --inplace`

**Build error `cannot find -lsl_lidar_sdk` / g++ exit code 1:** The Python extension links against the RPLidar SDK static library. If `rplidar_sdk/output/Linux/Release/` does not exist or has no `libsl_lidar_sdk.a`, build the SDK first: `make -C rplidar_sdk/sdk`. Then run `python lib/setup.py build_ext --inplace` again.

## Team-frame convention (`defence.py` / `simulate.py`)

**Problem:** `defence()` and `goalie()` used to contain separate yellow/non-yellow branches. That made the cyan-side behavior drift from the yellow-side behavior, including cases where `defence()` would face its own goal instead of the enemy goal.

**Solution:** Keep the strategy code in a single "yellow-side" frame inside `defence.py`, and let `simulate.py` rotate cyan bots into that frame before calling `defence()` or `goalie()`. The required transform is a 180° rotation of the full world state: `(x, y) -> (PITCH_WIDTH - x, PITCH_HEIGHT - y)` and `yaw/direction/rotation -> angle + 180° (mod 360)`. After the controller returns, rotate `direction` and `rotation` back into the real field frame.

## Label Studio YOLO pre-annotation (`training/ls_yolo_backend.py`)

**Problem:** Manually drawing Ball/Bot boxes on every frame is slow once a detect model already exists.

**Solution:** Run a small Label Studio ML backend that loads `training/runs/open-soccer-detect-m/weights/best.pt` (override with `LS_ML_WEIGHTS`) and returns axis-aligned `RectangleLabels` predictions. Same-host media is resolved from `LABEL_STUDIO_DATA_DIR`, so no API token is required. Uploaded files map `/data/...` → `label-studio-data/media/...`. Local Files storage URIs look like `/data/local-files/?d=Photos/foo.png`; `d` is relative to the data dir itself (`label-studio-data/Photos/foo.png`), not `media/`.

1. Start: `systemctl --user enable --now label-studio-ml` (unit from `systemd/label-studio-ml.service` → `~/.config/systemd/user/`), or `.venv/bin/python training/ls_yolo_backend.py`.
2. In Label Studio → Open Soccer → Settings → Model, connect `http://127.0.0.1:9090` (interactive preannotations off).
3. Open a task (or use Retrieve predictions) — only `Ball` / `Bot` are emitted; other model classes (e.g. goals) are skipped. Confidence defaults to `0.25` (`LS_ML_CONF`), or `model_score_threshold` on `<RectangleLabels>` if set.
4. After retraining, `systemctl --user restart label-studio-ml` so it reloads `best.pt`. Restart the same unit after changing `ls_yolo_backend.py`.

## Extract video frames into Label Studio Photos (`training/extract_video_frames.py`)

**Problem:** New game recordings need to become Label Studio Local Files images with clip-stable names (`output23_000001.png`, …) so train/val can split by clip.

**Solution:** Given a folder of videos, extract frames at a configurable rate (default 1 fps) into `label-studio-data/Photos/`. Each video gets the next free `output<N>` prefix (highest existing N + 1); source filenames are not used. Then sync/refresh the Label Studio Local Files storage so the new files become tasks.

```bash
python training/extract_video_frames.py /path/to/videos
python training/extract_video_frames.py /path/to/videos --fps 0.5
python training/extract_video_frames.py /path/to/videos --fps 2 --photos-dir label-studio-data/Photos
```

## Delete skipped Label Studio tasks (`training/delete_skipped_label_studio.py`)

**Problem:** Skip in Label Studio only writes a cancelled annotation. The task stays in the DB and the PNG stays in `Photos/` (or `media/upload/`), so a later Local Files sync can recreate the task.

**Solution:** Delete skip-only tasks (`cancelled_annotations > 0` and no real annotations) and image files that no remaining task still points at. Dry-run by default; stop Label Studio before `--apply`.

```bash
python training/delete_skipped_label_studio.py
python training/delete_skipped_label_studio.py --apply
```

## Hailo-8 AI HAT+ ball inference (`training/`)

**Problem:** Ultralytics NCNN on the Pi CPU (~5 FPS) never uses the AI HAT+ (Hailo-8, 26 TOPS). Hailo only runs compiled `.hef` models. Training is axis-aligned YOLO26 detect (not OBB).

**Solution:**

1. On the Pi: `bash scripts/verify_hailo.sh` (needs `hailo-all` + `/dev/hailo*`).
2. Optional — rewrite rotated Label Studio boxes to axis-aligned AABBs in the DB so you can review them in the UI (stop Label Studio first):  
   `python training/export_label_studio.py --update-label-studio` (dry-run), then `--update-label-studio --apply`.
3. Export YOLO-detect dataset: `python training/export_label_studio.py` → `training/datasets/open-soccer-detect/`. Train/val is **by clip**, not by frame: early uploads are `{hash}-output_NNNN.png` (the hash is per file) grouped by Label Studio import burst; Local Files under `Photos/` use prefixes like `output10_000001.png`. After export, `python training/check_split_overlap.py` reports leftover near-duplicates.
4. Train + ONNX (project `.venv`): `python training/train.py --size n` → `training/exports/open-soccer-detect-n/model.onnx` (XPU via `training/xpu_patch.py`; AMP off on XPU; `batch=-1` AutoBatch uses `torch.xpu` memory, not CUDA; use `--no-mosaic` if XPU scatter/gather errors).
5. On x86 with Hailo DFC: `python training/compile_hailo.py --hw-arch hailo8` → `open-soccer-detect-n_hailo_model/model.hef`.
6. Copy that folder to the Pi. `tests/model.py` and `lib/camera.py` use [`lib/hailo_ball.py`](lib/hailo_ball.py) (HailoRT) for inference.

YOLO26’s end2end TopK head is unsupported on Hailo. `compile_hailo.py` cuts at `/model.23/Mul_2` (decoded xyxy boxes) and `/model.23/Sigmoid` (class scores) as **two** HEF outputs so each gets its own INT8 range. The HEF uses input normalization only; `lib/hailo_ball.py` merges the streams and runs conf filter + NMS on the host.

**Empty detections / all class max=0.0 with float32 output:** Usually the HEF was compiled with a single end node (`/model.23/Transpose` or `Concat_3`) that concatenates pixel boxes (~0–640) with sigmoid scores (~0–1). One qp scale (~3+) cannot represent confidences, so dequantized scores collapse to ~0 even though ONNX float inference is fine. Fix: recompile with `python training/compile_hailo.py` (defaults are now `Mul_2` + `Sigmoid`), copy the new `model.hef` to the Pi. Diagnose with `python tests/model.py --image model_test_image.png --debug` — you want two outputs and Ball max ≫ 0. Sanity-check the ONNX with onnxruntime if needed.

**UINT8 vs FLOAT32:** Keep HailoRT output as `FormatType.FLOAT32` (`quantized=False`). Manual UINT8 + `quant_info` also drops detections when qp is wrong/missing.

**Live camera channel order:** Picamera2 format `"RGB888"` exposes a BGR-ordered numpy array (and `"BGR888"` exposes RGB bytes). Ultralytics training and the HEF expect RGB, so passing an `"RGB888"` frame directly to `HailoBallDetector` swaps red and blue and can collapse Ball scores. Convert live frames with `cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)` immediately before inference. Keep BGR for existing OpenCV HSV, drawing, and preview code. `tests/model.py --image` already converts OpenCV-loaded BGR images to RGB; therefore image tests and PyTorch validation do not reveal this live-camera bug.

## Striker ball-hiding hysteresis (`striker.py`)

**Problem:** With separate `BALL_HIDING_START_DIST` / `BALL_HIDING_END_DIST`, a naive `if dist < END: aim elif dist >= START: hide` leaves a dead zone between them. Crossing that band (or fluttering near either threshold) snapped `rotation` between wall-facing (`120`/`240`) and `0` / goal heading, so the bot oscillated CW/CCW.

**Solution:** Persist hiding in the returned `steering_state` bool. Enter hide when `dist >= START`; stay hidden until `dist < END`, then aim. `main.py` / `simulate.py` must feed the previous steering flag back in each call.

**Aim angle:** Do not aim at goal-centre Y. Take the angular midpoint of directions that hit `GOAL_BACK_Y_MIN..MAX` on `CYAN_GOAL_BACK_X`, clipped to rays that pass through the goal mouth (`CYAN_GOAL_MOUTH_X`, side walls at `GOAL_SIDE_WALL_Y_MIN/MAX` inset by `BALL_RADIUS`). If the clipped back-wall window is empty, bank onto the inside of the opposite side wall as close to the back as possible, keeping at least `SIDE_WALL_CLEARANCE_DEG` (2°) from the near wall when the mouth sector allows it. Aim/kick directions must also clear both mouth posts by `BALL_RADIUS + GOAL_LINE_WIDTH/2` (angular mouth clearance alone can still clip a post). Kick only when the actual `yaw` (kick impulse direction) passes `kick_direction_scores`, not merely when yaw is close to the aim angle. If neither a back-wall nor a valid rebound shot exists, dribble toward own goal and pitch centre (`x - SHOT_REPOSITION_PULL_X`, `GOAL_CENTRE_Y`) instead of kicking.

## Blinka / `import board` on Pi 5 (`ModuleNotFoundError: lgpio` / pip SWIG error)

**Problem:** Blinka’s Pi 5 backend does `import lgpio`. That module is not a pip wheel on Python 3.13+: `pip install lgpio` (or any requirements line for that package) downloads the source tarball, runs `swig -python`, and fails with `error: command 'swig' failed: No such file or directory`. `adafruit-lgpio` only has a cp313 aarch64 wheel; other Python/arch combos fall back to the same SWIG build (the log still says “Building wheel for lgpio”).

**Solution on Raspberry Pi OS** — use the distro package, not pip:

```bash
sudo apt-get install -y python3-lgpio
cd ~/Soccer   # or wherever the repo is
rm -rf .venv
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```

`--system-site-packages` is required so the venv can see apt’s `lgpio`. Recreate the venv if it was made without that flag.

**If you must stay on pip** (64-bit Pi, Python 3.13–3.15), install Adafruit’s prebuilt wheel *before* `requirements.txt`, and never the PyPI sdist:

```bash
pip install lgpio --find-links https://github.com/adafruit/lgpio-python-wheels/raw/main/wheels/
```

Do not `pip install lgpio` from PyPI and do not `sudo apt install swig` just to make that compile. Desktop/x86 does not need `lgpio` at all.

## Raspberry Pi native-library `Bus error` (SIGBUS)

**Symptom:** Python scripts such as `defence.py`, `lib/camera.py`, or `tests/model.py` crash with `Bus error` before a traceback while importing `numpy`, `cv2`, or `torch`. In Ultralytics this can misleadingly surface as a missing `YOLO` or `__version__` import; confirm with `import torch` or `import numpy` directly.

**Cause:** The Pi kernel is using 16 KB pages (`getconf PAGE_SIZE` prints `16384`). Some pip native wheels—especially `numpy`, `torch`, and dependencies installed through `opencv-python` or `ultralytics`—SIGBUS on this page size.

**Fix for camera, NumPy, and OpenCV (without pip Torch):** use Raspberry Pi OS packages:

```bash
sudo apt-get install python3-numpy python3-opencv python3-libcamera python3-picamera2
python3 -m venv .venv --system-site-packages  # or use system Python directly
```

Do not pip-install `numpy`, `opencv-python`, or `picamera2` into that environment.

**Fix when Torch / Ultralytics is required:** switch to 4 KB pages, then reinstall. On Pi OS Bookworm, first confirm `getconf PAGE_SIZE` is `16384`, add `kernel=kernel8.img` to `/boot/firmware/config.txt`, reboot, and verify it returns `4096`. Recreate the virtual environment with `--system-site-packages` so Picamera2/Libcamera remain available, then install:

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
```

Continue to use the apt-provided NumPy, OpenCV, and Picamera2 packages rather than pip versions.

## Other bots in recorded replays

**Problem:** Folder playback already follows video presentation timestamps, but Tk can deliver a programmatic slider update's callback on the next event pump, after `_updating_slider` is cleared. Treating that callback as a seek resets the playback clock every frame and slows real-time playback.

**Solution:** `LogPlaybackControls._on_slider()` ignores the current frame index. Only an actual position change seeks and resets the clock; automatic slider updates preserve accumulated elapsed time.

**Problem:** The simulator already rendered trailing bot coordinate pairs, but the game loop did not log them; session recordings also saved only ball annotations.

**Solution:** `main.py` appends friendly and enemy world-coordinate pairs after the 13 controller fields in plain/live logs. `RecordingSession.record_game(..., other_bots=...)` stores the same positions in an optional JSON CSV column; `game_event_tokens()` expands them for the existing simulator parser. Camera callbacks include all bot detections in the same timestamped event as the ball, saved as the optional `bots` JSON column and drawn by `annotate_video_frame()`. Missing columns in older recordings mean empty lists. Bot projection uses the bounding-box centre for both bearing and radial distance.

## Dashboard motor ownership and camera snapshots

**Problem:** Calibration command mode 15 runs autonomously, so writing speed zero alone does not cancel physical calibration. Motor setup also indexed motor 4 unconditionally despite documenting the dribbler as optional.

**Solution:** `disable_calibration_motors()` writes zero torque, switches command mode to 2, leaves calibration operating mode for FOC, and writes zero torque again, attempting every driver and reporting failures. `calibrate_motors()` uses this cleanup on success, cancellation and timeout, and saves only after all motors and cleanup succeed. Verify the transition with the installed Pi/driver firmware before relying on remote cancellation. Setup now handles missing dribblers and cleans up partially initialized drivers. `MovementController.stop()` terminates its thread, so a new target-driving session must create a new controller.

**Problem:** The existing MJPEG preview draws old inference results onto newer capture frames; its pixels are therefore unsuitable for exact distance samples or colour picking.

**Solution:** `Camera(diagnostics=True)` publishes `get_diagnostic_snapshot()` containing an owned copy of the unannotated inference frame, matching ball/bot boxes, and the source capture's monotonic timestamp. The dashboard renders from that snapshot and rejects stale samples. Pixel picking uses a separately retained lossless source frame, not JPEG/overlay pixels. Diagnostic copies are disabled for ordinary camera callers.
## Motor trust and localisation feedback

**Problem:** Low motor trust during movement can be caused by the estimator itself, rather than physical wheel slip. `compute_wheel_odometry_trust()` compares speed magnitudes: a 30% discrepancy gives 40% trust, and a discrepancy of 50% or more gives zero. Previously, both `main.py` and `lib/localisation_service.py` multiplied measured wheel velocity by this score before MCL prediction. `LidarVelocityEstimator` differentiates the fused MCL pose, which `loc_predict_odometry()` also updates, so the comparison is circular. Its smoothing (alpha 0.3, initialized at zero) can also trigger low trust during acceleration.

**Diagnostic approach:** Compare wheel, lidar, and fed velocities in `tests/localisation.py`. A controlled comparison using unscaled measured wheel velocity can distinguish feedback-induced underprediction from wheel calibration/slip problems. Raw measured wheel velocity is now the default in the game loop, service, and localisation test; trust remains diagnostic in the service/test. Runs at 350 and 500 mm/s tracked successfully with raw odometry despite trust dips, including zero trust at 500 mm/s. Run `python -m tests.localisation --scaled-odometry --max-speed 200` to reproduce legacy scaling; `--raw-odometry` remains an explicit alias for the default. A durable fix should model uncertain odometry with motion uncertainty and use time-aligned, independent evidence for slip rather than treating this score as a velocity multiplier.

## Native hardware controller (`lib/hardware_controller.cpp`)

**Native kicker GPIO:** Pi 5 GPIO chip numbering varies with the kernel. Discover
the chip using its device-tree `compatible` entry (`raspberrypi,rp1-gpio`), rather
than assuming gpiochip0 or gpiochip4. The configured Blinka pin's `id` is its BCM
line number. `linux_kicker.cpp` uses Linux GPIO v2 directly and restores input with
pull-down after each pulse, matching `legacy/kicker.py`; writing low alone does not
restore that input configuration. `main.py` passes the pin to the native controller
and must not also construct a Python Kicker for the same line.

**Problem:** The supplied `PowerfulBLDCdriver` C++ files use Arduino `Wire`, so they cannot compile directly on Linux. A native motor worker also cannot acquire Python's shared I2C lock without depending on the GIL.

**Solution:** `linux_wire.h/.cpp` implements the driver's small Wire surface using addressed Linux `I2C_RDWR` messages, checking transfer failures. The native controller serializes motor operations with its own mutex; Linux serializes individual transfers with the Python display. Only one controller may own any motor or IMU address, since register-selection/read pairs must not interleave with another owner. `main.py`, dashboard driving, and hardware tests use the pybind11 `lib.hardware_controller.HardwareController`; its workers use no Python callbacks. Calibration JSON is matched by address, and four wheels plus an optional dribbler are supported. Physical calibration remains in `legacy/movement.py` and must run separately from the game controller.

Build both extensions with `.venv/bin/python lib/setup.py build_ext --inplace`, or only motors with `SOCCER_HARDWARE_ONLY=1 .venv/bin/python lib/setup.py build_ext --inplace`. Rebuild on the Pi with its runtime Python. Offline protocol/lifecycle tests: `.venv/bin/python -m unittest tests.test_hardware_controller`. See `lib/HARDWARE_CONTROLLER.md` for the API and ownership/shutdown behavior.

## Native BNO08x / SH-2 (`lib/imu/linux_bno08x.cpp`)

**Problem:** MCL heading moved opposite to the physical turn while fast-rotation gating disabled scan correction. Quaternion yaw was reversed with `startup_yaw - raw_yaw`, but sensor gyro Z was passed through with its original sign.

**Solution:** The native adapter exports `-sensor_gyro_z * 180/pi` so clockwise turns produce positive relative yaw changes and positive angular velocity with the upside-down mounting. Python callers pass this value directly without another negation. The absolute-rate rotation gate is unaffected. Native protocol tests cover both rate signs; rebuild the hardware extension on the Pi after updating.

**Problem:** The imported Adafruit wrapper needs Arduino/BusIO, but the underlying SH-2/SHTP sources are portable C. SH-2 stores a global session; `sh2_open()` can return success after reset timeout, ignores the HAL open result, and its product-ID operation originally had no timeout. Returning zero from HAL writes also triggers an unbounded retry loop.

**Solution:** Compile the four SH-2 core `.c` files as C11 and the Linux adapter as C++17; exclude the Arduino wrapper. The adapter claims one session per process, checks transport errors and reset completion, and returns negative write errors. `getProdIdOp` has a one-second timeout. I2C reads repeat the four-byte SHTP header; the adapter reconstructs bounded transfers from 32-byte chunks, timestamps arrival with a monotonic microsecond counter, and re-enables reports after resets outside the callback. All IMU service and motor I/O share the controller's native bus mutex.

`main.py` now samples native raw yaw, sets the startup reference in C++, and reads relative yaw and gyro for LIDAR. `move(direction, speed, rotation, rotation_speed, dribbler=0)` has no yaw argument. Each drive tick uses native yaw; after 100 ms without a quaternion, yaw correction is disabled while translation uses the last known heading. Gyro freshness is separate. Both reports resume automatically; the startup reference is retained across a sensor reset, so re-zero while paused if its raw origin shifts. Stop joins both workers and releases SH-2. Python IMU/dashboard workflows must run separately. The hardware-only build now includes both motors and IMU.

## Separate ball and bot distance calibration

**Problem:** Bot projection and dashboard overlays reused the ball radial-distance polynomial, even though object box centres need different distance fits.

**Solution:** `Camera` loads `ball_distance_calibration.json` and `bot_distance_calibration.json` independently at the active camera resolution. Missing/invalid bot calibration yields bearings but no bot distances, never the ball fit. In dashboard **Distances**, select Ball or Bot to edit separate samples and fits; bot sampling requires exactly one detected bot and measures centre-to-centre distance. Saving backs up and reloads only the selected class via `reload_distance_calibration(path, target="bot")` (default target remains ball).

## LIDAR acquisition timestamps and slip uncertainty

**Problem:** Dashboard rotation stopped around 90 degrees because `LocalisationSession` required an MCL correction every 0.5 s, while the native rotation gate deliberately suspends corrections above 50 deg/s. Fresh raw scans and IMU reports did not prevent the stale-pose abort.

**Solution:** A gate closure from a recent valid MCL fix permits prediction for at most 3 s since the last correction, with continuing raw scans, IMU reports, and available finite yaw/gyro readings. On gate reopening, correction must resume within 0.5 s, still within the 3 s overall limit. Gate toggles cannot renew either deadline; an actual correction resets the allowance. The state includes `rotation_prediction` and retains the true `mcl_age_s`. The dashboard's worker watchdog and invalid-pose/pause checks remain active. Regression checks: `.venv/bin/python -m pytest tests/dashboard_test.py -q`.

**Problem:** Timing the nonblocking `grabScanDataHq()` call measures retrieval of an already completed revolution, not acquisition. The SDK timestamp's clock and reference point must be checked before using it with odometry history.

**Solution:** `grabScanDataHqWithTimeStamp()` returns the first sample's timestamp. The bundled Linux SDK uses `CLOCK_MONOTONIC` microseconds (`sdk/src/arch/linux/timer.cpp`), matching Linux steady-clock odometry. Retain `LidarScanMode` from `startScan()` and add `(count - 1) * us_per_sample / 2` before converting to seconds. Use the original count including misses, before sorting/filtering. This is a whole-scan midpoint approximation, not per-beam deskewing. Invalid timestamps are excluded from localisation; scans older than retained odometry cannot be fully rewound and are rejected.

**Problem:** In Pi reverse runs, wheel travel exceeded physical travel while recovery did not trigger. Stationary diffusion of 8 mm/sqrt(s) does not represent that moving uncertainty.

**Solution:** Translation noise per axis is `hypot(8, coefficient * measured_speed) * sqrt(dt)`, with coefficient 0.30 sqrt(s) initially; wheel velocity remains the mean. Noise is added only on prediction, not scan replay. `lidar.set_motion_noise(0)` restores legacy noise for timestamp-only comparisons; `python -m tests.localisation --motion-noise 0` exposes this option (default 0.30). The coefficient needs Pi validation. Offline checks: `.venv/bin/python -m unittest tests.test_localisation_motion`.
