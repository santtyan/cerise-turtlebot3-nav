# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# CERISE — TurtleBot3 Multi-Robot Digital Twin

Digital twin research project (UFG/EMC, advisor Alisson Assis Cardoso) with two independent papers sharing the same ROS2/Gazebo codebase and three-TurtleBot3 platform:
- **LARS 2026** (`docs/paper_lars/`, submitted) — PPO reinforcement learning for multi-robot task allocation (MRTA), grounded in a camera+YOLO digital twin. Central finding is a *negative result*: PPO does not beat the greedy `nearest-free` heuristic, and a causal test (action masking) refutes the leading explanation for the gap.
- **LAFusion 2026** (`docs/lafusion_paper/`, in progress, deadline 2026-09-04) — Extended Kalman Filter fusing YOLO visual detection with odometry, replacing a discrete fallback. Reports both a correction-instant gain (+23.7%) and a continuous-error degradation (-12.7%) under sparse visual coverage, grounded in Kalman-filtering-under-intermittent-observations theory.

## Build and test

This is a standard ROS2 Humble ament_python workspace (`src/cerise_nav/`), not a general Python project — commands assume ROS2 is sourced.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

There is no unit-test suite beyond `test_e2e_dataset_collector.py` (synthetic, no ROS/Gazebo required):
```bash
PYTHONPATH=src/cerise_nav python3 test_e2e_dataset_collector.py
```
Most validation instead happens through standalone scripts in `scripts/` that run against real recorded data or synthetic Monte Carlo trajectories (see Architecture below) — there's no `pytest`/`unittest` convention to follow here; new checks are typically new scripts, not test files.

Python deps for the RL/EKF pipeline (no ROS rosdep key) install via `pip install -r requirements.txt` (stable-baselines3, sb3-contrib, gymnasium, scipy, tensorboard).

## Architecture

### Two runtime "eras" that still coexist

The codebase grew as one digital twin serving increasingly different downstream consumers, and traces of each era remain live:
1. **Original YOLO-dataset era** (`dataset_collector.py`, `collect_teleport.py`) — collects labeled camera frames to train the YOLO detector (`model_robot_detector.pt`, mAP@0.5=0.995). This training happened once; these scripts aren't run routinely anymore but the model they produced is a dependency of everything below.
2. **RL task-allocation era** (`src/cerise_nav/cerise_nav/rl/`, `rl_task_allocator.py`, `task_allocator.py`) — the LARS paper. `rl/allocation_env.py` is an *analytical* Gymnasium environment (no ROS/Gazebo — each `step()` is pure arithmetic over `nav_model.py`), which is why training 500k timesteps takes ~2h on CPU instead of days. The exact same observation encoding (`rl/obs_encoding.py`) is shared between offline training (`AllocationEnv`) and the ROS2 inference node (`rl_task_allocator.py`) — if they ever diverge, the trained policy receives garbage in production. `task_allocator.py` is the `nearest-free` greedy baseline.
3. **EKF sensor-fusion era** (`ekf_fusion_node.py`, `association.py`, `projection.py`) — the LAFusion paper. Replaces the RL era's implicit odometry/YOLO handling with an explicit per-robot EKF: odometry drives prediction, YOLO detections (gated by Mahalanobis distance in `association.py`) drive correction.

All three eras consume the same live sensor pipeline: overhead camera (`world_with_camera.world`, fixed at `(0,0,3)`, 90° downward pitch) → YOLOv8n → `yolo_detector.py`, which publishes `/robot_detections`. `projection.py` holds two independently-maintained pixel↔world projections (FOV heuristic, used in production; camera-intrinsics-based, confirmed geometrically equivalent to the heuristic — see LAFusion Methodology) — don't assume changing one changes the other's behavior.

### EKF math: correct() unified, predict() intentionally stays split

As of 2026-08-24 (`refactor/unify-ekf-core`), `correct()` and `r_from_confidence()` — bit-for-bit identical across `ekf_fusion_node.py` (ROS2 production node), `scripts/eval_ekf_vs_baseline.py` / `scripts/eval_ekf_continuous_error.py` (offline evaluation against recorded bags), and `scripts/validate_ekf_synthetic.py` (synthetic Monte Carlo validation) — live in one place: `src/cerise_nav/cerise_nav/ekf_core.py` (no rclpy dependency, importable standalone). All four consumers import from there; a fix to Joseph-form covariance or Mahalanobis gating now only needs to happen once. `predict()` deliberately was **not** unified and still exists separately in each file: production/offline-eval consume already-integrated absolute odometry and step via delta (`predict_from_odom`/`predict`, `F=I`), while the synthetic validator drives an explicit unicycle kinematic model from `(v, w)` with a non-trivial Jacobian (`predict(state, cov, v, w, dt, Q)`) — these are genuinely different motion models, not the same function under different names, so forcing one interface over both was rejected (see research notes in the `refactor/unify-ekf-core` branch history). `Q`/`COV_CAP` remain per-consumer parameters, never hardcoded in `ekf_core.py` — production/eval use `Q_DIAG=(0.5, 0.5, 0.25)` calibrated against real bag drift, the synthetic validator uses `q_diag=(1e-6, 1e-6, 5e-7)` calibrated for its near-ideal synthetic noise; sharing one value across both would silently miscalibrate one of them. All consumers use the numerically-stable **Joseph form** covariance update and **Mahalanobis gating with `S=P+R`** (not just `P`) — reproduces the paper's published numbers exactly (bit-for-bit): NEES≈2.955/NIS≈1.982 synthetic, +23.7%/−5.3% aggressive/light-drift gain, −12.7% ATE / −4.2% RPE continuous-error degradation.

**Covariance ceiling is empirically justified, not just simple.** The `COV_CAP=0.05` clip (`ekf_fusion_node.py`) is mathematically inelegant (breaks PSD structure) — a fading factor (AFKF, scaling `Q` instead of clipping `P`) is the textbook-correct alternative. It was implemented and tested (`eval_ekf_vs_baseline.py:COV_UPDATE_MODE`, still present as an experimental flag) and **loses empirically**: without a cap, or with one loose enough to differ from the clip, `P` grows enough that Mahalanobis gating starts accepting the wrong robot's detection — the exact failure mode the clip exists to prevent. Gain under aggressive drift dropped from +23.7% (clip) to −16.5% (fading, no cap). Don't re-litigate this without new data.

### Where validation actually lives

There's no single test command that proves a pipeline change is correct — validation is split across scripts that target different guarantees:
- `scripts/validate_ekf_synthetic.py` — synthetic ground-truth Monte Carlo (30 seeds), checks filter consistency via NEES/NIS before trusting it against real data.
- `scripts/eval_ekf_vs_baseline.py` / `eval_ekf_continuous_error.py` — EKF vs. odometry-only vs. ground truth, against the three recorded scenarios in `bags/` (stationary/straight/curve, MCAP format — requires `ros-humble-rosbag2-storage-mcap`, not installed by default on Humble).
- `scripts/stats_tests.py` — shared statistical toolkit (paired Wilcoxon signed-rank, Cliff's delta, bootstrap CI) used by both papers' result tables; reused rather than reimplemented per-paper.
- `scripts/sweep_load.py` — LARS load-regime robustness sweep (six inter-arrival rates, 500 episodes/point).
- `scripts/eval_ekf_continuous_error.py` reports both **ATE** (absolute per-sample error, no rigid alignment needed since estimate/odometry/ground-truth already share the world frame via the fixed camera) and **RPE** (Sturm et al. 2012 convention — error of the position delta between consecutive readings, isolates local step-to-step drift from ATE's accumulated error). This is the project's adopted metric convention for any future trajectory-error script — follow it rather than reporting only a bare mean/RMSE.

### Reproducibility package

`bags/reproducibility_package/` pins a specific git SHA, hyperparameters (`params.yaml`), camera calibration, and world/URDF files needed to rerun the LAFusion results independently. `docs/lafusion/` mirrors (copies, not symlinks) the live scripts/code/figures for paper packaging — its own README says explicitly to edit the originals in `scripts/`/`src/`, not the copies, and to resync manually after changes.

## Papers and skills

Before writing/editing any paper section, generating a scientific figure, adding a citation, or building a presentation slide, load the matching skill (procedures are kept out of this file to stay short):
- `verify-citation` — verify a bibliographic reference against a primary source before it enters a `.tex`.
- `verify-claim-against-code` — verify every technical/numerical claim against code or a fresh execution, not memory.
- `paper-figure-style` — house style for matplotlib figures in this project's papers.
- `anonymize-paper` — double-blind anonymization checklist (`.tex` + PDF metadata).
- `critical-presentation-review` — rigor checklist for slides/presentations: every design decision justified, code architecture and benchmarks shown, hard questions pre-empted.

## Commands you won't guess

- **Never use Docker** — scope decision already made for the LARS/LAFusion papers.
- **Always `python3`**, never `python`.
- Compile LaTeX papers: `pdflatex -interaction=nonstopmode main.tex` — run **twice** (cross-references only resolve on the 2nd pass). Count pages: `pdfinfo main.pdf | grep Pages`.
- Bring up the multi-robot pipeline: `./launch_3robots_with_camera.sh`, wait **70s+** before running any other node (Nav2 takes a while to stabilize).
- Stuck Gazebo/Nav2 zombie nodes: `pkill -9 component_container` before relaunching — restarting without this leaves DDS ports held.
- If a ROS2 node keeps logging but no CLI subscriber receives anything (even with compatible QoS), that process's DDS state is stuck — kill and relaunch it, no need to restart the whole environment. **Caveat**: confirming `ros2 topic hz` looks healthy right after a relaunch does not guarantee publication stays stable through a subsequent `ros2 bag record` — always check the actual message count in the recorded bag, don't trust a prior `hz` check alone.

## Git

- **Never include `Co-Authored-By: Claude`** in commits — explicit user preference.
- Always check `git status` before a broad `git add` — the repo accumulates loose files from old sessions (e.g. `docs/RSL/`, `docs/paper_lars/`) that shouldn't be committed without confirming what they are first.

## Environment (Wayland/GNOME + VSCode snap)

- Legacy X11 screenshot tools (`scrot`, `import`, `gnome-screenshot`) return a black screen under Wayland — there's no fully automated screenshot capture in this environment.
- Env vars injected by the VSCode snap (`GTK_PATH`, `GDK_PIXBUF_MODULEDIR`, etc.) break native graphical binaries (RViz2 and similar) with GLIBC symbol errors — `unset` them before running system graphical binaries.
- RViz2 has no programmatic screenshot API on Humble (no ROS2 service, no Python binding) — don't attempt that route for paper figures; see the `paper-figure-style` skill for the matplotlib-over-bag-data alternative instead.
