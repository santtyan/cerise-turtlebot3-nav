# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# CERISE — TurtleBot3 Multi-Robot Digital Twin

Digital twin research project (UFG/EMC, advisor Alisson Assis Cardoso) with two independent papers sharing the same ROS2/Gazebo codebase and three-TurtleBot3 platform:
- **LARS 2026** (`docs/paper_lars/`, submitted — the actually-submitted PDF/source is `docs/paper_lars/SUBMISSAO_FINAL_2026-08-05/`, per its own `LEIA-ME.md`; the other subfolders there are earlier drafting rounds) — PPO reinforcement learning for multi-robot task allocation (MRTA), grounded in a camera+YOLO digital twin. Central finding is a *negative result*: PPO does not beat the greedy `nearest-free` heuristic, and a causal test (action masking) refutes the leading explanation for the gap.
- **LAFusion 2026** (`docs/lafusion_paper/`, submitted 2026-09-04, decision pending 2026-10-05) — Extended Kalman Filter fusing YOLO visual detection with odometry, replacing a discrete fallback. Reports both a correction-instant gain (+23.7%) and a continuous-error degradation (-12.7%) under sparse visual coverage, grounded in Kalman-filtering-under-intermittent-observations theory.

## Build and test

This is a standard ROS2 Humble ament_python workspace (`src/cerise_nav/`), not a general Python project — commands assume ROS2 is sourced.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

There is no `pytest`/`unittest` suite — new checks are standalone scripts, not test files, and each is invoked directly:
```bash
PYTHONPATH=src/cerise_nav python3 test_e2e_dataset_collector.py       # synthetic dataset-collector pipeline, no ROS/Gazebo required
python3 test_ekf_predict_regression.py                                 # golden-master: production vs. offline-eval predict() must still agree bit-for-bit on their shared 'clip' path
```
`test_ekf_predict_regression.py` (added 2026-08-28) does not import `ekf_fusion_node.py` or `eval_ekf_vs_baseline.py` directly — both require ROS packages (`rclpy`, `rosbag2_py`) not guaranteed available outside a sourced ROS environment, and the `predict()` logic itself has no ROS dependency. It reimplements both `RobotEKF.predict()` verbatim from the current source text of each file and asserts they agree step-by-step on `Q_DIAG`/`COV_CAP`/state/covariance for two synthetic odometry sequences (one at the real ~29Hz robot rate, one at a much smaller dt) — the real-rate sequence alone is insufficient because `COV_CAP=0.05` saturates within a single step at that rate, silently masking any divergence in the `Q` formula itself (found while validating this test — verify a new regression test actually fails before trusting it passes). If either source file's `predict()` changes, update the reimplementation in this test to match, per the comment at the top of the file.

Most validation otherwise happens through standalone scripts in `scripts/` that run against real recorded data or synthetic Monte Carlo trajectories (see Architecture below).

Python deps for the RL/EKF pipeline (no ROS rosdep key) install via `pip install -r requirements.txt` (stable-baselines3, sb3-contrib, gymnasium, scipy, tensorboard).

## Architecture

### Two runtime "eras" that still coexist

The codebase grew as one digital twin serving increasingly different downstream consumers, and traces of each era remain live:
1. **Original YOLO-dataset era** (`dataset_collector.py`, `collect_teleport.py`) — collects labeled camera frames to train the YOLO detector (`model_robot_detector.pt`, mAP@0.5=0.995). This training happened once; these scripts aren't run routinely anymore but the model they produced is a dependency of everything below.
2. **RL task-allocation era** (`src/cerise_nav/cerise_nav/rl/`, `rl_task_allocator.py`, `task_allocator.py`) — the LARS paper. `rl/allocation_env.py` is an *analytical* Gymnasium environment (no ROS/Gazebo — each `step()` is pure arithmetic over `nav_model.py`), which is why training 500k timesteps takes ~2h on CPU instead of days. The exact same observation encoding (`rl/obs_encoding.py`) is shared between offline training (`AllocationEnv`) and the ROS2 inference node (`rl_task_allocator.py`) — if they ever diverge, the trained policy receives garbage in production. `task_allocator.py` is the `nearest-free` greedy baseline.
3. **EKF sensor-fusion era** (`ekf_fusion_node.py`, `association.py`, `projection.py`) — the LAFusion paper. Replaces the RL era's implicit odometry/YOLO handling with an explicit per-robot EKF: odometry drives prediction, YOLO detections (gated by Mahalanobis distance in `association.py`) drive correction.

All three eras consume the same live sensor pipeline: overhead camera (`world_with_camera.world`, fixed at `(0,0,3)`, 90° downward pitch) → YOLOv8n → `yolo_detector.py`, which publishes `/robot_detections`. `projection.py` holds two independently-maintained pixel↔world projections (FOV heuristic, used in production; camera-intrinsics-based, confirmed geometrically equivalent to the heuristic — see LAFusion Methodology) — don't assume changing one changes the other's behavior.

### EKF math: correct() unified, predict() intentionally stays split

As of 2026-08-24 (`refactor/unify-ekf-core`), `correct()` and `r_from_confidence()` — bit-for-bit identical across `ekf_fusion_node.py` (ROS2 production node), `scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py` / `scripts/lafusion/2.evaluation/eval_ekf_continuous_error.py` (offline evaluation against recorded bags), and `scripts/lafusion/1.validation/validate_ekf_synthetic.py` (synthetic Monte Carlo validation) — live in one place: `src/cerise_nav/cerise_nav/ekf_core.py` (no rclpy dependency, importable standalone). All four consumers import from there; a fix to Joseph-form covariance or Mahalanobis gating now only needs to happen once. `predict()` deliberately was **not** unified and still exists separately in each file: production/offline-eval consume already-integrated absolute odometry and step via delta (`predict_from_odom`/`predict`, `F=I`), while the synthetic validator drives an explicit unicycle kinematic model from `(v, w)` with a non-trivial Jacobian (`predict(state, cov, v, w, dt, Q)`) — these are genuinely different motion models, not the same function under different names, so forcing one interface over both was rejected (see research notes in the `refactor/unify-ekf-core` branch history). `Q`/`COV_CAP` remain per-consumer parameters, never hardcoded in `ekf_core.py` — production/eval use `Q_DIAG=(0.5, 0.5, 0.25)` calibrated against real bag drift, the synthetic validator uses `q_diag=(1e-6, 1e-6, 5e-7)` calibrated for its near-ideal synthetic noise; sharing one value across both would silently miscalibrate one of them. All consumers use the numerically-stable **Joseph form** covariance update and **Mahalanobis gating with `S=P+R`** (not just `P`) — reproduces the paper's published numbers exactly (bit-for-bit): NEES≈2.955/NIS≈1.982 synthetic, +23.7%/−5.3% aggressive/light-drift gain, −12.7% ATE / −4.2% RPE continuous-error degradation.

**Covariance ceiling is empirically justified, not just simple.** The `COV_CAP=0.05` clip (`ekf_fusion_node.py`) is mathematically inelegant (breaks PSD structure) — a fading factor (AFKF, scaling `Q` instead of clipping `P`) is the textbook-correct alternative. It was implemented and tested (`scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py:COV_UPDATE_MODE`, still present as an experimental flag) and **loses empirically**: without a cap, or with one loose enough to differ from the clip, `P` grows enough that Mahalanobis gating starts accepting the wrong robot's detection — the exact failure mode the clip exists to prevent. Gain under aggressive drift dropped from +23.7% (clip) to −16.5% (fading, no cap). Don't re-litigate this without new data.

**The unify-`correct()`-duplicate-`predict()` split is validated against the software-design literature, not just internal convention (2026-08-28 research pass).** DRY (Hunt & Thomas, *The Pragmatic Programmer*) is about one unambiguous representation of a single piece of *knowledge*, not textual similarity — the three `predict()` implementations encode genuinely different motion-model knowledge, so they are not DRY violations. Forcing them into one function (e.g. to also carry the eval-only experimental fading-factor branch) would reproduce the exact anti-pattern Sandi Metz calls "The Wrong Abstraction" (flag/parameter hiding a diverging case). The `ekf_core.py`-outside-ROS / `ekf_fusion_node.py`-as-ROS-wrapper split mirrors a real, current pattern in the ROS2 ecosystem (e.g. `FusionCore`'s `fusioncore_core`/`fusioncore_ros` split, arXiv:2605.25239), not an ad-hoc choice. The one real gap this surfaced — no automated check that production's and offline-eval's `predict()` still agree on their shared "clip" path — is now closed by `test_ekf_predict_regression.py` (see Build and test above); that test does not unify the functions, it's a golden-master/characterization test (Michael Feathers) confirming they still agree, which is the correct-weight fix for a "silent drift" risk, not a reason to revisit the split itself.

### Where validation actually lives

There's no single test command that proves a pipeline change is correct — validation is split across scripts that target different guarantees:
- `scripts/lafusion/1.validation/validate_ekf_synthetic.py` — synthetic ground-truth Monte Carlo (30 seeds), checks filter consistency via NEES/NIS before trusting it against real data.
- `scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py` / `eval_ekf_continuous_error.py` — EKF vs. odometry-only vs. ground truth, against the three recorded scenarios in `bags/` (stationary/straight/curve, MCAP format — requires `ros-humble-rosbag2-storage-mcap`, not installed by default on Humble).
- `scripts/lafusion/2.evaluation/stats_tests.py` — statistical toolkit (paired Wilcoxon signed-rank, Cliff's delta, bootstrap CI), currently only imported by `eval_ekf_vs_baseline.py`. Written to be paper-agnostic (no EKF-specific assumptions) in case LARS result tables adopt it later, but as of the 2026-08-25 `scripts/lafusion/` move, no LARS script actually imports it — verify with a fresh `grep` before assuming otherwise.
- `scripts/2.rl_evaluation/sweep_load.py` — LARS load-regime robustness sweep (six inter-arrival rates, 500 episodes/point).
- `scripts/lafusion/2.evaluation/eval_ekf_continuous_error.py` reports both **ATE** (absolute per-sample error, no rigid alignment needed since estimate/odometry/ground-truth already share the world frame via the fixed camera) and **RPE** (Sturm et al. 2012 convention — error of the position delta between consecutive readings, isolates local step-to-step drift from ATE's accumulated error). This is the project's adopted metric convention for any future trajectory-error script — follow it rather than reporting only a bare mean/RMSE.

### Repository layout after the 2026-08-24 cleanup

The repo root used to have ~35 loose files; entry-point shell scripts now live in `launch/` (alongside the existing `*.launch.py`), Gazebo worlds/robot models in `worlds/`, YAML configs and `camera_calibration.npz` in `config/`, and superseded docs/slides/figures (no longer referenced by any active paper or script) in `docs/legacy/`. If a script you're editing references a bare filename like `params_r1.yaml` or `world_with_camera.world`, check whether it resolves the path dynamically (`$SCRIPT_DIR/../config/...` in shell, `Path(__file__).resolve().parent.parent / "config" / ...` in the `launch/*.launch.py` files) before assuming it still works after a move.

The LAFusion-only scripts live in `scripts/lafusion/`, organized by pipeline stage with a numeric prefix on each subfolder so the execution order is visible directly in a file listing (`0.setup/` → `1.validation/` → `2.evaluation/` → `3.figures/`) — `_REPO` in each resolves the repo root with as many `os.path.dirname(...)` calls as the file is nested (2 for `scripts/lafusion/*.py`, 3 for files inside a numbered subfolder):
- `scripts/lafusion/0.setup/calibrate_camera.py` — stage 0 (prerequisite, run once before any of the above): camera intrinsics calibration.
- `scripts/lafusion/1.validation/validate_ekf_synthetic.py` — stage 1: synthetic Monte Carlo consistency check (NEES/NIS), no bags involved.
- `scripts/lafusion/2.evaluation/` — stage 2: `eval_ekf_vs_baseline.py`, `eval_ekf_continuous_error.py` run the filter against recorded bags and report gain/ATE/RPE; `stats_tests.py` is the shared statistical toolkit they both import (kept alongside them since, as of the 2026-08-25 move, no LARS script imports it — verify with `grep` before assuming otherwise).
- `scripts/lafusion/3.figures/` — stage 3: everything that turns stage-1/2 output into paper figures (`plot_ekf_results.py`, `plot_nees_nis.py`, `plot_yolo_odom_ekf_comparison.py`, `plot_architecture_diagram.py`, `render_terminal_screenshot.py`). Several of these import directly from `2.evaluation/` or `1.validation/` by bare module name (e.g. `from eval_ekf_vs_baseline import RobotEKF`), which only resolves because each file's `sys.path.insert` explicitly adds the sibling stage folder — if you add a new figure script that needs `RobotEKF`/`read_bag`/etc., add `sys.path.insert(0, os.path.join(_REPO, 'scripts', 'lafusion', '2.evaluation'))` rather than assuming the plain import will work.

`render_terminal_screenshot.py` is shared with LARS (its `--preset validation` renders the LARS validation output) but was moved into `scripts/lafusion/3.figures/` anyway at the user's request on 2026-08-25 — its invocation path is `scripts/lafusion/3.figures/render_terminal_screenshot.py`, not `scripts/render_terminal_screenshot.py`. Folder names with a numeric prefix (`0.setup`, `1.validation`, ...) aren't valid Python package names, but that's fine — nothing imports these as packages; each script is invoked directly (`python3 scripts/lafusion/1.validation/validate_ekf_synthetic.py`) and cross-stage imports go through explicit `sys.path.insert`, not `from scripts.lafusion.evaluation import ...`.

The LARS-only scripts (rest of `scripts/`, outside `scripts/lafusion/`) are organized by track rather than a single linear pipeline, because LARS genuinely has four parallel tracks instead of one sequential flow — numbering all of them would imply an execution order that doesn't exist:
- `scripts/1.rl_training/train_ppo.py` → `scripts/2.rl_evaluation/` (`eval_policy.py`, `sweep_load.py`, `aggregate_multiseed.py`, `analyze_ppo_behavior.py`) → `scripts/3.figures/` (`animate_allocation.py`, `plot_benchmark.py`, `plot_detection_error.py`, `plot_learning_curve.py`, `gen_mdp_diagram.py`, `gen_slides_gif.py`) — these three *are* numbered, because train→evaluate→plot is a real sequence.
- `scripts/yolo_dataset/` (`collect_teleport.py`, `split_dataset.py`, `visualize_dataset.py`, `visualize_bboxes.py`, `verify_bboxes_gui.py`, `train_yolo.sh`) — the original YOLO-dataset-era pipeline (see Architecture above), largely frozen. Unnumbered: it doesn't feed into the RL track's stages.
- `scripts/calibration_debug/` (`debug_camera_validation.py`, `debug_projection.py`, `test_frame_reference_modes.py`, `test_odom_offset.py`, `validate_camera.sh`, `analyze_odom.py`, `benchmark_detector.py`, `cmd_vel_random.py`, `random_nav_goals.py`) — ad-hoc tools invoked on demand, not in any fixed sequence. Unnumbered for the same reason.
- `scripts/prepare_validation.sh` and `scripts/run_digital_twin_demo.sh` stay directly under `scripts/` — they orchestrate the whole pipeline (multiple tracks), not a single stage within one.

Same `_REPO` convention as `scripts/lafusion/`: two `os.path.dirname(...)` calls for `scripts/*.sh` at the top level, three for anything inside one of the subfolders above.

### Reproducibility package

`bags/reproducibility_package/` pins a specific git SHA, hyperparameters (`params.yaml`), camera calibration, and world/URDF files needed to rerun the LAFusion results independently. `docs/lafusion/` mirrors (copies, not symlinks) the live scripts/code/figures for paper packaging — its own README says explicitly to edit the originals in `scripts/`/`src/`, not the copies. As of 2026-08-25, resync is automated: run `python3 scripts/lafusion/build_package.py` after editing any of `association.py`/`ekf_core.py`/`ekf_fusion_node.py`/`projection.py`/`yolo_detector.py`, any `scripts/lafusion/*/` script, or `bags/reproducibility_package/README.md` — it diffs source against the copy and only touches files that actually changed. Manual `cp` is no longer the expected workflow; the earlier manual-resync process was a repeated source of drift (forgotten "for later in the session") that this script replaces.

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
- Bring up the multi-robot pipeline: `./launch/launch_3robots_with_camera.sh`, wait **70s+** before running any other node (Nav2 takes a while to stabilize). Full production run (one terminal each, `scripts/prepare_validation.sh {baseline|ppo}` prints this same sequence and kills stale processes first): (1) `launch/launch_3robots_with_camera.sh` → (2) wait 70s+ → (3) `ros2 run cerise_nav demand_generator` → (4) `ros2 run cerise_nav task_allocator` (nearest-free baseline) or `ros2 run cerise_nav rl_task_allocator --ros-args -p model_path:=$PWD/models/ppo_allocator_yolo.zip` (trained PPO) → (5) `ros2 run cerise_nav yolo_detector`.
- Stuck Gazebo/Nav2 zombie nodes: `pkill -9 component_container` before relaunching — restarting without this leaves DDS ports held.
- If a ROS2 node keeps logging but no CLI subscriber receives anything (even with compatible QoS), that process's DDS state is stuck — kill and relaunch it, no need to restart the whole environment. **Caveat**: confirming `ros2 topic hz` looks healthy right after a relaunch does not guarantee publication stays stable through a subsequent `ros2 bag record` — always check the actual message count in the recorded bag, don't trust a prior `hz` check alone.

## Git

- **Never include `Co-Authored-By: Claude`** in commits — explicit user preference.
- Always check `git status` before a broad `git add` — the repo has repeatedly accumulated loose files from old sessions that shouldn't be committed without confirming what they are first (e.g. `docs/RSL/`, 34MB of third-party literature-review PDFs, was found committed and untracked in a 2026-08-24 cleanup).

## Environment (Wayland/GNOME + VSCode snap)

- Legacy X11 screenshot tools (`scrot`, `import`, `gnome-screenshot`) return a black screen under Wayland — there's no fully automated screenshot capture in this environment.
- Env vars injected by the VSCode snap (`GTK_PATH`, `GDK_PIXBUF_MODULEDIR`, etc.) break native graphical binaries (RViz2 and similar) with GLIBC symbol errors — `unset` them before running system graphical binaries.
- RViz2 has no programmatic screenshot API on Humble (no ROS2 service, no Python binding) — don't attempt that route for paper figures; see the `paper-figure-style` skill for the matplotlib-over-bag-data alternative instead.
