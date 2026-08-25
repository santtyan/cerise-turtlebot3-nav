---
name: paper-figure-style
description: Generate matplotlib figures following the gold-standard visual convention for robotics/sensor-fusion papers (EKF, SLAM, tracking, multi-robot localization). Use whenever creating or revising a figure for a paper in this repo (docs/lafusion_paper/, docs/paper_lars/). Triggers on requests like "make a trajectory figure", "plot the error over time", "generate a figure for the paper", or when a generated figure "doesn't look scientific/professional".
---

# Paper Figure Style

## Why this exists

Confirmed by reading 4 real published papers in this exact subfield
(Housein et al. 2022 IJACSA; Hoang et al. arXiv:1611.07112; Zhao &
Whittaker WM2019/arXiv:2002.12408; Cioffi & Scaramuzza IROS 2020) and by
trial-and-error in this repo's own figure history (an early
`lafusion_trajectory.png` used `scatter()` and looked like a debug dump,
not a paper figure — replaced after this research).

## The confirmed gold standard

**Trajectory plots are always a continuous connected line, never a
scatter/point cloud.** Scatter is reserved for: raw noisy sensor
readings shown as background context, or static landmarks. If your
scenario has real displacement, plot the trajectory with `ax.plot()`,
not `ax.scatter()`. Ground truth gets the thickest/most distinct line
(black, or a standout color like magenta in some papers). If raw
per-point noise makes a directly-connected line illegible ("spaghetti"),
apply a light moving-average smoothing before plotting — do not fall
back to scatter as the fix.

**When there is no real displacement to plot** (a stationary scenario),
do not force a trajectory line. Use an empirical dispersion ellipse (1σ,
from the sample covariance of the estimates around their centroid) plus
a centroid marker instead — this was the resolution found in this repo
after a scatter-cloud figure was judged illegible.

**Covariance/uncertainty over time**: if the filter's covariance
saturates at a ceiling (common under sparse observation — see
`verify-claim-against-code` and the LAFusion continuous-error finding),
plot `trace(P)` vs. time as a **step plot**, not a smoothed curve — the
staircase pattern is the real signal, not noise to hide. Mark the
ceiling value as an explicit horizontal dashed line, and mark correction
instants as a rug plot (short tick marks) rather than full vertical
`axvline`s, which visually flood the plot when corrections are frequent.

**Never use RViz2 screenshots as a paper figure.** Confirmed both by
literature convention (never seen in the 4 papers reviewed) and by this
project's own environment: RViz2 has no programmatic screenshot API in
ROS2 Humble, and screen-capture tools fail under this project's
Wayland/GNOME setup anyway. Generate the equivalent visualization from
the same topic/bag data in matplotlib instead.

## Visual conventions (already implemented in `scripts/lafusion/3.figures/plot_ekf_results.py`)

- Colorblind-safe palette (Okabe-Ito subset): black `#000000` for ground
  truth, blue `#0072B2` for the primary/proposed method, orange-red
  `#D55E00` for baseline, green `#009E73` for start/event markers.
- Serif font family (matches LNCS/CCIS body text), base size 8-9pt.
- Ticks pointing inward, top/right spines removed, light dotted grid.
- `savefig.dpi=400`.
- Legend placed outside the data area (below-center or right of axes),
  never overlapping the plotted data.
- `ax.set_aspect('equal')` for spatial (x/y) plots.

## Where the reusable plotting code lives

`scripts/lafusion/3.figures/plot_ekf_results.py` — `run_scenario_with_trajectory()` already
returns trajectory, per-instant covariance, and a continuous covariance
trace with correction timestamps. Reuse this rather than writing a new
bag-parsing loop for a new figure.
