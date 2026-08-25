#!/usr/bin/env python3
"""Figura complementar para o paper LAFusion: compara as 3 fontes de posição
(YOLO cru, odometria com drift) contra a saída do EKF e o ground truth, no
mesmo plano x-y. Mostra visualmente por que a fusão supera qualquer uma das
duas fontes isoladas — YOLO é ruidoso mas sem drift, odometria tem drift mas
é suave, EKF combina os dois pontos fortes.

Mesmo padrão de estilo das outras figuras (fonte serifada, paleta Okabe-Ito,
ver plot_ekf_results.py).

Uso: python3 scripts/plot_yolo_odom_ekf_comparison.py
Saída: docs/lafusion_yolo_odom_ekf.png
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))
sys.path.insert(0, os.path.join(_REPO, 'scripts', 'lafusion', '2.evaluation'))

from cerise_nav.association import mahalanobis_gate  # noqa: E402
from eval_ekf_vs_baseline import (POS_DRIFT_ODOM, RobotEKF,  # noqa: E402
                                   quaternion_to_yaw, read_bag)

ROBOT_TO_PLOT = os.environ.get('LAFUSION_ROBOT', 'robot1')
OUT_PATH = os.path.join(_REPO, 'docs', 'lafusion_yolo_odom_ekf.png')

COLOR_GT = '#000000'
COLOR_EKF = '#0072B2'
COLOR_ODOM = '#D55E00'
COLOR_YOLO = '#009E73'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'Nimbus Roman'],
    'font.size': 9,
    'axes.titlesize': 9,
    'axes.labelsize': 9,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 7.5,
    'axes.linewidth': 0.7,
    'xtick.direction': 'in',
    'ytick.direction': 'in',
    'xtick.major.size': 3,
    'ytick.major.size': 3,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.linewidth': 0.4,
    'grid.color': '#cccccc',
    'grid.alpha': 0.6,
    'legend.frameon': False,
    'lines.linewidth': 1.0,
    'savefig.dpi': 400,
    'figure.dpi': 150,
})


def collect_sources(bag_path, inject_drift, rng, drift_rate_divisor=20.0):
    """Guarda duas séries separadas para o robô de interesse:
    - full coverage (a cada leitura de odometria): odom/EKF/GT — sempre
      disponíveis, é o que o robô usaria em produção o tempo todo.
    - YOLO coverage (só quando o gating associa uma detecção): YOLO/GT nesses
      instantes específicos — é um subconjunto raro quando há oclusão.
    Comparar RMSE dessas duas séries diretamente seria injusto (populações de
    amostras diferentes) — por isso ambas voltam separadas, com a fração de
    cobertura de cada uma."""
    events = read_bag(bag_path)

    filters = {}
    gt = {}
    odom_drifted = {}
    task_count = {'robot1': 0, 'robot2': 0, 'robot3': 0}

    traj_yolo, gt_at_yolo = [], []
    traj_odom, traj_ekf, traj_gt = [], [], []

    for t, topic, msg in events:
        if topic == '/robot_detections':
            detections = [(p.position.x, p.position.y) for p in msg.poses]
            det_conf = {(p.position.x, p.position.y): p.position.z for p in msg.poses}
            if not odom_drifted or not filters:
                continue
            cov_by_robot = {r: f.cov for r, f in filters.items() if r in odom_drifted}
            assignments, _ = mahalanobis_gate(odom_drifted, cov_by_robot, detections)
            for robot_id, det_xy in assignments.items():
                conf = det_conf.get(det_xy, 0.5)
                filters[robot_id].correct(det_xy, conf)

            if ROBOT_TO_PLOT in assignments and ROBOT_TO_PLOT in gt:
                r = ROBOT_TO_PLOT
                traj_yolo.append(assignments[r])
                gt_at_yolo.append(gt[r])
            continue

        robot_id = topic.split('/')[1]
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        gt[robot_id] = (p.x, p.y)

        if inject_drift:
            task_count[robot_id] += 1
            drift_scale = POS_DRIFT_ODOM * min(task_count[robot_id] / drift_rate_divisor, 5.0)
            noisy = (p.x + rng.normal(0, drift_scale), p.y + rng.normal(0, drift_scale))
        else:
            noisy = (p.x, p.y)
        odom_drifted[robot_id] = noisy

        if robot_id not in filters:
            filters[robot_id] = RobotEKF([noisy[0], noisy[1], yaw])
        else:
            filters[robot_id].predict(noisy[0], noisy[1], yaw, t / 1e9)

        if robot_id == ROBOT_TO_PLOT:
            traj_odom.append(odom_drifted[robot_id])
            traj_ekf.append(tuple(filters[robot_id].state[:2]))
            traj_gt.append(gt[robot_id])

    return (np.array(traj_yolo), np.array(gt_at_yolo), np.array(traj_odom),
            np.array(traj_ekf), np.array(traj_gt))


def main():
    scenario = os.environ.get('LAFUSION_SCENARIO', 'cenario2_reto')
    bag_path = os.path.join(_REPO, 'bags', scenario)
    rng = np.random.default_rng(42)
    traj_yolo, gt_at_yolo, traj_odom, traj_ekf, traj_gt = collect_sources(
        bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

    coverage_pct = 100.0 * len(traj_yolo) / len(traj_odom) if len(traj_odom) else 0.0

    fig, ax = plt.subplots(figsize=(3.5, 3.1))

    ax.plot(traj_gt[:, 0], traj_gt[:, 1], color=COLOR_GT, linewidth=1.6,
            label='Ground truth', zorder=5)

    ax.scatter(traj_yolo[:, 0], traj_yolo[:, 1], s=10, c=COLOR_YOLO, alpha=0.7,
               linewidths=0, label=f'YOLO detection ({coverage_pct:.0f}% coverage)',
               zorder=4)

    ax.scatter(traj_odom[:, 0], traj_odom[:, 1], s=6, c=COLOR_ODOM, alpha=0.35,
               linewidths=0, label='Odometry only (drift)', zorder=1)

    ax.scatter(traj_ekf[:, 0], traj_ekf[:, 1], s=6, c=COLOR_EKF, alpha=0.5,
               linewidths=0, label='EKF (fusion)', zorder=2)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=1,
              markerscale=1.3, handletextpad=0.6, labelspacing=0.4,
              fontsize=7)

    def rmse(traj, ref):
        return math.sqrt(np.mean(np.sum((traj - ref) ** 2, axis=1)))

    print(f'YOLO coverage: {len(traj_yolo)}/{len(traj_odom)} samples ({coverage_pct:.1f}%)')
    print(f'RMSE YOLO (only at {len(traj_yolo)} covered instants): {rmse(traj_yolo, gt_at_yolo):.4f}m')
    print(f'RMSE odom-only (all {len(traj_odom)} instants):        {rmse(traj_odom, traj_gt):.4f}m')
    print(f'RMSE EKF fusion (all {len(traj_ekf)} instants):        {rmse(traj_ekf, traj_gt):.4f}m')

    fig.tight_layout(pad=0.3)
    plt.savefig(OUT_PATH, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {OUT_PATH}')


if __name__ == '__main__':
    main()
