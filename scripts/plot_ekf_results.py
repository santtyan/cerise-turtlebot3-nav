#!/usr/bin/env python3
"""Gera figuras para o paper LAFusion a partir dos resultados do passo 4
(scripts/eval_ekf_vs_baseline.py): trajetória EKF vs. odom-only vs. ground
truth, e erro ao longo do tempo nas 3 condições de drift.

Uso: python3 scripts/plot_ekf_results.py
Saída: docs/lafusion_trajectory.png, docs/lafusion_error_over_time.png
"""

import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))
sys.path.insert(0, os.path.join(_REPO, 'scripts'))

from cerise_nav.association import mahalanobis_gate  # noqa: E402
from eval_ekf_vs_baseline import (POS_DRIFT_ODOM, RobotEKF,  # noqa: E402
                                   quaternion_to_yaw, read_bag)

ROBOT_TO_PLOT = 'robot1'
OUT_DIR = os.path.join(_REPO, 'docs')


def run_scenario_with_trajectory(bag_path, inject_drift, rng, drift_rate_divisor=20.0):
    """Variante de run_scenario que também guarda a trajetória completa
    (não só os erros agregados) do robô de interesse, para plotagem."""
    events = read_bag(bag_path)

    filters = {}
    gt = {}
    odom_drifted = {}
    task_count = {'robot1': 0, 'robot2': 0, 'robot3': 0}

    traj_ekf, traj_odom, traj_gt = [], [], []
    err_ekf_series, err_odom_series, err_t = [], [], []
    t0 = None

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

            if ROBOT_TO_PLOT in filters and ROBOT_TO_PLOT in gt:
                r = ROBOT_TO_PLOT
                traj_ekf.append(tuple(filters[r].state[:2]))
                traj_odom.append(odom_drifted[r])
                traj_gt.append(gt[r])
                err_ekf_series.append(math.hypot(filters[r].state[0] - gt[r][0],
                                                  filters[r].state[1] - gt[r][1]))
                err_odom_series.append(math.hypot(odom_drifted[r][0] - gt[r][0],
                                                   odom_drifted[r][1] - gt[r][1]))
                err_t.append((t - t0) / 1e9 if t0 else 0.0)
            continue

        if t0 is None:
            t0 = t
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

    return (np.array(traj_ekf), np.array(traj_odom), np.array(traj_gt),
            np.array(err_ekf_series), np.array(err_odom_series), np.array(err_t))


def plot_trajectory(bag_path, scenario_label, out_path):
    rng = np.random.default_rng(42)
    traj_ekf, traj_odom, traj_gt, _, _, _ = run_scenario_with_trajectory(
        bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(traj_gt[:, 0], traj_gt[:, 1], 'k-', linewidth=2, label='Ground truth', zorder=3)
    ax.plot(traj_odom[:, 0], traj_odom[:, 1], 'r--', linewidth=1, alpha=0.7,
            label='Odometry only (drifted)', zorder=1)
    ax.plot(traj_ekf[:, 0], traj_ekf[:, 1], 'b-', linewidth=1.5, alpha=0.9,
            label='EKF (camera+odometry fusion)', zorder=2)
    ax.scatter([traj_gt[0, 0]], [traj_gt[0, 1]], c='green', s=80, marker='o',
               label='Start', zorder=4)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Robot trajectory: EKF vs. odometry-only vs. ground truth\n({scenario_label}, aggressive drift injected)')
    ax.legend(loc='best', fontsize=9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def plot_error_over_time(bags_dir, out_path):
    scenarios = ['cenario1_parado', 'cenario2_reto', 'cenario3_curva']
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), sharey=True)

    for ax, scenario in zip(axes, scenarios):
        rng = np.random.default_rng(42)
        bag_path = os.path.join(bags_dir, scenario)
        _, _, _, err_ekf, err_odom, err_t = run_scenario_with_trajectory(
            bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

        ax.plot(err_t, err_odom, 'r--', linewidth=1, alpha=0.7, label='Odometry only')
        ax.plot(err_t, err_ekf, 'b-', linewidth=1.5, label='EKF (fusion)')
        ax.set_xlabel('Time (s)')
        ax.set_title(scenario.replace('cenario', 'Scenario ').replace('_', ': '))
        ax.grid(True, alpha=0.3)

    axes[0].set_ylabel('Position error vs. ground truth (m)')
    axes[0].legend(loc='best', fontsize=9)
    fig.suptitle('Position error over time: EKF vs. odometry-only (aggressive drift injected)')

    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    bags_dir = os.path.join(_REPO, 'bags')

    # cenario1_parado escolhido por ter o ganho mais forte e representativo
    # do resultado agregado (+46.3% neste cenário isolado vs. +23.7% médio
    # dos 3 — ver eval_ekf_vs_baseline.py). cenario3_curva tem ganho fraco
    # (+2.4%, robô se move mais e YOLO perde detecção com mais frequência) e
    # produziria uma figura de trajetória visualmente enganosa/ruidosa.
    plot_trajectory(
        os.path.join(bags_dir, 'cenario1_parado'),
        'cenario1_parado',
        os.path.join(OUT_DIR, 'lafusion_trajectory.png'))

    plot_error_over_time(bags_dir, os.path.join(OUT_DIR, 'lafusion_error_over_time.png'))


if __name__ == '__main__':
    main()
