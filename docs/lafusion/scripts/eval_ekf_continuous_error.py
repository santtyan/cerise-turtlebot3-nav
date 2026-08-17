#!/usr/bin/env python3
"""Erro CONTÍNUO do EKF (amostrado a cada leitura de odometria, não só nos
instantes de correção YOLO) — reproduz a Tabela 2 (Continuous Error) do
paper LAFusion, incluindo RMSE além do erro médio já publicado.

Complementa eval_ekf_vs_baseline.py (que mede só nos instantes de
correção, Tabela 1) reaproveitando a mesma classe RobotEKF/mahalanobis_gate,
mas amostrando erro a cada leitura de odometria do robô-alvo (mesma lógica
usada para descobrir o achado do erro contínuo, ver
project_lafusion_ekf_continuous_error_finding na memória do projeto).

Uso: python3 scripts/eval_ekf_continuous_error.py
"""

import math
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))
sys.path.insert(0, os.path.join(_REPO, 'scripts'))

from cerise_nav.association import mahalanobis_gate  # noqa: E402
from eval_ekf_vs_baseline import (POS_DRIFT_ODOM, ROBOTS, RobotEKF,  # noqa: E402
                                   quaternion_to_yaw, read_bag)

TARGET_ROBOT = 'robot1'


def run_continuous(bag_path, rng, drift_rate_divisor=20.0, target=TARGET_ROBOT):
    events = read_bag(bag_path)
    filters = {}
    gt = {}
    odom_drifted = {}
    task_count = {r: 0 for r in ROBOTS}
    errs_ekf, errs_odom = [], []
    yolo_hits = 0
    total_odom_readings = 0

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
                if robot_id == target:
                    yolo_hits += 1
            continue

        robot_id = topic.split('/')[1]
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        gt[robot_id] = (p.x, p.y)

        task_count[robot_id] += 1
        drift_scale = POS_DRIFT_ODOM * min(task_count[robot_id] / drift_rate_divisor, 5.0)
        noisy = (p.x + rng.normal(0, drift_scale), p.y + rng.normal(0, drift_scale))
        odom_drifted[robot_id] = noisy

        if robot_id not in filters:
            filters[robot_id] = RobotEKF([noisy[0], noisy[1], yaw])
        else:
            filters[robot_id].predict(noisy[0], noisy[1], yaw, t / 1e9)

        if robot_id == target:
            total_odom_readings += 1
            err_ekf = math.hypot(filters[robot_id].state[0] - gt[robot_id][0],
                                  filters[robot_id].state[1] - gt[robot_id][1])
            err_odom = math.hypot(odom_drifted[robot_id][0] - gt[robot_id][0],
                                   odom_drifted[robot_id][1] - gt[robot_id][1])
            errs_ekf.append(err_ekf)
            errs_odom.append(err_odom)

    return np.array(errs_ekf), np.array(errs_odom), yolo_hits, total_odom_readings


def main():
    bags_dir = os.path.join(_REPO, 'bags')
    scenarios = ['cenario1_parado', 'cenario2_reto', 'cenario3_curva']
    titles = ['Stationary', 'Straight motion', 'Curve/occlusion']

    all_ekf, all_odom = [], []
    for scenario, title in zip(scenarios, titles):
        rng = np.random.default_rng(42)
        e_ekf, e_odom, hits, total = run_continuous(os.path.join(bags_dir, scenario), rng, 20.0)
        coverage = hits / total * 100 if total else 0
        mean_ekf, mean_odom = e_ekf.mean(), e_odom.mean()
        change = (mean_odom - mean_ekf) / mean_odom * 100 if mean_odom > 0 else 0
        rmse_ekf = np.sqrt(np.mean(e_ekf ** 2))
        rmse_odom = np.sqrt(np.mean(e_odom ** 2))
        rmse_change = (rmse_odom - rmse_ekf) / rmse_odom * 100 if rmse_odom > 0 else 0
        print(f'{title}: n={total} coverage={coverage:.1f}% '
              f'mean EKF={mean_ekf:.4f}m odom={mean_odom:.4f}m change={change:.1f}% | '
              f'RMSE EKF={rmse_ekf:.4f}m odom={rmse_odom:.4f}m change={rmse_change:.1f}%')
        all_ekf.extend(e_ekf)
        all_odom.extend(e_odom)

    all_ekf = np.array(all_ekf)
    all_odom = np.array(all_odom)
    mean_ekf, mean_odom = all_ekf.mean(), all_odom.mean()
    change = (mean_odom - mean_ekf) / mean_odom * 100
    rmse_ekf = np.sqrt(np.mean(all_ekf ** 2))
    rmse_odom = np.sqrt(np.mean(all_odom ** 2))
    rmse_change = (rmse_odom - rmse_ekf) / rmse_odom * 100
    print(f'\nAggregated: n={len(all_ekf)} '
          f'mean EKF={mean_ekf:.4f}m odom={mean_odom:.4f}m change={change:.1f}% | '
          f'RMSE EKF={rmse_ekf:.4f}m odom={rmse_odom:.4f}m change={rmse_change:.1f}%')


if __name__ == '__main__':
    main()
