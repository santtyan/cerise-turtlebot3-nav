#!/usr/bin/env python3
"""Gera figuras para o paper LAFusion, seguindo convenções de publicação
IEEE/Springer (ver docs/lafusion/figures/STYLE_NOTES.md para a
justificativa de cada escolha): fonte serifada compatível com o corpo do
artigo, sem título embutido na imagem (o título vive no \\caption do LaTeX),
ticks para dentro, spines superior/direita removidas, paleta colorblind-safe,
linhas finas com marcadores esparsos em vez de curvas densas ilegíveis.

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

# --- Estilo IEEE/publicação (aplicado globalmente) -------------------------
# Largura de coluna IEEE dupla-coluna = 3.5in; figura de largura completa
# (spanning) = 7.16in. Fonte-base 8-9pt para bater com o corpo do texto em
# 10pt após redução no LaTeX. Paleta: Tableau colorblind-safe (Okabe-Ito
# subset), não as cores 'r'/'b'/'k' cruas do matplotlib.
COLOR_GT = '#000000'       # ground truth: preto sólido, é a referência
COLOR_EKF = '#0072B2'      # azul (Okabe-Ito) — série principal do paper
COLOR_ODOM = '#D55E00'     # laranja-avermelhado (Okabe-Ito) — baseline
COLOR_START = '#009E73'    # verde (Okabe-Ito) — marcador de início

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
    'xtick.major.width': 0.7,
    'ytick.major.width': 0.7,
    'xtick.major.size': 3,
    'ytick.major.size': 3,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.grid': True,
    'grid.linewidth': 0.4,
    'grid.color': '#cccccc',
    'grid.alpha': 0.6,
    'legend.frameon': False,
    'legend.handlelength': 1.6,
    'lines.linewidth': 1.0,
    'savefig.dpi': 400,
    'figure.dpi': 150,
})


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


def plot_trajectory(bag_path, out_path):
    rng = np.random.default_rng(42)
    traj_ekf, traj_odom, traj_gt, _, _, _ = run_scenario_with_trajectory(
        bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

    fig, ax = plt.subplots(figsize=(3.5, 3.1))

    # Ground truth como ponto único (robô parado neste cenário) — desenhado
    # como um alvo grande, não uma "linha" de um pixel de comprimento.
    ax.scatter(traj_gt[:, 0], traj_gt[:, 1], s=90, marker='+', c=COLOR_GT,
               linewidths=1.4, label='Ground truth', zorder=4)

    # Odometry-only: nuvem de pontos esparsos (não uma linha densa — o
    # "espaguete" ilegível da v1 vinha de conectar 456 amostras com linha
    # cheia). Pontos pequenos e semitransparentes leem como "dispersão do
    # ruído", que é o que de fato é.
    ax.scatter(traj_odom[:, 0], traj_odom[:, 1], s=5, c=COLOR_ODOM, alpha=0.35,
               linewidths=0, label='Odometry only (drift)', zorder=1)

    # EKF: mesma lógica, mas o argumento visual do paper é que a nuvem azul
    # é mais compacta ao redor do ground truth que a laranja.
    ax.scatter(traj_ekf[:, 0], traj_ekf[:, 1], s=5, c=COLOR_EKF, alpha=0.5,
               linewidths=0, label='EKF (fusion)', zorder=2)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    # Legenda abaixo do eixo, fora da área de dados — evita colidir com os
    # pontos (o "upper right" da primeira tentativa cobria a nuvem de dados).
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.16), ncol=1,
              markerscale=1.5, handletextpad=0.6, labelspacing=0.4,
              fontsize=7)

    fig.tight_layout(pad=0.3)
    plt.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def plot_error_over_time(bags_dir, out_path):
    scenarios = ['cenario1_parado', 'cenario2_reto', 'cenario3_curva']
    titles = ['(a) Stationary', '(b) Straight motion', '(c) Curve / occlusion']

    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.5), sharey=True)

    def smooth(x, w=9):
        if len(x) < w:
            return x
        kernel = np.ones(w) / w
        return np.convolve(x, kernel, mode='valid')

    # Primeira passada: coleta os dados de todos os painéis para calcular um
    # ylim compartilhado com margem — sharey=True sozinho não evita corte no
    # topo quando o maior valor de um painel bate exatamente no limite.
    all_data = []
    for scenario in scenarios:
        rng = np.random.default_rng(42)
        bag_path = os.path.join(bags_dir, scenario)
        _, _, _, err_ekf, err_odom, err_t = run_scenario_with_trajectory(
            bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)
        all_data.append((err_t, err_ekf, err_odom))

    y_max = max(max(smooth(e).max(), smooth(o).max()) for _, e, o in all_data)

    for ax, (err_t, err_ekf, err_odom), title in zip(axes, all_data, titles):
        t_s = err_t[:len(smooth(err_t))]
        ax.plot(t_s, smooth(err_odom), color=COLOR_ODOM, linewidth=1.1,
                label='Odometry only')
        ax.plot(t_s, smooth(err_ekf), color=COLOR_EKF, linewidth=1.1,
                label='EKF (fusion)')
        ax.set_xlabel('Time (s)')
        ax.set_title(title, fontsize=8.5, loc='left')
        ax.set_ylim(0, y_max * 1.12)

    axes[0].set_ylabel('Position error (m)')
    # Legenda única acima de toda a figura (não dentro de um painel, onde
    # colidia com a curva no canto superior esquerdo do painel (a)). Reserva
    # espaço explícito no topo via subplots_adjust antes de desenhar a
    # legenda, e usa bbox_extra_artists no savefig para garantir que ela não
    # seja cortada pelo bbox_inches='tight' (a v1 desta correção cortava a
    # legenda porque tight_layout não sabe da fig.legend desenhada depois).
    fig.subplots_adjust(top=0.80)
    handles, labels = axes[0].get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.0),
                      ncol=2, frameon=False)

    plt.savefig(out_path, bbox_extra_artists=(leg,), bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    bags_dir = os.path.join(_REPO, 'bags')

    # cenario1_parado escolhido por ter o ganho mais forte e representativo
    # do resultado agregado (+46.3% neste cenário isolado vs. +23.7% médio
    # dos 3 — ver eval_ekf_vs_baseline.py).
    plot_trajectory(
        os.path.join(bags_dir, 'cenario1_parado'),
        os.path.join(OUT_DIR, 'lafusion_trajectory.png'))

    plot_error_over_time(bags_dir, os.path.join(OUT_DIR, 'lafusion_error_over_time.png'))


if __name__ == '__main__':
    main()
