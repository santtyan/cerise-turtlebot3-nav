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

    traj_ekf, traj_odom, traj_gt, cov_ekf = [], [], [], []
    err_ekf_series, err_odom_series, err_t = [], [], []
    # Série contínua (amostrada a cada leitura de odometria, não só nos
    # instantes de correção YOLO) — necessária para ver a covariância
    # crescer/saturar de fato entre correções, não só o "antes/depois".
    cov_trace_t, cov_trace_val, correction_t = [], [], []
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
                cov_ekf.append(filters[r].cov.copy())
                err_ekf_series.append(math.hypot(filters[r].state[0] - gt[r][0],
                                                  filters[r].state[1] - gt[r][1]))
                err_odom_series.append(math.hypot(odom_drifted[r][0] - gt[r][0],
                                                   odom_drifted[r][1] - gt[r][1]))
                t_rel = (t - t0) / 1e9 if t0 else 0.0
                err_t.append(t_rel)
                if ROBOT_TO_PLOT in assignments:
                    correction_t.append(t_rel)
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

        if robot_id == ROBOT_TO_PLOT:
            cov_trace_t.append((t - t0) / 1e9 if t0 else 0.0)
            cov_trace_val.append(np.trace(filters[robot_id].cov[:2, :2]))

    return (np.array(traj_ekf), np.array(traj_odom), np.array(traj_gt),
            np.array(err_ekf_series), np.array(err_odom_series), np.array(err_t),
            cov_ekf, np.array(cov_trace_t), np.array(cov_trace_val), np.array(correction_t))


def plot_trajectory(bag_path, out_path, smooth_window=21):
    """Trajetória como linha contínua conectada (padrão confirmado em
    papers reais de EKF/fusão sensorial — Housein et al. 2022, Hoang et al.
    arXiv:1611.07112, Cioffi & Scaramuzza IROS 2020: trajetória é sempre
    linha, nunca nuvem de pontos dispersos como em versões anteriores desta
    figura). Usa um cenário com movimento real (cenario2_reto, não
    cenario1_parado — sem deslocamento real não há trajetória para uma
    linha representar). Aplica suavização leve (média móvel) antes de
    plotar — sem isso, o ruído ponto-a-ponto da odometria/EKF vira
    "espaguete" ilegível quando conectado diretamente."""
    rng = np.random.default_rng(42)
    traj_ekf, traj_odom, traj_gt, _, _, _, _, _, _, _ = run_scenario_with_trajectory(
        bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

    def smooth_xy(traj, w):
        if len(traj) < w:
            return traj
        kernel = np.ones(w) / w
        x = np.convolve(traj[:, 0], kernel, mode='valid')
        y = np.convolve(traj[:, 1], kernel, mode='valid')
        return np.column_stack([x, y])

    gt_s = smooth_xy(traj_gt, smooth_window)
    odom_s = smooth_xy(traj_odom, smooth_window)
    ekf_s = smooth_xy(traj_ekf, smooth_window)

    # Figura mais larga que alta: a trajetória real é ~4x mais extensa em x
    # que em y (movimento ~reto), então figsize quadrado (usado nas demais
    # figuras) deixa a legenda abaixo do eixo colidir com o próprio plot.
    fig, ax = plt.subplots(figsize=(5.0, 2.6))

    ax.plot(gt_s[:, 0], gt_s[:, 1], color=COLOR_GT, linewidth=2.0,
             label='Ground truth', zorder=4)
    ax.plot(odom_s[:, 0], odom_s[:, 1], color=COLOR_ODOM, linewidth=1.0,
             linestyle='--', alpha=0.85, label='Odometry only (drift)', zorder=1)
    ax.plot(ekf_s[:, 0], ekf_s[:, 1], color=COLOR_EKF, linewidth=1.2,
             label='EKF (fusion)', zorder=2)

    ax.scatter(*gt_s[0], marker='o', s=25, c=COLOR_START, zorder=5, label='Start')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), ncol=1,
              markerscale=1.2, handletextpad=0.6, labelspacing=0.4,
              fontsize=7)

    fig.tight_layout(pad=0.3)
    plt.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def plot_trajectory_dispersion_ellipse(bag_path, out_path):
    """Variante C: mesmo cenário parado (cenario1_parado) da figura
    original, mas troca a nuvem de pontos crus por uma elipse de dispersão
    empírica (1 desvio-padrão, a partir da matriz de covariância amostral
    dos próprios pontos — não a covariância do filtro) por série, mais o
    centroide. Sem trajetória real para desenhar como linha (robô parado),
    isso comunica "quão disperso" cada fonte é ao redor do ground truth sem
    depender de nuvem de pontos ilegível."""
    from matplotlib.patches import Ellipse

    rng = np.random.default_rng(42)
    traj_ekf, traj_odom, traj_gt, _, _, _, _, _, _, _ = run_scenario_with_trajectory(
        bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

    fig, ax = plt.subplots(figsize=(3.5, 3.1))

    ax.scatter(traj_gt[:, 0], traj_gt[:, 1], s=90, marker='+', c=COLOR_GT,
               linewidths=1.4, label='Ground truth', zorder=4)

    for traj, color, name in [(traj_odom, COLOR_ODOM, 'Odometry only (drift)'),
                               (traj_ekf, COLOR_EKF, 'EKF (fusion)')]:
        centroid = traj.mean(axis=0)
        cov_xy = np.cov(traj[:, 0], traj[:, 1])
        eigvals, eigvecs = np.linalg.eigh(cov_xy)
        eigvals = np.clip(eigvals, 0, None)
        width, height = 2 * np.sqrt(eigvals)  # 1 sigma
        angle = np.degrees(np.arctan2(eigvecs[1, -1], eigvecs[0, -1]))
        ellipse = Ellipse(centroid, width, height, angle=angle,
                           facecolor=color, edgecolor=color, alpha=0.18,
                           linewidth=1.2, zorder=2 if color == COLOR_EKF else 1)
        ax.add_patch(ellipse)
        ax.scatter(*centroid, s=25, c=color, edgecolors='black', linewidths=0.4,
                   zorder=3, label=f'{name} (centroid ± 1σ)')

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=1,
              markerscale=1.2, handletextpad=0.6, labelspacing=0.4,
              fontsize=7)

    fig.tight_layout(pad=0.3)
    plt.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {out_path}')


def plot_covariance_trace(bags_dir, out_path, cov_cap=0.05):
    """trace(P_xy) vs. tempo, em step plot, para os 3 cenários — mostra que
    a covariância do EKF satura no teto de design (COV_CAP) durante os
    intervalos sem correção YOLO em vez de crescer suavemente, um regime
    previsto teoricamente por Sinopoli et al. 2004 ("Kalman Filtering with
    Intermittent Observations", IEEE TAC) para taxas de observação abaixo de
    um limiar crítico. O teto é desenhado como linha horizontal explícita
    para deixar claro que é um limite de engenharia, não um valor de
    convergência do filtro — e os instantes de correção são marcados para
    mostrar a queda abrupta que os intervalos sem correção não têm.

    Nota: COV_CAP (eval_ekf_vs_baseline.py) faz np.clip elemento-a-elemento
    na matriz 3x3 inteira (posição x,y e yaw), então o teto do TRAÇO da
    submatriz de posição 2x2 é 2*COV_CAP, não COV_CAP — refletido no eixo
    aqui para a linha de referência bater com o platô real dos dados."""
    cov_cap_trace = 2 * cov_cap
    scenarios = ['cenario1_parado', 'cenario2_reto', 'cenario3_curva']
    titles = ['(a) Stationary', '(b) Straight motion', '(c) Curve / occlusion']

    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.5), sharey=True)

    for ax, scenario, title in zip(axes, scenarios, titles):
        rng = np.random.default_rng(42)
        bag_path = os.path.join(bags_dir, scenario)
        (_, _, _, _, _, _, _,
         cov_trace_t, cov_trace_val, correction_t) = run_scenario_with_trajectory(
            bag_path, inject_drift=True, rng=rng, drift_rate_divisor=20.0)

        ax.step(cov_trace_t, cov_trace_val, where='post', color=COLOR_EKF,
                linewidth=1.1, label='trace(P) — EKF', zorder=2)
        ax.axhline(cov_cap_trace, color=COLOR_ODOM, linewidth=0.8, linestyle='--',
                   label=f'2×COV_CAP = {cov_cap_trace}', zorder=1)
        # Rug plot na base do eixo em vez de axvline por evento: com até 152
        # correções numa figura de ~2in de largura, linhas verticais cheias
        # se sobrepõem em um bloco sólido que compete visualmente com a
        # série principal — marcadores curtos na base ficam legíveis mesmo
        # em alta densidade e ainda comunicam a frequência relativa.
        tick_y = cov_trace_val.min() - 0.1 * (cov_cap_trace - cov_trace_val.min())
        ax.plot(correction_t, [tick_y] * len(correction_t), '|', color=COLOR_START,
                markersize=4, markeredgewidth=0.6, label='YOLO correction', zorder=3)

        ax.set_xlabel('Time (s)')
        ax.set_title(title, fontsize=8.5, loc='left')

    axes[0].set_ylabel('trace(P) — position covariance')
    fig.subplots_adjust(top=0.78)
    handles, labels = axes[0].get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.0),
                      ncol=2, frameon=False)

    plt.savefig(out_path, bbox_extra_artists=(leg,), bbox_inches='tight')
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
        _, _, _, err_ekf, err_odom, err_t, _, _, _, _ = run_scenario_with_trajectory(
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

    # Figura principal de Results: trajetória em linha, cenario2_reto (tem
    # deslocamento real de ~1.5m — necessário para uma linha fazer sentido).
    plot_trajectory(
        os.path.join(bags_dir, 'cenario2_reto'),
        os.path.join(OUT_DIR, 'lafusion_trajectory.png'))

    # Figura complementar (Methodology/Discussion): cenario1_parado tem o
    # ganho mais forte isolado (+46.3% vs. +23.7% agregado — ver
    # eval_ekf_vs_baseline.py), mas sem deslocamento real; elipse de
    # dispersão comunica compacidade sem precisar de nuvem de pontos crua.
    plot_trajectory_dispersion_ellipse(
        os.path.join(bags_dir, 'cenario1_parado'),
        os.path.join(OUT_DIR, 'lafusion_trajectory_dispersion.png'))

    plot_covariance_trace(bags_dir, os.path.join(OUT_DIR, 'lafusion_covariance_trace.png'))

    plot_error_over_time(bags_dir, os.path.join(OUT_DIR, 'lafusion_error_over_time.png'))


if __name__ == '__main__':
    main()
