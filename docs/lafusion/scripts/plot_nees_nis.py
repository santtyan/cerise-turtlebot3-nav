#!/usr/bin/env python3
"""Gera figura da validação sintética do EKF (etapa 1.5): séries de NEES e
NIS ao longo do tempo, com a banda de confiança de 95% (teste chi-quadrado)
e o valor esperado marcados — evidência visual de consistência estatística
complementar aos números impressos por validate_ekf_synthetic.py.

Mesmo padrão de estilo das outras figuras do paper (fonte serifada, paleta
Okabe-Ito, ver plot_ekf_results.py).

Uso: python3 scripts/plot_nees_nis.py
Saída: docs/lafusion_nees_nis.png
"""

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'scripts'))

from validate_ekf_synthetic import generate_synthetic_trajectory, run_filter  # noqa: E402

OUT_PATH = os.path.join(_REPO, 'docs', 'lafusion_nees_nis.png')

COLOR_NEES = '#0072B2'
COLOR_NIS = '#D55E00'
COLOR_BAND = '#999999'

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'Nimbus Roman'],
    'font.size': 9,
    'axes.titlesize': 8.5,
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
    'lines.linewidth': 0.8,
})


def moving_average(x, w=20):
    if len(x) < w:
        return x
    kernel = np.ones(w) / w
    return np.convolve(x, kernel, mode='valid')


def main():
    steps, dt, seed = 2000, 0.1, 42
    true_states, v_meas, w_meas, yolo_positions, conf_meas = \
        generate_synthetic_trajectory(steps, dt, seed)
    _, nees, nis = run_filter(true_states, v_meas, w_meas, yolo_positions, conf_meas, dt)

    warmup = 50
    nees, nis = nees[warmup:], nis[warmup:]
    t = np.arange(len(nees)) * dt

    # Bandas de confiança de 95% por-amostra (não a média agregada — aqui
    # queremos ilustrar a variação amostra-a-amostra, então usamos os
    # quantis do qui-quadrado com 1 grau de liberdade efetivo por amostra,
    # não df=state_dim*n como no teste da média agregada em
    # validate_ekf_synthetic.py::consistency_report).
    state_dim, meas_dim = 3, 2
    nees_lo, nees_hi = stats.chi2.ppf([0.025, 0.975], df=state_dim)
    nis_lo, nis_hi = stats.chi2.ppf([0.025, 0.975], df=meas_dim)

    fig, axes = plt.subplots(1, 2, figsize=(7.16, 2.6))

    for ax, series, color, label, exp, lo, hi, title in [
        (axes[0], nees, COLOR_NEES, 'NEES', state_dim, nees_lo, nees_hi,
         '(a) NEES (state dim. = 3)'),
        (axes[1], nis, COLOR_NIS, 'NIS', meas_dim, nis_lo, nis_hi,
         '(b) NIS (measurement dim. = 2)'),
    ]:
        smoothed = moving_average(series)
        t_s = t[:len(smoothed)]

        ax.axhspan(lo, hi, color=COLOR_BAND, alpha=0.18, linewidth=0,
                   label='95% CI (per-sample $\\chi^2$)')
        ax.axhline(exp, color='black', linewidth=0.8, linestyle=':',
                   label=f'Expected ({exp})')
        ax.plot(t_s, smoothed, color=color, linewidth=1.0,
                label=f'{label} (20-sample moving avg.)')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel(label)
        ax.set_title(title, loc='left')
        ax.set_ylim(0, max(hi, smoothed.max()) * 1.15)

    axes[1].legend(loc='upper right', fontsize=6.5)

    fig.tight_layout(pad=0.3, w_pad=1.5)
    plt.savefig(OUT_PATH, dpi=400, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {OUT_PATH}')


if __name__ == '__main__':
    main()
