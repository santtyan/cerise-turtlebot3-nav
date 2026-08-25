#!/usr/bin/env python3
"""Gera diagrama de arquitetura do pipeline de fusão EKF para a seção
Methodology do paper LAFusion. Mesmo padrão visual de plot_ekf_results.py
(fonte serifada, paleta Okabe-Ito, sem título embutido — ver
docs/lafusion/figures/STYLE_NOTES.md).

Uso: python3 scripts/plot_architecture_diagram.py
Saída: docs/lafusion_architecture.png
"""

import os

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'docs', 'lafusion_architecture.png')

plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'Nimbus Roman'],
    'font.size': 8,
})

BOX_STYLE = dict(boxstyle='round,pad=0.35', linewidth=0.9)

# Mesma paleta Okabe-Ito das outras 2 figuras (consistência entre todas as
# figuras do paper), com tons claros (fills) + a cor sólida só na borda.
COLOR_SENSOR = '#CCE5F6'   # azul claro (fill), borda '#0072B2'
COLOR_PROCESS = '#FDE6D2'  # laranja claro (fill), borda '#D55E00'
COLOR_FILTER = '#D6EEE6'   # verde claro (fill), borda '#009E73'
COLOR_OUTPUT = '#F2E2EE'   # roxo claro (fill), borda '#CC79A7'

EDGE_SENSOR = '#0072B2'
EDGE_PROCESS = '#D55E00'
EDGE_FILTER = '#009E73'
EDGE_OUTPUT = '#CC79A7'


def box(ax, xy, w, h, text, facecolor, edgecolor, fontsize=7.5):
    x, y = xy
    rect = FancyBboxPatch((x, y), w, h, facecolor=facecolor, edgecolor=edgecolor,
                           **BOX_STYLE)
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center', fontsize=fontsize)
    return (x, y, w, h)


def arrow(ax, start_box, end_box, label='', label_offset=(0, 0.15)):
    sx, sy, sw, sh = start_box
    ex, ey, ew, eh = end_box
    start = (sx + sw / 2, sy) if sy > ey else (sx + sw / 2, sy + sh)
    end = (ex + ew / 2, ey + eh) if sy > ey else (ex + ew / 2, ey)
    if abs(sx - ex) > max(sw, ew):
        start = (sx + sw, sy + sh / 2) if sx < ex else (sx, sy + sh / 2)
        end = (ex, ey + eh / 2) if sx < ex else (ex + ew, ey + eh / 2)

    arr = FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=9,
                           linewidth=0.9, color='#333333')
    ax.add_patch(arr)
    if label:
        mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
        ax.text(mx + label_offset[0], my + label_offset[1], label,
                ha='center', va='center', fontsize=6.5, style='italic', color='#333333',
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.85, pad=1))


def main():
    fig, ax = plt.subplots(figsize=(3.5, 4.3))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 14)
    ax.axis('off')

    cam = box(ax, (0.4, 11.7), 2.8, 1.2, 'Overhead camera\n(Gazebo, 640×480, FOV 60°)',
              COLOR_SENSOR, EDGE_SENSOR)
    calib = box(ax, (0.4, 9.9), 2.8, 1.2, 'Camera calibration\n(cv2.calibrateCamera,\nZhang 2000)',
                COLOR_PROCESS, EDGE_PROCESS)
    yolo = box(ax, (0.4, 8.1), 2.8, 1.2, 'YOLOv8n detector\n(mAP@0.5 = 0.995)',
               COLOR_PROCESS, EDGE_PROCESS)

    odom = box(ax, (4.8, 11.7), 2.8, 1.2, 'Wheel odometry\n(per robot, ~29 Hz)',
               COLOR_SENSOR, EDGE_SENSOR)

    assoc = box(ax, (2.6, 6.0), 2.8, 1.3, 'Data association\n(Mahalanobis gating,\n' + r'$\chi^2$=9.21, 99% conf.)',
                COLOR_PROCESS, EDGE_PROCESS)

    ekf = box(ax, (1.6, 3.0), 4.8, 2.1,
              'Extended Kalman Filter (per robot)\n'
              r'State: [$p_x$, $p_y$, $\theta$]' + '\n'
              'Predict: odometry (Q)\n'
              r'Correct: detection ($R \propto$ confidence)',
              COLOR_FILTER, EDGE_FILTER, fontsize=8)

    out = box(ax, (2.6, 0.6), 2.8, 1.3, 'Fused pose estimate\n(/robotN/ekf_pose)',
              COLOR_OUTPUT, EDGE_OUTPUT)

    arrow(ax, cam, calib, 'raw image')
    arrow(ax, calib, yolo, 'intrinsics')
    arrow(ax, yolo, assoc, 'detections\n(x, y, conf)', label_offset=(0.3, 0.2))
    arrow(ax, odom, assoc, 'robot pose (v, ω)\nfor gating + predict', label_offset=(0.9, 0.2))
    arrow(ax, assoc, ekf, 'matched det.\n(correct)')
    arrow(ax, ekf, out, '')

    fig.tight_layout(pad=0.2)
    plt.savefig(OUT_PATH, dpi=400, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {OUT_PATH}')


if __name__ == '__main__':
    main()
