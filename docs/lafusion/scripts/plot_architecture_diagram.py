#!/usr/bin/env python3
"""Gera diagrama de arquitetura do pipeline de fusão EKF para a seção
Methodology do paper LAFusion.

Uso: python3 scripts/plot_architecture_diagram.py
Saída: docs/lafusion_architecture.png
"""

import os

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'docs', 'lafusion_architecture.png')

BOX_STYLE = dict(boxstyle='round,pad=0.4', linewidth=1.5)
COLORS = {
    'sensor': '#cfe8ff',
    'process': '#ffe8b3',
    'filter': '#c8f0c8',
    'output': '#f0c8c8',
}


def box(ax, xy, w, h, text, color, fontsize=10):
    x, y = xy
    rect = FancyBboxPatch((x, y), w, h, facecolor=color, edgecolor='black', **BOX_STYLE)
    ax.add_patch(rect)
    ax.text(x + w / 2, y + h / 2, text, ha='center', va='center',
            fontsize=fontsize, wrap=True)
    return (x, y, w, h)


def arrow(ax, start_box, end_box, label='', label_offset=(0, 0.15)):
    sx, sy, sw, sh = start_box
    ex, ey, ew, eh = end_box
    start = (sx + sw / 2, sy) if sy > ey else (sx + sw / 2, sy + sh)
    end = (ex + ew / 2, ey + eh) if sy > ey else (ex + ew / 2, ey)
    if abs(sx - ex) > max(sw, ew):
        start = (sx + sw, sy + sh / 2) if sx < ex else (sx, sy + sh / 2)
        end = (ex, ey + eh / 2) if sx < ex else (ex + ew, ey + eh / 2)

    arr = FancyArrowPatch(start, end, arrowstyle='-|>', mutation_scale=15,
                           linewidth=1.5, color='black')
    ax.add_patch(arr)
    if label:
        mx, my = (start[0] + end[0]) / 2, (start[1] + end[1]) / 2
        ax.text(mx + label_offset[0], my + label_offset[1], label,
                ha='center', va='center', fontsize=8, style='italic',
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.8, pad=1))


def main():
    fig, ax = plt.subplots(figsize=(9, 11))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 14)
    ax.axis('off')

    # Coluna esquerda: pipeline de câmera (topo -> baixo), com espaço entre caixas
    cam = box(ax, (0.4, 11.7), 2.8, 1.2, 'Overhead Camera\n(Gazebo, 640×480,\nFOV 60°)', COLORS['sensor'])
    calib = box(ax, (0.4, 9.9), 2.8, 1.2, 'Camera calibration\n(cv2.calibrateCamera,\nZhang 2000)', COLORS['process'], fontsize=9)
    yolo = box(ax, (0.4, 8.1), 2.8, 1.2, 'YOLOv8n Detector\n(mAP@0.5=0.995)', COLORS['process'])

    # Coluna direita: odometria (topo, alinhada com a câmera)
    odom = box(ax, (4.8, 11.7), 2.8, 1.2, 'Wheel Odometry\n(per robot, ~29 Hz)', COLORS['sensor'])

    # Centro: associação, espaço livre acima e abaixo
    assoc = box(ax, (2.6, 6.0), 2.8, 1.3, 'Data Association\n(Mahalanobis gating,\nχ²=9.21, 99% conf.)', COLORS['process'], fontsize=9)

    # EKF central, com espaço claro abaixo da associação
    ekf = box(ax, (1.6, 3.0), 4.8, 2.1,
              'Extended Kalman Filter (per robot)\nState: [px, py, θ]\nPredict: odometry (Q)\nCorrect: detection (R ∝ confidence)',
              COLORS['filter'], fontsize=10)

    # Output, abaixo do EKF (não mais lateral, evita sobreposição)
    out = box(ax, (2.6, 0.6), 2.8, 1.3, 'Fused pose estimate\n(/robotN/ekf_pose)', COLORS['output'], fontsize=9)

    # Arrows — fluxo vertical na coluna da câmera, depois convergindo no centro.
    # Odometria alimenta tanto a associação (posição de referência para o
    # gating) quanto a predição do EKF (v, ω) — só uma seta é desenhada
    # (odom->assoc) para não sobrepor rótulos; o uso na predição do EKF já
    # está indicado no texto da própria caixa ("Predict: odometry (Q)").
    arrow(ax, cam, calib, 'raw image')
    arrow(ax, calib, yolo, 'intrinsics')
    arrow(ax, yolo, assoc, 'detections\n(x, y, conf)', label_offset=(0.3, 0.2))
    arrow(ax, odom, assoc, 'robot pose (v, ω)\nfor gating + predict', label_offset=(0.9, 0.2))
    arrow(ax, assoc, ekf, 'matched det.\n(correct)')
    arrow(ax, ekf, out, '')

    ax.set_title('CERISE EKF Fusion Pipeline: Camera + Odometry', fontsize=13, pad=15)

    plt.tight_layout()
    plt.savefig(OUT_PATH, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'Salvo: {OUT_PATH}')


if __name__ == '__main__':
    main()
