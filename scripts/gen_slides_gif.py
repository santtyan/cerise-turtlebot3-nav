#!/usr/bin/env python3
"""Gera GIF de alta qualidade para os slides CERISE.

PPO(YOLO) vs nearest_free lado a lado — mesmo episódio, seed fixo.
Visual limpo: fundo branco, robôs grandes, setas espessas, sem ruído.

Uso:
    python3 scripts/gen_slides_gif.py
"""
import os, sys
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.patches import FancyArrowPatch, Circle
from PIL import Image

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS
from cerise_nav.rl import baselines
from cerise_nav.rl.nav_model import calibrate_from_csv

SEED        = 7
EP_LEN      = 8
INTER       = 15.0
FRAME_MS    = 900   # ms por frame

ROBOT_COLORS = ['#E74C3C', '#3B82F6', '#10B981']   # vermelho, azul, verde
WP_COLOR     = '#64748B'
DEMAND_COLOR = '#F59E0B'
BG           = 'white'

LABEL_NF  = 'nearest_free\n(baseline guloso)'
LABEL_PPO = 'PPO  (YOLO)\n(agente treinado)'


def make_env(nav):
    return AllocationEnv(
        num_robots=3,
        waypoints=WAYPOINT_SETS['default'],
        episode_len=EP_LEN,
        inter_arrival=INTER,
        obs_source='none',
        nav_model=nav,
        seed=SEED,
    )


def collect_frames(policy_fn, env):
    obs, _ = env.reset(seed=SEED)
    frames, cum = [], 0.0
    done = False
    while not done:
        action = policy_fn(obs, env)
        frames.append({
            'positions': [tuple(p) for p in env._positions],
            'busy':      list(env._busy),
            'step':      env._step_idx,
            'action':    action,
            'cum':       cum,
            'demands':   list(env._demands),
        })
        obs, _, term, trunc, info = env.step(action)
        cum += info['response_time']
        done = term or trunc
    return frames, cum


def draw_panel(ax, f, waypoints, title, cum_label):
    ax.clear()
    ax.set_facecolor(BG)
    ax.set_xlim(-0.85, 0.85)
    ax.set_ylim(-0.85, 0.85)
    ax.set_aspect('equal')
    ax.axis('off')

    # Grade suave
    for v in [-0.55, 0, 0.55]:
        ax.axhline(v, color='#E2E8F0', lw=0.8, zorder=0)
        ax.axvline(v, color='#E2E8F0', lw=0.8, zorder=0)

    # Waypoints
    for name, (wx, wy) in waypoints.items():
        ax.plot(wx, wy, 's', color=WP_COLOR, markersize=22,
                alpha=0.18, zorder=1)
        ax.text(wx, wy, name, ha='center', va='center',
                fontsize=11, fontweight='bold', color=WP_COLOR, zorder=2)

    # Demanda atual
    step = f['step']
    if step < len(f['demands']):
        (ox, oy), (dx, dy) = f['demands'][step]
        ax.annotate('',
            xy=(dx, dy), xytext=(ox, oy),
            arrowprops=dict(arrowstyle='-|>', color=DEMAND_COLOR,
                            lw=3, mutation_scale=22),
            zorder=4)
        ax.plot(ox, oy, 'o', color=DEMAND_COLOR, markersize=10, zorder=5)
        ax.plot(dx, dy, '*', color=DEMAND_COLOR, markersize=16, zorder=5)
        ax.text(ox + 0.07, oy + 0.07, 'origem', fontsize=7.5,
                color=DEMAND_COLOR, fontweight='bold')
        ax.text(dx + 0.07, dy + 0.07, 'destino', fontsize=7.5,
                color=DEMAND_COLOR, fontweight='bold')

    # Robôs
    for r, (rx, ry) in enumerate(f['positions']):
        busy    = f['busy'][r] > 0
        chosen  = (r == f['action'])
        color   = ROBOT_COLORS[r]
        alpha   = 1.0 if not busy else 0.35
        radius  = 0.10

        circ = Circle((rx, ry), radius, color=color, alpha=alpha, zorder=6)
        ax.add_patch(circ)

        if chosen:
            ring = Circle((rx, ry), radius + 0.035,
                          fill=False, edgecolor='#1E293B', lw=2.5, zorder=7)
            ax.add_patch(ring)

        label = f'R{r+1}' + (' ✓' if chosen else '')
        ax.text(rx, ry, label, ha='center', va='center',
                fontsize=9, fontweight='bold', color='white', zorder=8,
                path_effects=[pe.withStroke(linewidth=1.5, foreground=color)])

    # Status robôs
    status = '   '.join(
        f'R{r+1} {"[ocup]" if f["busy"][r]>0 else "[livre]"}'
        for r in range(3)
    )
    ax.text(0, -0.80, status, ha='center', va='bottom', fontsize=7.5,
            color='#475569', fontfamily='monospace')

    # Título do painel
    ax.set_title(f'{title}\nDemanda {step+1}/{EP_LEN}  •  custo acum. {f["cum"]:.0f}s',
                 fontsize=10, fontweight='bold', color='#1E293B', pad=6)


def render_frame(frames_nf, frames_ppo, i, waypoints):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 4.5),
                                    facecolor=BG, constrained_layout=True)

    f_nf  = frames_nf[min(i, len(frames_nf)-1)]
    f_ppo = frames_ppo[min(i, len(frames_ppo)-1)]

    draw_panel(ax1, f_nf,  waypoints, LABEL_NF,  f_nf['cum'])
    draw_panel(ax2, f_ppo, waypoints, LABEL_PPO, f_ppo['cum'])

    fig.suptitle('CERISE — Alocação de Tarefas Multi-Robô',
                 fontsize=13, fontweight='bold', color='#0F172A', y=1.04)

    # Linha divisória
    fig.add_artist(plt.Line2D([0.5, 0.5], [0.02, 0.98],
                               transform=fig.transFigure,
                               color='#CBD5E1', lw=1.2))
    return fig


def main():
    nav = calibrate_from_csv()
    wp  = WAYPOINT_SETS['default']

    nf_fn  = lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots)
    from stable_baselines3 import PPO
    model  = PPO.load(os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip'))
    ppo_fn = lambda obs, env: int(model.predict(obs, deterministic=True)[0])

    env_nf  = make_env(nav)
    env_ppo = make_env(nav)

    print('Coletando frames nearest_free...')
    frames_nf,  cost_nf  = collect_frames(nf_fn,  env_nf)
    print('Coletando frames PPO...')
    frames_ppo, cost_ppo = collect_frames(ppo_fn, env_ppo)

    n_frames = max(len(frames_nf), len(frames_ppo))
    print(f'{n_frames} frames | NF={cost_nf:.0f}s | PPO={cost_ppo:.0f}s')

    tmp_dir = os.path.join(_REPO, 'docs', 'allocation_frames_fluid')
    os.makedirs(tmp_dir, exist_ok=True)

    pngs = []
    for i in range(n_frames):
        fig = render_frame(frames_nf, frames_ppo, i, wp)
        path = os.path.join(tmp_dir, f'f{i:04d}.png')
        fig.savefig(path, dpi=130, bbox_inches='tight', facecolor=BG)
        plt.close(fig)
        pngs.append(path)
        print(f'  frame {i+1}/{n_frames}', end='\r')

    print()

    out = os.path.join(_REPO, 'docs', 'slides_amanha', 'E_allocation_animated.gif')
    imgs = [Image.open(p).convert('RGB').convert('P', palette=Image.ADAPTIVE, colors=128)
            for p in pngs]
    imgs[0].save(out, save_all=True, append_images=imgs[1:],
                 duration=FRAME_MS, loop=0, optimize=True)
    size_kb = os.path.getsize(out) // 1024
    print(f'GIF salvo: {out}  ({size_kb} KB)')


if __name__ == '__main__':
    main()
