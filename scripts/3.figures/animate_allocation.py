#!/usr/bin/env python3
"""Visualização 2D da alocação de tarefas no AllocationEnv.

Gera frames PNG (ou GIF animado) mostrando:
  - Waypoints marcados no plano
  - Robôs como círculos coloridos (verde=livre, vermelho=ocupado)
  - Demanda corrente como seta origem → destino
  - Título mostrando qual política escolheu qual robô e o custo acumulado

Útil para debug visual (por que PPO ≠ nearest_free) e como figura pro paper.

Uso:
    python3 scripts/3.figures/animate_allocation.py --policy nearest_free --gif
    python3 scripts/3.figures/animate_allocation.py --policy ppo --model models/ppo_allocator_yolo.zip
    python3 scripts/3.figures/animate_allocation.py --compare  # lado a lado nearest_free vs PPO
"""

import argparse
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl import baselines, oracle                             # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv                  # noqa: E402


# Cores por robô (até 4 robôs)
ROBOT_COLORS = ['#E74C3C', '#3498DB', '#2ECC71', '#9B59B6']
FREE_ALPHA   = 0.9
BUSY_ALPHA   = 0.4


def make_policy(args):
    name = args.policy
    if name == 'nearest_free':
        return lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots), 'nearest_free'
    if name == 'random':
        return lambda obs, env: baselines.random_policy(obs, env.num_robots), 'random'
    if name == 'round_robin':
        return baselines.make_round_robin(args.num_robots), 'round_robin'
    if name == 'oracle':
        fn = oracle.make_oracle_policy(args.beam_width)
        return fn, 'oracle'
    if name == 'ppo':
        from stable_baselines3 import PPO
        m = PPO.load(args.model)
        return lambda obs, env: int(m.predict(obs, deterministic=True)[0]), 'PPO'
    raise ValueError(f'política desconhecida: {name}')


def render_frame(ax, env, action, step_idx, policy_name, cum_cost, waypoints):
    """Renderiza um frame do estado ANTES do step (demanda ainda não atendida)."""
    ax.clear()
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    ax.set_xlabel('x (m)', fontsize=10)
    ax.set_ylabel('y (m)', fontsize=10)

    # Waypoints
    for name, (wx, wy) in waypoints.items():
        ax.plot(wx, wy, 's', color='#95A5A6', markersize=10, zorder=2)
        ax.text(wx + 0.05, wy + 0.05, name, fontsize=9, color='#555')

    # Demanda corrente: seta origem → destino
    if env._step_idx < env.episode_len:
        (ox, oy), (dx, dy) = env._demands[env._step_idx]
        ax.annotate('', xy=(dx, dy), xytext=(ox, oy),
                    arrowprops=dict(arrowstyle='->', color='#E67E22',
                                    lw=2.5, mutation_scale=18))
        ax.plot(ox, oy, 'o', color='#E67E22', markersize=8, zorder=5, label='origem')
        ax.plot(dx, dy, '*', color='#E67E22', markersize=12, zorder=5, label='destino')

    # Robôs
    for r, (rx, ry) in enumerate(env._positions):
        busy = env._busy[r] > 0
        alpha = BUSY_ALPHA if busy else FREE_ALPHA
        color = ROBOT_COLORS[r % len(ROBOT_COLORS)]
        chosen = (r == action)
        edgecolor = 'black' if chosen else color
        lw = 2.5 if chosen else 1.0
        ax.plot(rx, ry, 'o', color=color, markersize=18, alpha=alpha,
                markeredgecolor=edgecolor, markeredgewidth=lw, zorder=6)
        label = f'R{r+1}' + (' ✓' if chosen else '')
        ax.text(rx, ry, label, ha='center', va='center',
                fontsize=8, fontweight='bold', color='white', zorder=7)

    status = ' | '.join(
        f'R{r+1}:{"ocupado" if env._busy[r]>0 else "livre"}'
        for r in range(env.num_robots))
    ax.set_title(
        f'[{policy_name}] Step {step_idx+1}/{env.episode_len} — '
        f'custo acum={cum_cost:.1f}s\n{status}',
        fontsize=10)


def run_episode_frames(policy_fn, env, seed, waypoints, policy_name):
    """Roda um episódio e coleta (frame_data) para animação."""
    obs, _ = env.reset(seed=seed)
    frames = []
    cum_cost = 0.0
    done = False
    while not done:
        action = policy_fn(obs, env)
        # Snapshot ANTES do step
        frames.append({
            'positions': list(env._positions),
            'busy': list(env._busy),
            'step_idx': env._step_idx,
            'action': action,
            'cum_cost': cum_cost,
        })
        obs, _, term, trunc, info = env.step(action)
        cum_cost += info['response_time']
        done = term or trunc
    return frames, cum_cost


def save_frames(frames, env, waypoints, policy_name, out_dir, seed):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    fig, ax = plt.subplots(figsize=(5, 5))

    # Reconstruir env só para renderização (sem side effects)
    render_env = AllocationEnv(
        num_robots=env.num_robots, waypoints=env.waypoints,
        episode_len=env.episode_len, inter_arrival=env.inter_arrival,
        obs_source='none', nav_model=env.nav, seed=seed)
    render_env.reset(seed=seed)

    paths = []
    for i, f in enumerate(frames):
        render_env._positions = [tuple(p) for p in f['positions']]
        render_env._busy = list(f['busy'])
        render_env._step_idx = f['step_idx']
        render_frame(ax, render_env, f['action'], f['step_idx'],
                     policy_name, f['cum_cost'], waypoints)
        fig.tight_layout()
        path = os.path.join(out_dir, f'frame_{i:03d}.png')
        fig.savefig(path, dpi=100, bbox_inches='tight')
        paths.append(path)

    plt.close(fig)
    return paths


def make_gif(frame_paths, out_path, duration=600):
    try:
        from PIL import Image
    except ImportError:
        print('[aviso] Pillow não instalado — pulando GIF. pip install Pillow')
        return
    imgs = [Image.open(p) for p in frame_paths]
    imgs[0].save(out_path, save_all=True, append_images=imgs[1:],
                 duration=duration, loop=0)
    print(f'[gif] salvo em {out_path}')


def compare_side_by_side(policies, env, waypoints, seed, out_path):
    """Gera imagem estática lado a lado de todas as políticas no mesmo step."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    n = len(policies)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 5))
    if n == 1:
        axes = [axes]

    for ax, (policy_name, policy_fn) in zip(axes, policies):
        obs, _ = env.reset(seed=seed)
        action = policy_fn(obs, env)
        # Renderiza só o primeiro step
        env._step_idx = 0
        render_frame(ax, env, action, 0, policy_name, 0.0, waypoints)

    fig.suptitle(f'Comparação de políticas — seed={seed}', fontsize=13, fontweight='bold')
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=130, bbox_inches='tight')
    print(f'[compare] salvo em {out_path}')


def main():
    p = argparse.ArgumentParser(description='Visualiza alocação 2D por episódio.')
    p.add_argument('--policy', choices=['nearest_free', 'random', 'round_robin', 'oracle', 'ppo'],
                   default='nearest_free')
    p.add_argument('--model', default=os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip'))
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'], default='default')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--inter-arrival', type=float, default=12.0)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--beam-width', type=int, default=100)
    p.add_argument('--gif', action='store_true', help='Gera GIF animado.')
    p.add_argument('--compare', action='store_true',
                   help='Compara nearest_free vs PPO lado a lado (1 frame).')
    p.add_argument('--out-dir', default=os.path.join(_REPO, 'docs', 'allocation_frames'))
    args = p.parse_args()

    nav = calibrate_from_csv()
    wp = WAYPOINT_SETS[args.waypoints]
    env = AllocationEnv(
        num_robots=args.num_robots, waypoints=wp,
        episode_len=args.episode_len, inter_arrival=args.inter_arrival,
        obs_source='none', nav_model=nav)

    if args.compare:
        policies = [
            ('nearest_free', lambda o, e: baselines.nearest_free_policy(o, e.num_robots)),
        ]
        if os.path.exists(args.model):
            from stable_baselines3 import PPO
            m = PPO.load(args.model)
            policies.append(('PPO', lambda o, e: int(m.predict(o, deterministic=True)[0])))
        out = os.path.join(_REPO, 'docs', 'allocation_compare.png')
        compare_side_by_side(policies, env, wp, args.seed, out)
        return

    policy_fn, policy_name = make_policy(args)
    frames, total_cost = run_episode_frames(policy_fn, env, args.seed, wp, policy_name)
    print(f'[{policy_name}] seed={args.seed} | {len(frames)} steps | custo={total_cost:.1f}s')

    frame_paths = save_frames(frames, env, wp, policy_name, args.out_dir, args.seed)
    print(f'[frames] {len(frame_paths)} PNGs em {args.out_dir}/')

    if args.gif:
        gif_path = os.path.join(_REPO, 'docs', f'allocation_{policy_name}.gif')
        make_gif(frame_paths, gif_path)


if __name__ == '__main__':
    main()
