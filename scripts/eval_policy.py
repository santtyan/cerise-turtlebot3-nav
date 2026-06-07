#!/usr/bin/env python3
"""Avalia PPO vs baseline nearest_free no AllocationEnv (env leve).

Roda N episódios com sementes PAREADAS (mesma sequência de demandas para todas
as políticas) e reporta métricas comparáveis para o paper:
  - latência média e p95
  - makespan médio (tempo até a última tarefa terminar)
  - balanceamento de carga (desvio do nº de tarefas por robô)
  - taxa de ação inválida

Uso:
    python3 scripts/eval_policy.py --model models/ppo_allocator_yolo.zip
    python3 scripts/eval_policy.py --model models/ppo_allocator_yolo.zip --plot
"""

import argparse
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl import baselines                        # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv     # noqa: E402


def make_env(args, nav):
    return AllocationEnv(
        num_robots=args.num_robots, waypoints=WAYPOINT_SETS[args.waypoints],
        episode_len=args.episode_len,
        inter_arrival=args.inter_arrival, obs_source=args.obs_source,
        nav_model=nav, load_balance=not args.no_load_balance)


def eval_policy(predict_fn, args, nav, n_episodes):
    """Roda predict_fn em n_episodes (sementes 0..n-1) e agrega métricas."""
    env = make_env(args, nav)
    travels, makespans, imbalances, invalid, total = [], [], [], 0, 0
    for ep in range(n_episodes):
        obs, _ = env.reset(seed=ep)
        ep_travels, done = [], False
        while not done:
            a = predict_fn(obs, env)
            obs, _, term, trunc, info = env.step(a)
            ep_travels.append(info['travel_time'])
            invalid += int(info['invalid'])
            total += 1
            done = term or trunc
        travels.extend(ep_travels)
        makespans.append(sum(ep_travels))
        counts = info['task_count']
        mean_c = sum(counts) / len(counts)
        imbalances.append(float(np.std(counts)) / (mean_c + 1.0))
    return {
        'lat_mean': float(np.mean(travels)),
        'lat_p95': float(np.percentile(travels, 95)),
        'makespan_mean': float(np.mean(makespans)),
        'load_imbalance': float(np.mean(imbalances)),
        'invalid_rate': invalid / max(1, total),
    }


def main():
    p = argparse.ArgumentParser(description='Avalia PPO vs nearest_free.')
    p.add_argument('--model', help='Caminho do .zip do PPO (opcional).')
    p.add_argument('--obs-source', choices=['yolo', 'odom', 'none'],
                   default='yolo')
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'],
                   default='default')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--inter-arrival', type=float, default=30.0)
    p.add_argument('--episodes', type=int, default=1000)
    p.add_argument('--no-load-balance', action='store_true')
    p.add_argument('--plot', action='store_true', help='Gera gráfico comparativo.')
    args = p.parse_args()

    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s\n')

    results = {}

    # Baseline nearest_free (sempre).
    results['nearest_free'] = eval_policy(
        lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots),
        args, nav, args.episodes)

    # PPO, se um modelo foi fornecido.
    if args.model:
        from stable_baselines3 import PPO
        model = PPO.load(args.model)
        results['PPO'] = eval_policy(
            lambda obs, env: int(model.predict(obs, deterministic=True)[0]),
            args, nav, args.episodes)

    _print_table(results)
    if args.plot:
        _plot(results)


def _print_table(results):
    cols = ['lat_mean', 'lat_p95', 'makespan_mean', 'load_imbalance',
            'invalid_rate']
    header = ['política'] + cols
    print('  '.join(f'{h:>14s}' for h in header))
    for name, m in results.items():
        row = [name] + [f'{m[c]:.3f}' for c in cols]
        print('  '.join(f'{v:>14s}' for v in row))


def _plot(results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    metrics = ['lat_mean', 'lat_p95', 'makespan_mean', 'load_imbalance']
    labels = ['Latência média (s)', 'Latência p95 (s)',
              'Makespan médio (s)', 'Desbalanceamento']
    names = list(results.keys())
    fig, axes = plt.subplots(1, len(metrics), figsize=(4 * len(metrics), 4))
    for ax, met, lab in zip(axes, metrics, labels):
        vals = [results[n][met] for n in names]
        ax.bar(names, vals, color=['#888', '#7B2D8B'][:len(names)])
        ax.set_title(lab)
        ax.grid(axis='y', alpha=0.3)
    fig.suptitle('CERISE — PPO vs nearest_free (env leve)')
    fig.tight_layout()
    out = os.path.join(_REPO, 'docs', 'rl_eval_comparison.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=120)
    print(f'\n[plot] salvo em {out}')


if __name__ == '__main__':
    main()
