#!/usr/bin/env python3
"""Varre inter_arrival para mapear o crossover PPO vs nearest_free.

Para cada carga (inter_arrival), roda N episódios pareados com todas as
políticas e calcula o gap% vs oráculo. O crossover aparece como o ponto onde
PPO(yolo) < nearest_free no eixo de response_time.

Uso:
    python3 scripts/sweep_load.py --plot
    python3 scripts/sweep_load.py --episodes 200 --quick   # rápido (~2 min)
"""

import argparse
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl import baselines, oracle                             # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv                  # noqa: E402

# Cargas a varrer (inter_arrival em segundos). Alta carga = inter_arrival pequeno.
LOAD_POINTS = [30, 25, 20, 15, 12, 10]


def eval_policy_at_load(predict_fn, inter_arrival, nav, args, obs_source=None):
    env = AllocationEnv(
        num_robots=args.num_robots,
        waypoints=WAYPOINT_SETS[args.waypoints],
        episode_len=args.episode_len,
        inter_arrival=inter_arrival,
        obs_source=obs_source or args.obs_source,
        nav_model=nav,
        load_balance=not args.no_load_balance,
    )
    ep_costs = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=ep)
        ep_resp, done = [], False
        while not done:
            a = predict_fn(obs, env)
            obs, _, term, trunc, info = env.step(a)
            ep_resp.append(info['response_time'])
            done = term or trunc
        ep_costs.append(sum(ep_resp))
    return float(np.mean(ep_costs))


def main():
    p = argparse.ArgumentParser(description='Sweep de carga para curva de crossover PPO vs baseline.')
    p.add_argument('--model-yolo', default=os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip'))
    p.add_argument('--model-odom', default=os.path.join(_REPO, 'models', 'ppo_allocator_odom.zip'))
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'], default='default')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--episodes', type=int, default=500)
    p.add_argument('--obs-source', default='yolo')
    p.add_argument('--no-load-balance', action='store_true')
    p.add_argument('--beam-width', type=int, default=200)
    p.add_argument('--quick', action='store_true', help='Usa 200 episódios e beam=100.')
    p.add_argument('--plot', action='store_true')
    p.add_argument('--load-points', type=float, nargs='+', default=LOAD_POINTS)
    p.add_argument('--no-ppo', action='store_true', help='Pula PPO (só baseline + oráculo).')
    args = p.parse_args()

    if args.quick:
        args.episodes = 200
        args.beam_width = 100

    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s\n')

    policies = {}

    policies['random'] = (lambda obs, env: baselines.random_policy(obs, env.num_robots), 'yolo')
    policies['round_robin'] = (baselines.make_round_robin(args.num_robots), 'yolo')
    policies['nearest_free'] = (lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots), 'yolo')

    oracle_fn = oracle.make_oracle_policy(args.beam_width)
    policies['oracle'] = (oracle_fn, 'none')

    if not args.no_ppo:
        from stable_baselines3 import PPO
        if os.path.exists(args.model_yolo):
            m_yolo = PPO.load(args.model_yolo)
            policies['PPO(yolo)'] = (
                lambda obs, env, m=m_yolo: int(m.predict(obs, deterministic=True)[0]),
                'yolo')
        else:
            print(f'[aviso] {args.model_yolo} não encontrado — pulando PPO(yolo).')

        if os.path.exists(args.model_odom):
            m_odom = PPO.load(args.model_odom)
            policies['PPO(odom)'] = (
                lambda obs, env, m=m_odom: int(m.predict(obs, deterministic=True)[0]),
                'odom')
        else:
            print(f'[aviso] {args.model_odom} não encontrado — pulando PPO(odom).')

    load_pts = sorted(args.load_points, reverse=True)
    results = {name: [] for name in policies}

    print(f'{"inter_arrival":>14s}' + ''.join(f'{n:>14s}' for n in policies) + f'{"gap_ppo%":>10s}')
    for ia in load_pts:
        row = {}
        for name, (fn, src) in policies.items():
            cost = eval_policy_at_load(fn, ia, nav, args, obs_source=src)
            results[name].append(cost)
            row[name] = cost

        oracle_cost = row.get('oracle', None)
        gap_str = ''
        if oracle_cost and 'PPO(yolo)' in row:
            gap = 100.0 * (row['PPO(yolo)'] - oracle_cost) / oracle_cost
            gap_base = 100.0 * (row['nearest_free'] - oracle_cost) / oracle_cost
            gap_str = f'{gap:+.1f}% (base:{gap_base:+.1f}%)'

        print(f'{ia:>14.1f}' + ''.join(f'{row[n]:>14.2f}' for n in policies) + f'  {gap_str}')

    if args.plot:
        _plot(load_pts, results, policies)


def _plot(load_pts, results, policies):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colors = {
        'random':       '#AAAAAA',
        'round_robin':  '#E57373',
        'nearest_free': '#888888',
        'oracle':       '#3FA34D',
        'PPO(yolo)':    '#7B2D8B',
        'PPO(odom)':    '#2D7B8B',
    }
    styles = {
        'random':       ':D',
        'round_robin':  '--x',
        'nearest_free': '-o',
        'oracle':       '-s',
        'PPO(yolo)':    '-^',
        'PPO(odom)':    '-v',
    }

    fig, ax = plt.subplots(figsize=(8, 5))
    for name, costs in results.items():
        if not costs:
            continue
        ax.plot(load_pts, costs, styles.get(name, '-o'),
                color=colors.get(name, '#444444'),
                label=name, linewidth=2, markersize=7)

    ax.set_xlabel('Inter-arrival (s)  [←  high load   |   low load  →]', fontsize=12)
    ax.set_ylabel('Mean cost (∑ response_time per ep, s)', fontsize=12)
    ax.set_title('CERISE — Response Cost vs Load (nearest_free ≈ optimal)',
                 fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    ax.invert_xaxis()

    out = os.path.join(_REPO, 'docs', 'rl_crossover_inter_arrival.png')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\n[plot] salvo em {out}')


if __name__ == '__main__':
    main()
