#!/usr/bin/env python3
"""Testes estatísticos para o paper LARS: PPO vs nearest_free vs oráculo.

Roda as políticas no AllocationEnv com sementes pareadas e aplica:
  - Wilcoxon signed-rank pareado (por episódio, custo total)
  - Correção de Holm-Bonferroni para múltiplas comparações
  - Cliff's delta (tamanho de efeito não-paramétrico)
  - IC 95% bootstrap do gap%

Uso:
    python3 scripts/stats_tests.py --ablation --episodes 1000
    python3 scripts/stats_tests.py --inter-arrival 12 --episodes 1000
"""

import argparse
import os
import sys
import math
import random

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl import baselines, oracle                             # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv                  # noqa: E402


# ---------------------------------------------------------------------------
# Coleta de dados

def collect_episode_costs(predict_fn, inter_arrival, nav, args, obs_source, n_ep):
    env = AllocationEnv(
        num_robots=args.num_robots, waypoints=WAYPOINT_SETS[args.waypoints],
        episode_len=args.episode_len, inter_arrival=inter_arrival,
        obs_source=obs_source, nav_model=nav,
        load_balance=not args.no_load_balance)
    costs = []
    for ep in range(n_ep):
        obs, _ = env.reset(seed=ep)
        ep_resp, done = [], False
        while not done:
            a = predict_fn(obs, env)
            obs, _, term, trunc, info = env.step(a)
            ep_resp.append(info['response_time'])
            done = term or trunc
        costs.append(sum(ep_resp))
    return np.array(costs)


# ---------------------------------------------------------------------------
# Estatísticas

def wilcoxon_signed_rank(x, y):
    """Wilcoxon signed-rank pareado (dois lados). Retorna (statistic, p_value)."""
    from scipy import stats
    return stats.wilcoxon(x, y, alternative='two-sided')


def cliffs_delta(x, y):
    """Cliff's delta: P(x > y) - P(x < y). Range [-1, 1]."""
    n1, n2 = len(x), len(y)
    greater = sum(xi > yj for xi in x for yj in y)
    less = sum(xi < yj for xi in x for yj in y)
    return (greater - less) / (n1 * n2)


def cliffs_magnitude(d):
    """Classificação de magnitude do Cliff's delta."""
    d = abs(d)
    if d < 0.147:
        return 'negligível'
    if d < 0.330:
        return 'pequeno'
    if d < 0.474:
        return 'médio'
    return 'grande'


def bootstrap_ci(x, y, metric_fn, n_boot=2000, alpha=0.05, seed=42):
    """IC bootstrap do metric_fn(x, y) via percentil. Retorna (low, high)."""
    rng = np.random.default_rng(seed)
    stats_boot = []
    n = len(x)
    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        stats_boot.append(metric_fn(x[idx], y[idx]))
    lo = float(np.percentile(stats_boot, 100 * alpha / 2))
    hi = float(np.percentile(stats_boot, 100 * (1 - alpha / 2)))
    return lo, hi


def gap_pct(costs_policy, costs_oracle):
    """Gap% médio de costs_policy relativo ao oráculo."""
    return float(np.mean((costs_policy - costs_oracle) / costs_oracle) * 100)


def holm_bonferroni(p_values):
    """Correção de Holm-Bonferroni. Retorna lista de p_adj na mesma ordem."""
    n = len(p_values)
    order = sorted(range(n), key=lambda i: p_values[i])
    p_adj = [0.0] * n
    running_max = 0.0
    for rank, i in enumerate(order):
        adj = p_values[i] * (n - rank)
        running_max = max(running_max, adj)
        p_adj[i] = min(1.0, running_max)
    return p_adj


# ---------------------------------------------------------------------------
# Main

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--ablation', action='store_true',
                   help='Compara PPO(yolo) vs PPO(odom) vs baseline vs oráculo.')
    p.add_argument('--model-yolo', default=os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip'))
    p.add_argument('--model-odom', default=os.path.join(_REPO, 'models', 'ppo_allocator_odom.zip'))
    p.add_argument('--model-yolo-masked', default=os.path.join(_REPO, 'models', 'ppo_allocator_yolo_masked.zip'))
    p.add_argument('--model-odom-masked', default=os.path.join(_REPO, 'models', 'ppo_allocator_odom_masked.zip'))
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'], default='default')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--inter-arrival', type=float, default=12.0,
                   help='Carga. Default=12 (alta carga, onde o PPO deve ganhar).')
    p.add_argument('--episodes', type=int, default=1000)
    p.add_argument('--beam-width', type=int, default=200)
    p.add_argument('--no-load-balance', action='store_true')
    p.add_argument('--n-boot', type=int, default=2000)
    args = p.parse_args()

    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s')
    print(f'Carga: inter_arrival={args.inter_arrival}s | {args.episodes} episódios pareados\n')

    baseline_fn  = lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots)
    random_fn    = lambda obs, env: baselines.random_policy(obs, env.num_robots)
    rr_fn        = baselines.make_round_robin(args.num_robots)
    oracle_fn    = oracle.make_oracle_policy(args.beam_width)

    print('Coletando oráculo...')
    c_oracle = collect_episode_costs(oracle_fn, args.inter_arrival, nav, args, 'none', args.episodes)

    print('Coletando baseline (nearest_free)...')
    c_base = collect_episode_costs(baseline_fn, args.inter_arrival, nav, args, 'yolo', args.episodes)

    print('Coletando random...')
    c_random = collect_episode_costs(random_fn, args.inter_arrival, nav, args, 'yolo', args.episodes)

    print('Coletando round-robin...')
    c_rr = collect_episode_costs(rr_fn, args.inter_arrival, nav, args, 'yolo', args.episodes)

    # nearest_free é a referência pareada para os testes (não o oráculo).
    # O gap% é calculado vs oráculo; significância é testada vs nearest_free.
    policies = {
        'random':       (c_random, 'yolo'),
        'round_robin':  (c_rr, 'yolo'),
        'nearest_free': (c_base, 'yolo'),
    }

    if args.ablation:
        from stable_baselines3 import PPO
        from sb3_contrib import MaskablePPO
        for label, path, src, masked in [
            ('PPO(yolo)', args.model_yolo, 'yolo', False),
            ('PPO(odom)', args.model_odom, 'odom', False),
            ('PPO(yolo,masked)', args.model_yolo_masked, 'yolo', True),
            ('PPO(odom,masked)', args.model_odom_masked, 'odom', True),
        ]:
            if not os.path.exists(path):
                print(f'[aviso] {path} não encontrado — pulando {label}.')
                continue
            if masked:
                m = MaskablePPO.load(path)
                fn = lambda obs, env, m=m: int(m.predict(
                    obs, action_masks=env.action_masks(), deterministic=True)[0])
            else:
                m = PPO.load(path)
                fn = lambda obs, env, m=m: int(m.predict(obs, deterministic=True)[0])
            print(f'Coletando {label}...')
            costs = collect_episode_costs(fn, args.inter_arrival, nav, args, src, args.episodes)
            policies[label] = (costs, src)

    # Pares de comparação: cada política vs nearest_free (referência)
    comparisons = [(name, costs) for name, (costs, _) in policies.items() if name != 'nearest_free']
    p_values_raw = []
    results = []

    for name, costs in comparisons:
        stat, pval = wilcoxon_signed_rank(costs, c_base)
        d = cliffs_delta(costs.tolist(), c_base.tolist())
        gap = gap_pct(costs, c_oracle)
        gap_base = gap_pct(c_base, c_oracle)
        gap_lo, gap_hi = bootstrap_ci(costs, c_oracle,
                                      lambda x, y: gap_pct(x, y), args.n_boot)
        results.append({
            'name': name, 'stat': stat, 'p_raw': pval, 'd': d,
            'gap': gap, 'gap_base': gap_base, 'gap_lo': gap_lo, 'gap_hi': gap_hi,
        })
        p_values_raw.append(pval)

    p_adj = holm_bonferroni(p_values_raw)
    for i, r in enumerate(results):
        r['p_adj'] = p_adj[i]

    # --- Impressão ---
    print(f'\n{"Política":<18} {"gap%":>8} {"IC95%":>18} {"vs base":>8} '
          f'{"Cliff d":>10} {"magn.":>10} {"p (raw)":>10} {"p (Holm)":>10} {"sig":>5}')
    print('-' * 105)

    gap_b = gap_pct(c_base, c_oracle)
    print(f'{"nearest_free":<18} {gap_b:>+8.1f}   {"—":>18}   {"—":>8}   '
          f'{"—":>10}   {"—":>10}   {"—":>10}   {"—":>10}   {"—":>5}')

    for r in results:
        sig = '***' if r['p_adj'] < 0.001 else ('**' if r['p_adj'] < 0.01
              else ('*' if r['p_adj'] < 0.05 else 'ns'))
        diff = r['gap'] - gap_b
        ic = f'[{r["gap_lo"]:+.1f}, {r["gap_hi"]:+.1f}]'
        print(f'{r["name"]:<18} {r["gap"]:>+8.1f}   {ic:>18}   {diff:>+8.1f}   '
              f'{r["d"]:>+10.3f}   {cliffs_magnitude(r["d"]):>10}   '
              f'{r["p_raw"]:>10.4f}   {r["p_adj"]:>10.4f}   {sig:>5}')

    print()
    print('Oráculo (teto):  gap=+0.0% (por definição)')
    print(f'Baseline gap vs oráculo: {gap_b:+.1f}%')
    print('\nLegenda: gap% = quanto cada política está acima do oráculo (↓ melhor).')
    print('         Cliff d: positivo = política é PIOR que baseline.')
    print('         sig: * p<0.05, ** p<0.01, *** p<0.001 (Holm-Bonferroni).')


if __name__ == '__main__':
    main()
