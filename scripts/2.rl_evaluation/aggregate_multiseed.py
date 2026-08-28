#!/usr/bin/env python3
"""Agrega resultados de múltiplos seeds do PPO e reporta média±std.

Uso:
    python3 scripts/2.rl_evaluation/aggregate_multiseed.py --seeds 42 123 7 --inter-arrival 12
"""
import argparse, os, sys
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv
from cerise_nav.rl.nav_model import calibrate_from_csv
from cerise_nav.rl import baselines, oracle


def eval_seed(model_path, obs_source, args, nav, n_ep=200):
    from stable_baselines3 import PPO
    model = PPO.load(model_path)
    env = AllocationEnv(num_robots=args.num_robots, inter_arrival=args.inter_arrival,
                        obs_source=obs_source, nav_model=nav, episode_len=args.episode_len)
    costs = []
    for ep in range(n_ep):
        obs, _ = env.reset(seed=ep)
        total = 0.0
        done = False
        while not done:
            action = int(model.predict(obs, deterministic=True)[0])
            obs, _, term, trunc, info = env.step(action)
            total += info['response_time']
            done = term or trunc
        costs.append(total)
    return np.array(costs)


def eval_baseline(policy_fn, obs_source, args, nav, n_ep=200):
    env = AllocationEnv(num_robots=args.num_robots, inter_arrival=args.inter_arrival,
                        obs_source=obs_source, nav_model=nav, episode_len=args.episode_len)
    costs = []
    for ep in range(n_ep):
        obs, _ = env.reset(seed=ep)
        total = 0.0
        done = False
        while not done:
            action = policy_fn(obs, env.num_robots)
            obs, _, term, trunc, info = env.step(action)
            total += info['response_time']
            done = term or trunc
        costs.append(total)
    return np.array(costs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seeds', nargs='+', type=int, default=[42, 123, 7])
    p.add_argument('--inter-arrival', type=float, default=12.0)
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--episodes', type=int, default=200)
    p.add_argument('--model-dir', default=os.path.join(_REPO, 'models', 'multiseed'))
    args = p.parse_args()

    nav = calibrate_from_csv()
    n_ep = args.episodes

    # Baseline (determinístico — 1 run é suficiente)
    print('Avaliando nearest_free...')
    nf_costs = eval_baseline(baselines.nearest_free_policy, 'none', args, nav, n_ep)
    oracle_fn = oracle.make_oracle_policy()
    print('Avaliando oráculo...')
    or_costs = eval_baseline(lambda obs, n: oracle_fn(obs, None), 'none', args, nav, n_ep)
    oracle_mean = or_costs.mean()

    print(f'\n{"Política":<22} {"Custo médio":>12} {"Std":>8} {"Gap% vs oráculo":>18}')
    print('-' * 65)

    def gap(c): return 100 * (c - oracle_mean) / oracle_mean

    print(f'{"nearest_free":<22} {nf_costs.mean():>12.2f} {nf_costs.std():>8.2f} {gap(nf_costs.mean()):>17.1f}%')
    print(f'{"oracle":<22} {or_costs.mean():>12.2f} {or_costs.std():>8.2f} {"0.0":>17}%')

    for src in ['yolo', 'odom']:
        seed_means = []
        for seed in args.seeds:
            path = os.path.join(args.model_dir, f'ppo_{src}_seed{seed}.zip')
            if not os.path.exists(path):
                # fallback: nome sem seed
                path = os.path.join(_REPO, 'models', f'ppo_allocator_{src}.zip')
            if not os.path.exists(path):
                print(f'  [aviso] modelo não encontrado: {path}')
                continue
            costs = eval_seed(path, src, args, nav, n_ep)
            seed_means.append(costs.mean())
            print(f'  PPO({src}) seed={seed}: {costs.mean():.2f} ± {costs.std():.2f}  gap={gap(costs.mean()):.1f}%')

        if seed_means:
            arr = np.array(seed_means)
            print(f'{"PPO("+src+") agregado":<22} {arr.mean():>12.2f} {arr.std():>8.2f} {gap(arr.mean()):>17.1f}%')

    print()


if __name__ == '__main__':
    main()
