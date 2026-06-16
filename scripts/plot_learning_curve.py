#!/usr/bin/env python3
"""Gera curva de aprendizado retreinando PPO com checkpoints a cada 50k steps.

Avalia cada checkpoint no env leve (200 episódios) e plota ep_rew_mean
para PPO(yolo), PPO(odom) e a linha do nearest_free como referência.

Uso:
    python3 scripts/plot_learning_curve.py
"""
import os
import sys
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS
from cerise_nav.rl import baselines
from cerise_nav.rl.nav_model import calibrate_from_csv

TOTAL_STEPS   = 500_000
EVAL_INTERVAL = 50_000
N_EVAL_EPS    = 200
N_ENVS        = 8
SEED          = 42

ENV_KWARGS = dict(
    num_robots=3,
    waypoints=WAYPOINT_SETS['default'],
    episode_len=20,
    inter_arrival=30.0,
    load_balance=True,
)


def mean_reward(model, nav, obs_source, n_eps):
    env = AllocationEnv(**ENV_KWARGS, obs_source=obs_source, nav_model=nav)
    rewards = []
    for ep in range(n_eps):
        obs, _ = env.reset(seed=ep)
        ep_r, done = 0.0, False
        while not done:
            a = int(model.predict(obs, deterministic=True)[0])
            obs, r, term, trunc, _ = env.step(a)
            ep_r += r
            done = term or trunc
        rewards.append(ep_r)
    return float(np.mean(rewards))


def baseline_reward(nav, n_eps):
    env = AllocationEnv(**ENV_KWARGS, obs_source='yolo', nav_model=nav)
    rewards = []
    for ep in range(n_eps):
        obs, _ = env.reset(seed=ep)
        ep_r, done = 0.0, False
        while not done:
            a = baselines.nearest_free_policy(obs, 3)
            obs, r, term, trunc, _ = env.step(a)
            ep_r += r
            done = term or trunc
        rewards.append(ep_r)
    return float(np.mean(rewards))


def train_with_checkpoints(obs_source, nav):
    vec_env = make_vec_env(
        AllocationEnv, n_envs=N_ENVS, seed=SEED,
        env_kwargs={**ENV_KWARGS, 'obs_source': obs_source, 'nav_model': nav})
    model = PPO('MlpPolicy', vec_env, verbose=0, seed=SEED)

    steps_done = 0
    curve = []
    checkpoints = list(range(EVAL_INTERVAL, TOTAL_STEPS + 1, EVAL_INTERVAL))

    for ckpt in checkpoints:
        model.learn(total_timesteps=ckpt - steps_done, reset_num_timesteps=False)
        steps_done = ckpt
        r = mean_reward(model, nav, obs_source, N_EVAL_EPS)
        curve.append(r)
        print(f'  [{obs_source}] {ckpt:>7d} steps → ep_rew_mean={r:.2f}')

    return checkpoints, curve


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s\n')

    print('Calculando baseline nearest_free...')
    bl_r = baseline_reward(nav, N_EVAL_EPS)
    print(f'  nearest_free ep_rew_mean={bl_r:.2f}\n')

    print('Treinando PPO(yolo) com checkpoints...')
    steps, curve_yolo = train_with_checkpoints('yolo', nav)

    print('\nTreinando PPO(odom) com checkpoints...')
    _, curve_odom = train_with_checkpoints('odom', nav)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(steps, curve_yolo, color='#7B2D8B', linewidth=2, marker='o',
            markersize=4, label='PPO(YOLO)')
    ax.plot(steps, curve_odom, color='#2D7B8B', linewidth=2, marker='s',
            markersize=4, label='PPO(odom)')
    ax.axhline(bl_r, color='#888888', linewidth=1.5, linestyle='--',
               label=f'nearest_free ({bl_r:.1f})')

    ax.set_xlabel('Timesteps de treino', fontsize=12)
    ax.set_ylabel('Reward médio por episódio', fontsize=12)
    ax.set_title('CERISE — Curva de Aprendizado PPO(YOLO) vs PPO(odom)',
                 fontsize=13, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_xlim(0, TOTAL_STEPS)

    out = os.path.join(_REPO, 'docs', 'rl_learning_curve_ablation.png')
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\n[plot] salvo em {out}')


if __name__ == '__main__':
    main()
