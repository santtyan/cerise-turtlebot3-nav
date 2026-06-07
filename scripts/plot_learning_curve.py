#!/usr/bin/env python3
"""Gera curva de aprendizado do PPO e salva em docs/rl_learning_curve.png."""

import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS
from cerise_nav.rl import baselines
from cerise_nav.rl.nav_model import calibrate_from_csv


class EpisodeRewardCallback(BaseCallback):
    """Registra reward médio por episódio a cada rollout via Monitor."""
    def __init__(self, eval_freq=4096):
        super().__init__()
        self.eval_freq = eval_freq
        self.timesteps = []
        self.rewards = []
        self._last_log = 0

    def _on_step(self):
        if self.num_timesteps - self._last_log >= self.eval_freq:
            # Pega ep_rew_mean do buffer de infos do VecEnv
            infos = self.locals.get('infos', [])
            ep_rewards = [i['episode']['r'] for i in infos
                          if 'episode' in i]
            if ep_rewards:
                self.timesteps.append(self.num_timesteps)
                self.rewards.append(np.mean(ep_rewards))
                self._last_log = self.num_timesteps
        return True


def compute_baseline_reward(nav, n_episodes=500):
    env = AllocationEnv(
        num_robots=3, waypoints=WAYPOINT_SETS['expanded'],
        episode_len=30, inter_arrival=15, nav_model=nav)
    rewards = []
    for ep in range(n_episodes):
        obs, _ = env.reset(seed=ep)
        done, total = False, 0.0
        while not done:
            a = baselines.nearest_free_policy(obs, env.num_robots)
            obs, r, term, trunc, _ = env.step(a)
            total += r
            done = term or trunc
        rewards.append(total)
    return float(np.mean(rewards))


def main():
    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s')

    baseline_reward = compute_baseline_reward(nav)
    print(f'[baseline] nearest_free reward médio: {baseline_reward:.2f}')

    def make_monitored():
        env = AllocationEnv(
            num_robots=3, waypoints=WAYPOINT_SETS['expanded'],
            episode_len=30, inter_arrival=15, obs_source='yolo', nav_model=nav)
        return Monitor(env)

    vec_env = make_vec_env(make_monitored, n_envs=8, seed=42)

    model = PPO('MlpPolicy', vec_env, verbose=1, seed=42)
    cb = EpisodeRewardCallback(eval_freq=8192)

    print('[train] rodando 500k timesteps...')
    model.learn(total_timesteps=500_000, callback=cb)

    out_model = os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip')
    model.save(out_model)
    print(f'[done] modelo salvo em {out_model}')
    print(f'[curva] {len(cb.rewards)} pontos coletados')

    if cb.rewards:
        _plot(cb.timesteps, cb.rewards, baseline_reward)
    else:
        print('[erro] nenhum ponto coletado — aumentar episode_len ou n_envs')


def _plot(timesteps, rewards, baseline_reward):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rewards = np.array(rewards)
    window = max(1, len(rewards) // 10)
    if len(rewards) >= window:
        smooth = np.convolve(rewards, np.ones(window) / window, mode='valid')
        ts_smooth = timesteps[window - 1:]
    else:
        smooth, ts_smooth = rewards, timesteps

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(timesteps, rewards, alpha=0.25, color='#7B2D8B', linewidth=0.8)
    ax.plot(ts_smooth, smooth, color='#7B2D8B', linewidth=2,
            label='PPO(yolo) — suavizado')
    ax.axhline(baseline_reward, color='#888', linestyle='--', linewidth=1.5,
               label=f'nearest_free ({baseline_reward:.1f})')
    ax.set_xlabel('Timesteps de treino')
    ax.set_ylabel('Reward médio por episódio')
    ax.set_title('CERISE — Curva de Aprendizado PPO vs Baseline')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()

    out = os.path.join(_REPO, 'docs', 'rl_learning_curve.png')
    fig.savefig(out, dpi=150)
    print(f'[plot] salvo em {out}')


if __name__ == '__main__':
    main()
