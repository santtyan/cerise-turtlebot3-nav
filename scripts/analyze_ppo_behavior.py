#!/usr/bin/env python3
"""Análise qualitativa do comportamento aprendido pelo PPO.

Encontra episódios onde PPO e nearest_free tomam decisões DIFERENTES e
compara o resultado (travel_time). Gera exemplos concretos para o paper.
"""

import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from stable_baselines3 import PPO

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS
from cerise_nav.rl import baselines, obs_encoding
from cerise_nav.rl.nav_model import calibrate_from_csv

ROBOTS = ['robot1', 'robot2', 'robot3']
WP_NAMES = ['A', 'B', 'C', 'D', 'E', 'F']


def analyze(n_episodes=2000, max_examples=5):
    nav = calibrate_from_csv()
    model = PPO.load(os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip'))

    env = AllocationEnv(
        num_robots=3, waypoints=WAYPOINT_SETS['expanded'],
        episode_len=30, inter_arrival=15, obs_source='none', nav_model=nav)

    ppo_wins, nf_wins, ties = 0, 0, 0
    examples_ppo_better = []
    examples_nf_better = []

    for ep in range(n_episodes):
        obs, _ = env.reset(seed=ep)
        done = False
        step = 0
        while not done:
            a_ppo = int(model.predict(obs, deterministic=True)[0])
            a_nf = baselines.nearest_free_policy(obs, env.num_robots)

            if a_ppo != a_nf:
                # Posições e busy antes do step
                positions, busy_norm, origin = obs_encoding.decode_obs(
                    obs, env.num_robots)
                busy_s = [b * obs_encoding.BUSY_REF for b in busy_norm]

                # Simula os dois e compara travel_time
                t_ppo = nav.travel_time(positions[a_ppo], origin,
                                        env._demands[env._step_idx][1])
                t_nf = nav.travel_time(positions[a_nf], origin,
                                       env._demands[env._step_idx][1])

                dest = env._demands[env._step_idx][1]
                example = {
                    'ep': ep, 'step': step,
                    'origin': origin, 'dest': dest,
                    'positions': positions,
                    'busy_s': busy_s,
                    'a_ppo': a_ppo, 'a_nf': a_nf,
                    't_ppo': t_ppo, 't_nf': t_nf,
                    'gain': t_nf - t_ppo,
                }
                if t_ppo < t_nf:
                    ppo_wins += 1
                    if len(examples_ppo_better) < max_examples:
                        examples_ppo_better.append(example)
                elif t_nf < t_ppo:
                    nf_wins += 1
                    if len(examples_nf_better) < max_examples:
                        examples_nf_better.append(example)
                else:
                    ties += 1

            obs, _, term, trunc, _ = env.step(a_ppo)
            done = term or trunc
            step += 1

    total = ppo_wins + nf_wins + ties
    print(f'\n=== Análise de {n_episodes} episódios ===')
    print(f'Decisões diferentes: {total}')
    print(f'  PPO melhor:   {ppo_wins:5d} ({100*ppo_wins/max(1,total):.1f}%)')
    print(f'  NF melhor:    {nf_wins:5d} ({100*nf_wins/max(1,total):.1f}%)')
    print(f'  Empate:       {ties:5d} ({100*ties/max(1,total):.1f}%)')

    print(f'\n--- Exemplos onde PPO ganhou ---')
    _print_examples(examples_ppo_better, winner='PPO')

    print(f'\n--- Exemplos onde nearest_free ganhou ---')
    _print_examples(examples_nf_better, winner='NF')


def _fmt_pos(p):
    return f'({p[0]:+.2f},{p[1]:+.2f})'


def _print_examples(examples, winner):
    for i, e in enumerate(examples):
        print(f'\nExemplo {i+1} (ep={e["ep"]}, step={e["step"]}):')
        for r in range(len(e['positions'])):
            busy = e['busy_s'][r]
            status = f'ocupado {busy:.0f}s' if busy > 0.1 else 'livre'
            print(f'  robot{r+1}: pos={_fmt_pos(e["positions"][r])}  {status}')
        print(f'  Demanda: origem={_fmt_pos(e["origin"])} → dest={_fmt_pos(e["dest"])}')
        print(f'  PPO escolheu robot{e["a_ppo"]+1}  → travel={e["t_ppo"]:.1f}s')
        print(f'  NF  escolheu robot{e["a_nf"]+1}  → travel={e["t_nf"]:.1f}s')
        gain = abs(e['gain'])
        print(f'  {"PPO" if winner == "PPO" else "NF"} ganhou {gain:.1f}s ({100*gain/max(0.1,e["t_nf"]):.1f}%)')


if __name__ == '__main__':
    analyze()
