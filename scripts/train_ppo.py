#!/usr/bin/env python3
"""Treina um agente PPO no AllocationEnv (env leve, sem Gazebo).

Para a ablação YOLO vs odometria, use --obs-source:
    python3 scripts/train_ppo.py --obs-source yolo
    python3 scripts/train_ppo.py --obs-source odom

Salva o modelo em models/ppo_allocator_<obs_source>.zip e loga curvas no
TensorBoard (runs/). A calibração do tempo de navegação vem de ~/cerise_log.csv
se existir (senão usa os defaults do NavModel).
"""

import argparse
import os
import sys

# Permite importar cerise_nav.rl sem instalar o pacote (rodar do repo).
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from stable_baselines3 import PPO            # noqa: E402
from stable_baselines3.common.env_util import make_vec_env  # noqa: E402

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv      # noqa: E402


def _tensorboard_available():
    try:
        import tensorboard  # noqa: F401
        return True
    except ImportError:
        return False


def build_env_kwargs(args, nav):
    return dict(
        num_robots=args.num_robots,
        waypoints=WAYPOINT_SETS[args.waypoints],
        episode_len=args.episode_len,
        inter_arrival=args.inter_arrival,
        obs_source=args.obs_source,
        nav_model=nav,
        load_balance=not args.no_load_balance,
    )


def main():
    p = argparse.ArgumentParser(description='Treina PPO para alocação CERISE.')
    p.add_argument('--obs-source', choices=['yolo', 'odom', 'none'],
                   default='yolo', help='Fonte do estado de posição (ablação).')
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'],
                   default='default', help='Conjunto de waypoints da arena.')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--inter-arrival', type=float, default=30.0)
    p.add_argument('--timesteps', type=int, default=300_000)
    p.add_argument('--n-envs', type=int, default=8)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--no-load-balance', action='store_true')
    p.add_argument('--out-dir', default=os.path.join(_REPO, 'models'))
    p.add_argument('--tensorboard', default=os.path.join(_REPO, 'runs'))
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Calibra o modelo de navegação com os tempos reais já coletados.
    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s  '
          f'noise_std={nav.noise_std:.3f}')

    env_kwargs = build_env_kwargs(args, nav)
    vec_env = make_vec_env(AllocationEnv, n_envs=args.n_envs,
                           seed=args.seed, env_kwargs=env_kwargs)

    # TensorBoard é opcional: se não estiver instalado, treina sem ele.
    tb_log = args.tensorboard if _tensorboard_available() else None
    if tb_log is None:
        print('[train] tensorboard indisponível — logando só no stdout.')

    model = PPO('MlpPolicy', vec_env, verbose=1, seed=args.seed,
                tensorboard_log=tb_log)

    run_name = f'ppo_allocator_{args.obs_source}'
    print(f'[train] {run_name}: {args.timesteps} timesteps, '
          f'{args.num_robots} robôs, obs={args.obs_source}')
    model.learn(total_timesteps=args.timesteps, tb_log_name=run_name)

    out_path = os.path.join(args.out_dir, f'{run_name}.zip')
    model.save(out_path)
    print(f'[done] modelo salvo em {out_path}')


if __name__ == '__main__':
    main()
