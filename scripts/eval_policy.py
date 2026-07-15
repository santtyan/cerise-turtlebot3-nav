#!/usr/bin/env python3
"""Avalia PPO vs baseline nearest_free no AllocationEnv (env leve).

Roda N episódios com sementes PAREADAS (mesma sequência de demandas para todas
as políticas) e reporta métricas comparáveis para o paper. A métrica PRINCIPAL é
o response_time (espera na fila + travel) — a mesma que o reward do env e o
oráculo otimizam. Sob baixa carga a espera é ~0 e response_time ≈ latência de
viagem; sob alta carga elas divergem e é onde o RL pode superar o guloso.
  - response_time médio e p95 (PRINCIPAL)
  - latência de viagem média e p95 (só travel, secundária)
  - custo acumulado por episódio = soma dos response_time (o que o beam minimiza)
  - balanceamento de carga (desvio do nº de tarefas por robô)
  - taxa de ação inválida

Uso:
    python3 scripts/eval_policy.py --model models/ppo_allocator_yolo.zip --plot
    python3 scripts/eval_policy.py --ablation --plot
"""

import argparse
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))

from cerise_nav.rl.allocation_env import AllocationEnv, WAYPOINT_SETS  # noqa: E402
from cerise_nav.rl import baselines                        # noqa: E402
from cerise_nav.rl import oracle                           # noqa: E402
from cerise_nav.rl.nav_model import calibrate_from_csv     # noqa: E402


def make_env(args, nav, obs_source=None):
    return AllocationEnv(
        num_robots=args.num_robots, waypoints=WAYPOINT_SETS[args.waypoints],
        episode_len=args.episode_len,
        inter_arrival=args.inter_arrival,
        obs_source=obs_source or args.obs_source,
        nav_model=nav, load_balance=not args.no_load_balance)


def eval_policy(predict_fn, args, nav, n_episodes, obs_source=None):
    """Roda predict_fn em n_episodes (sementes 0..n-1) e agrega métricas."""
    env = make_env(args, nav, obs_source)
    travels, responses, ep_costs, imbalances, invalid, total = [], [], [], [], 0, 0
    for ep in range(n_episodes):
        obs, _ = env.reset(seed=ep)
        ep_travels, ep_responses, done = [], [], False
        while not done:
            a = predict_fn(obs, env)
            obs, _, term, trunc, info = env.step(a)
            ep_travels.append(info['travel_time'])
            ep_responses.append(info['response_time'])
            invalid += int(info['invalid'])
            total += 1
            done = term or trunc
        travels.extend(ep_travels)
        responses.extend(ep_responses)
        # custo do episódio = soma dos response_time (espera+travel), exatamente
        # o que o beam do oráculo minimiza. NÃO é makespan de parede.
        ep_costs.append(sum(ep_responses))
        counts = info['task_count']
        mean_c = sum(counts) / len(counts)
        imbalances.append(float(np.std(counts)) / (mean_c + 1.0))
    return {
        'resp_mean': float(np.mean(responses)),
        'resp_p95': float(np.percentile(responses, 95)),
        'lat_mean': float(np.mean(travels)),
        'lat_p95': float(np.percentile(travels, 95)),
        'resp_cost_mean': float(np.mean(ep_costs)),
        'load_imbalance': float(np.mean(imbalances)),
        'invalid_rate': invalid / max(1, total),
        '_travels': travels,
        '_responses': responses,
    }


def main():
    p = argparse.ArgumentParser(description='Avalia PPO vs nearest_free.')
    p.add_argument('--model', help='Caminho do .zip do PPO (opcional).')
    p.add_argument('--ablation', action='store_true',
                   help='Compara PPO(yolo) vs PPO(odom) vs baseline.')
    p.add_argument('--obs-source', choices=['yolo', 'odom', 'none'],
                   default='yolo')
    p.add_argument('--num-robots', type=int, default=3)
    p.add_argument('--waypoints', choices=['default', 'expanded'],
                   default='default')
    p.add_argument('--episode-len', type=int, default=20)
    p.add_argument('--inter-arrival', type=float, default=30.0)
    p.add_argument('--episodes', type=int, default=1000)
    p.add_argument('--beam-width', type=int, default=300,
                   help='Largura do beam search do oráculo clarividente.')
    p.add_argument('--no-load-balance', action='store_true')
    p.add_argument('--plot', action='store_true', help='Gera gráfico comparativo.')
    args = p.parse_args()

    nav = calibrate_from_csv()
    print(f'[nav_model] v_nominal={nav.v_nominal:.3f} m/s\n')

    results = {}

    baseline_fn = lambda obs, env: baselines.nearest_free_policy(obs, env.num_robots)
    random_fn   = lambda obs, env: baselines.random_policy(obs, env.num_robots)
    rr_fn       = baselines.make_round_robin(args.num_robots)

    if args.ablation:
        from stable_baselines3 import PPO
        from sb3_contrib import MaskablePPO
        yolo_path = os.path.join(_REPO, 'models', 'ppo_allocator_yolo.zip')
        odom_path = os.path.join(_REPO, 'models', 'ppo_allocator_odom.zip')
        yolo_masked_path = os.path.join(_REPO, 'models', 'ppo_allocator_yolo_masked.zip')
        odom_masked_path = os.path.join(_REPO, 'models', 'ppo_allocator_odom_masked.zip')

        print('Avaliando random...')
        results['Random'] = eval_policy(random_fn, args, nav, args.episodes, 'yolo')

        print('Avaliando round-robin...')
        results['Round\nRobin'] = eval_policy(
            baselines.make_round_robin(args.num_robots), args, nav, args.episodes, 'yolo')

        print('Avaliando baseline...')
        results['Baseline\n(nearest_free)'] = eval_policy(baseline_fn, args, nav, args.episodes, 'yolo')

        print('Avaliando PPO(yolo)...')
        model_yolo = PPO.load(yolo_path)
        results['PPO\n(YOLO)'] = eval_policy(
            lambda obs, env: int(model_yolo.predict(obs, deterministic=True)[0]),
            args, nav, args.episodes, 'yolo')

        print('Avaliando PPO(odom)...')
        model_odom = PPO.load(odom_path)
        results['PPO\n(odom)'] = eval_policy(
            lambda obs, env: int(model_odom.predict(obs, deterministic=True)[0]),
            args, nav, args.episodes, 'odom')

        # Variantes com action masking (opcionais — só entram se os modelos já
        # foram treinados com `train_ppo.py --masked`). A máscara vem direto do
        # AllocationEnv (mesmo método usado no treino), sem precisar de ActionMasker
        # aqui, já que estamos chamando env.step() manualmente em eval_policy().
        if os.path.exists(yolo_masked_path):
            print('Avaliando PPO(yolo, masked)...')
            model_yolo_masked = MaskablePPO.load(yolo_masked_path)
            results['PPO\n(YOLO, masked)'] = eval_policy(
                lambda obs, env: int(model_yolo_masked.predict(
                    obs, action_masks=env.action_masks(), deterministic=True)[0]),
                args, nav, args.episodes, 'yolo')
        else:
            print(f'[skip] {yolo_masked_path} não encontrado — rode train_ppo.py --obs-source yolo --masked')

        if os.path.exists(odom_masked_path):
            print('Avaliando PPO(odom, masked)...')
            model_odom_masked = MaskablePPO.load(odom_masked_path)
            results['PPO\n(odom, masked)'] = eval_policy(
                lambda obs, env: int(model_odom_masked.predict(
                    obs, action_masks=env.action_masks(), deterministic=True)[0]),
                args, nav, args.episodes, 'odom')
        else:
            print(f'[skip] {odom_masked_path} não encontrado — rode train_ppo.py --obs-source odom --masked')

        print('Avaliando Clairvoyant (oráculo míope)...')
        results['Clairvoyant\n(1 passo)'] = eval_policy(
            oracle.clairvoyant_step_policy, args, nav, args.episodes, 'none')

        print(f'Avaliando Oráculo (beam={args.beam_width})...')
        results['Oráculo\n(clarividente)'] = eval_policy(
            oracle.make_oracle_policy(args.beam_width), args, nav, args.episodes, 'none')
    else:
        print('Avaliando baseline...')
        results['nearest_free'] = eval_policy(baseline_fn, args, nav, args.episodes)

        if args.model:
            is_masked = 'masked' in os.path.basename(args.model)
            if is_masked:
                from sb3_contrib import MaskablePPO
                model = MaskablePPO.load(args.model)
                predict_fn = lambda obs, env: int(model.predict(
                    obs, action_masks=env.action_masks(), deterministic=True)[0])
            else:
                from stable_baselines3 import PPO
                model = PPO.load(args.model)
                predict_fn = lambda obs, env: int(model.predict(obs, deterministic=True)[0])
            obs_kind = 'yolo' if 'yolo' in args.model else 'odom'
            label = f'PPO({obs_kind}, masked)' if is_masked else f'PPO({obs_kind})'
            print(f'Avaliando {label}...')
            results[label] = eval_policy(predict_fn, args, nav, args.episodes)

    _print_table(results)
    if args.plot:
        _plot(results, ablation=args.ablation)


def _print_table(results):
    cols = ['resp_mean', 'resp_p95', 'lat_mean', 'lat_p95', 'resp_cost_mean', 'load_imbalance', 'invalid_rate']
    # Optimality gap (%) relativo ao custo de response_time do oráculo, se presente.
    oracle_key = next((k for k in results if 'Oráculo' in k), None)
    oracle_cost = results[oracle_key]['resp_cost_mean'] if oracle_key else None

    header = ['política'] + cols + (['gap_%'] if oracle_cost else [])
    print('  '.join(f'{h:>16s}' for h in header))
    for name, m in results.items():
        row = [name.replace('\n', ' ')] + [f'{m[c]:.3f}' for c in cols]
        if oracle_cost:
            gap = 100.0 * (m['resp_cost_mean'] - oracle_cost) / oracle_cost
            row.append(f'{gap:+.1f}')
        print('  '.join(f'{v:>16s}' for v in row))


def _plot(results, ablation=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    names = list(results.keys())
    _palette = ['#AAAAAA', '#E57373', '#FF8C00', '#888888', '#7B2D8B',
                '#2D7B8B', '#C2783F', '#3FA34D']
    colors = (_palette * 2)[:len(names)]

    metrics = ['resp_mean', 'resp_p95', 'resp_cost_mean', 'load_imbalance']
    labels = ['Response time médio (s)', 'Response time p95 (s)',
              'Custo resp. acum. (s)', 'Desbalanceamento']

    fig, axes = plt.subplots(1, len(metrics), figsize=(4 * len(metrics), 5.5))
    for ax, met, lab in zip(axes, metrics, labels):
        vals = [results[n][met] for n in names]
        bars = ax.bar(names, vals, color=colors)
        for bar, val in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.01 * max(vals),
                    f'{val:.2f}', ha='center', va='bottom', fontsize=9, fontweight='bold')
        ax.set_title(lab, fontsize=11)
        ax.set_ylim(0, max(vals) * 1.2)
        ax.grid(axis='y', alpha=0.3)
        ax.set_xticklabels(names, rotation=45, ha='right', fontsize=10)
        ax.tick_params(axis='x', labelsize=10)

    title = 'CERISE — PPO(YOLO) vs PPO(odom) vs Baseline Ablation' if ablation \
        else 'CERISE — PPO vs nearest_free (light env, 1000 episodes)'
    fig.suptitle(title, fontsize=13, fontweight='bold')
    fig.tight_layout()

    fname = 'rl_ablation.png' if ablation else 'rl_eval_comparison.png'
    out = os.path.join(_REPO, 'docs', fname)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\n[plot] salvo em {out}')

    # Boxplot de distribuição de latências
    _plot_latency_dist(results, ablation)


def _plot_latency_dist(results, ablation=False):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    names = list(results.keys())
    _palette = ['#AAAAAA', '#E57373', '#FF8C00', '#888888', '#7B2D8B',
                '#2D7B8B', '#C2783F', '#3FA34D']
    colors = (_palette * 2)[:len(names)]
    data = [results[n]['_responses'] for n in names]

    fig, ax = plt.subplots(figsize=(9, 5))
    bp = ax.boxplot(data, patch_artist=True, notch=False,
                    medianprops=dict(color='white', linewidth=2))
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.8)

    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=10)
    ax.set_ylabel('Response time per task (s)', fontsize=11)
    ax.set_title('CERISE — Response Time Distribution by Policy', fontsize=12, fontweight='bold')
    ax.grid(axis='y', alpha=0.3)

    fname = 'rl_latency_boxplot_ablation.png' if ablation else 'rl_latency_boxplot.png'
    out = os.path.join(_REPO, 'docs', fname)
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print(f'[plot] salvo em {out}')


if __name__ == '__main__':
    main()
