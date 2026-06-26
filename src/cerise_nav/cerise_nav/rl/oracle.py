"""Oráculos de referência para situar baseline e PPO frente ao ótimo.

Sem um teto ótimo, não dá para saber se o gap PPO↔nearest_free é grande ou se
ambos já estão perto do melhor possível. Este módulo fornece dois tetos:

  1. clairvoyant_step_policy — oráculo MÍOPE de 1 passo: a cada demanda escolhe o
     robô que minimiza o RESPONSE_TIME determinístico (espera até liberar +
     travel robô->origem->destino), considerando TODOS os robôs (não só os livres
     como o nearest_free). Revela quanto a restrição "só livres" custa.

  2. make_oracle_policy — oráculo CLARIVIDENTE do episódio: conhece a sequência
     inteira de demandas e resolve por beam search a atribuição que minimiza a
     soma dos response_time (espera + travel) — a MESMA métrica otimizada pelo
     reward do env e reportada no eval. É o limite inferior prático contra o qual
     se mede o optimality gap das políticas.

Ambos reusam `NavModel.travel_time(..., rng=None)` (custo determinístico) e a
mesma dinâmica de `AllocationEnv.step` (espera por robô ocupado + inter_arrival),
para que o plano seja um teto válido sob as MESMAS regras do ambiente.
"""


def clairvoyant_step_policy(obs, env):
    """Escolhe o robô de menor response_time (espera + travel) para a demanda.

    Diferente de nearest_free, considera todos os robôs (inclusive ocupados):
    o custo de um robô ocupado é a espera até ele liberar (busy restante) mais o
    travel da tarefa. Em baixa carga (todos livres) recai no "mais próximo"; sob
    fila, prefere quem combina proximidade e disponibilidade.

    Recebe o env para acessar o destino real (não presente na observação), as
    posições verdadeiras (sem o ruído de percepção da obs) e o busy atual.
    """
    origin, dest = env._demands[env._step_idx]
    positions = env._positions
    busy = env._busy
    return min(
        range(env.num_robots),
        key=lambda r: busy[r] + env._nav_per_robot[r].travel_time(
            positions[r], origin, dest, rng=None),
    )


def make_oracle_policy(beam_width=300):
    """Cria uma política-oráculo clarividente (predict_fn(obs, env)).

    No primeiro step de cada episódio (step_idx == 0) resolve o episódio inteiro
    por beam search e devolve a ação planejada para cada step subsequente. Como o
    plano é fixado no início, o oráculo "se compromete" com uma alocação ótima
    conhecendo todas as demandas de antemão.
    """
    cache = {'plan': None}

    def predict(obs, env):
        if env._step_idx == 0:
            cache['plan'], _ = plan_beam(
                env._demands, env._positions, env._busy, env._nav_per_robot,
                env.num_robots, env.inter_arrival, beam_width)
        return cache['plan'][env._step_idx]

    return predict


def plan_beam(demands, init_positions, init_busy, nav_or_list, num_robots,
              inter_arrival, beam_width=300):
    """Beam search sobre a sequência de demandas; minimiza a soma de response_time.

    Replica a dinâmica de AllocationEnv.step (custo determinístico):
      1. se o robô está ocupado, registra a espera e o relógio avança até liberar;
      2. calcula o travel determinístico robô->origem->destino;
      3. o robô fica ocupado por travel e sua posição passa a ser o destino;
      4. entre demandas, o relógio avança inter_arrival.

    Retorna (lista_de_ações, custo_total). custo_total = soma dos response_time
    (espera + travel), exatamente a métrica reportada no eval.
    """
    # Aceita NavModel único (compat) ou lista por robô (heterogeneidade).
    if isinstance(nav_or_list, list):
        navs = nav_or_list
    else:
        navs = [nav_or_list] * num_robots

    n_steps = len(demands)
    # Cada item do beam: (custo, positions, busy, actions).
    beam = [(
        0.0,
        tuple(tuple(p) for p in init_positions),
        tuple(float(b) for b in init_busy),
        tuple(),
    )]

    for idx, (origin, dest) in enumerate(demands):
        last = idx == n_steps - 1
        cand = []
        for cost, positions, busy, actions in beam:
            for r in range(num_robots):
                b = busy
                wait = b[r] if b[r] > 0.0 else 0.0
                if wait > 0.0:  # robô ocupado: espera liberar
                    b = _advance(b, wait)
                travel = navs[r].travel_time(positions[r], origin, dest, rng=None)
                b = list(b)
                b[r] = travel
                pos = list(positions)
                pos[r] = tuple(dest)
                b = tuple(b)
                if not last:
                    b = _advance(b, inter_arrival)
                # custo = response_time = espera + travel (espelha o reward do env)
                cand.append((cost + wait + travel, tuple(pos), b, actions + (r,)))
        cand.sort(key=lambda x: x[0])
        beam = cand[:beam_width]

    best = min(beam, key=lambda x: x[0])
    return list(best[3]), best[0]


def _advance(busy, dt):
    """Decrementa o tempo de ocupação de todos os robôs em dt (espelha env)."""
    if dt <= 0:
        return busy
    return tuple(max(0.0, b - dt) for b in busy)
