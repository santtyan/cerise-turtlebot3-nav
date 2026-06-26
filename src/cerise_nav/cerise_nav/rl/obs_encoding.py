"""Codificação compartilhada da observação (estado) do MDP de alocação.

Este módulo é a ÚNICA fonte de verdade sobre a ordem e a escala dos campos da
observação. É usado por DOIS caminhos que precisam ser idênticos:

  1. AllocationEnv (treino offline, env leve)
  2. rl_task_allocator (nó ROS2, inferência com posições do YOLO)

Se os dois montarem a observação de forma diferente, a política treinada recebe
lixo em produção. Por isso encode_obs() é a única função que ambos chamam, e
o teste de paridade compara os dois caminhos.

Layout da observação (vetor float32 de tamanho 3*N + 4 + 4*LOOKAHEAD):
    [ r0_x, r0_y, r0_busy,                 # robô 0
      r1_x, r1_y, r1_busy,                 # robô 1
      ...                                  # ... N robôs
      cur_origin_x, cur_origin_y,          # demanda corrente: origem
      cur_dest_x,   cur_dest_y,            # demanda corrente: destino
      f0_origin_x, f0_origin_y, f0_dest_x, f0_dest_y,   # próxima demanda
      ... ]                                # ... LOOKAHEAD demandas futuras

Por que destino + lookahead: sem o destino a política nem vê o que define o
tempo de viagem; sem as próximas demandas ela não consegue ANTECIPAR (segurar um
robô para a demanda seguinte), que é a única forma de superar a regra fixa
nearest_free sob alta carga. Slots de demanda futura inexistentes (fim do
episódio / fila curta) recebem PAD_VALUE, fora da arena e distinguível.

- coordenadas em metros, espaço esperado ~[-COORD_BOUND, COORD_BOUND]
- busy = tempo restante de ocupação normalizado por BUSY_REF (0 = livre)
"""

import numpy as np

# Limites do espaço de observação. A arena do CERISE fica em ~[-1.5, 1.5];
# COORD_BOUND com folga acomoda jitter e o cenário ampliado.
COORD_BOUND = 3.0
# Tempo de referência (s) para normalizar busy_remaining para ~[0, 1].
BUSY_REF = 60.0
# Quantas demandas futuras (além da corrente) entram na observação.
LOOKAHEAD = 2
# Valor de preenchimento para slots de demanda futura inexistentes. Fica no
# limite da arena (fora dos waypoints reais ~[-0.55, 0.55]) para a política
# aprender "não há demanda aqui".
PAD_VALUE = COORD_BOUND


def obs_dim(num_robots):
    """Tamanho do vetor de observação para N robôs."""
    return 3 * num_robots + 4 + 4 * LOOKAHEAD


def obs_bounds(num_robots):
    """Retorna (low, high) como arrays float32 para o Box do Gymnasium."""
    dim = obs_dim(num_robots)
    low = np.full(dim, -COORD_BOUND, dtype=np.float32)
    high = np.full(dim, COORD_BOUND, dtype=np.float32)
    # Campos busy_remaining são >= 0; manter low=0 nesses índices.
    for r in range(num_robots):
        low[3 * r + 2] = 0.0
        high[3 * r + 2] = 1.0
    return low, high


def encode_obs(robot_positions, robot_busy_remaining, demand_origin,
               demand_dest, future_demands=()):
    """Monta o vetor de observação a partir do estado bruto.

    robot_positions: sequência de (x, y) por robô, em metros.
    robot_busy_remaining: sequência de tempo restante de ocupação (s) por robô.
    demand_origin: (x, y) da origem da demanda corrente, em metros.
    demand_dest: (x, y) do destino da demanda corrente, em metros.
    future_demands: sequência de ((ox, oy), (dx, dy)) das próximas demandas;
        truncada/preenchida a LOOKAHEAD.

    Retorna np.ndarray float32 de tamanho obs_dim(N). Coordenadas são recortadas
    a [-COORD_BOUND, COORD_BOUND] e busy normalizado/recortado a [0, 1].
    """
    n = len(robot_positions)
    assert len(robot_busy_remaining) == n, 'positions e busy com tamanhos diferentes'
    out = np.empty(obs_dim(n), dtype=np.float32)
    for r in range(n):
        x, y = robot_positions[r]
        out[3 * r + 0] = _clip_coord(x)
        out[3 * r + 1] = _clip_coord(y)
        out[3 * r + 2] = _clip01(robot_busy_remaining[r] / BUSY_REF)

    base = 3 * n
    out[base + 0] = _clip_coord(demand_origin[0])
    out[base + 1] = _clip_coord(demand_origin[1])
    out[base + 2] = _clip_coord(demand_dest[0])
    out[base + 3] = _clip_coord(demand_dest[1])

    fut = list(future_demands)[:LOOKAHEAD]
    for k in range(LOOKAHEAD):
        off = base + 4 + 4 * k
        if k < len(fut):
            (ox, oy), (dx, dy) = fut[k]
            out[off + 0] = _clip_coord(ox)
            out[off + 1] = _clip_coord(oy)
            out[off + 2] = _clip_coord(dx)
            out[off + 3] = _clip_coord(dy)
        else:
            out[off:off + 4] = PAD_VALUE
    return out


def decode_obs(obs, num_robots):
    """Inverso de encode_obs (para testes/round-trip).

    Retorna (robot_positions, robot_busy_remaining_normalizado, demand_origin,
    demand_dest, future_demands). future_demands é a lista de ((ox,oy),(dx,dy))
    de tamanho LOOKAHEAD (slots vazios vêm como PAD_VALUE).
    Nota: busy é retornado normalizado (não em segundos), pois encode descarta
    a escala original ao normalizar.
    """
    positions = []
    busy = []
    for r in range(num_robots):
        positions.append((float(obs[3 * r + 0]), float(obs[3 * r + 1])))
        busy.append(float(obs[3 * r + 2]))
    base = 3 * num_robots
    origin = (float(obs[base + 0]), float(obs[base + 1]))
    dest = (float(obs[base + 2]), float(obs[base + 3]))
    future = []
    for k in range(LOOKAHEAD):
        off = base + 4 + 4 * k
        future.append((
            (float(obs[off + 0]), float(obs[off + 1])),
            (float(obs[off + 2]), float(obs[off + 3])),
        ))
    return positions, busy, origin, dest, future


def _clip_coord(v):
    return float(np.clip(v, -COORD_BOUND, COORD_BOUND))


def _clip01(v):
    return float(np.clip(v, 0.0, 1.0))
