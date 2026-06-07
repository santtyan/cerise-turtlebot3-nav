"""Codificação compartilhada da observação (estado) do MDP de alocação.

Este módulo é a ÚNICA fonte de verdade sobre a ordem e a escala dos campos da
observação. É usado por DOIS caminhos que precisam ser idênticos:

  1. AllocationEnv (treino offline, env leve)
  2. rl_task_allocator (nó ROS2, inferência com posições do YOLO)

Se os dois montarem a observação de forma diferente, a política treinada recebe
lixo em produção. Por isso encode_obs() é a única função que ambos chamam, e
o teste de paridade compara os dois caminhos.

Layout da observação (vetor float32 de tamanho 3*N + 2):
    [ r0_x, r0_y, r0_busy,        # robô 0
      r1_x, r1_y, r1_busy,        # robô 1
      ...                         # ... N robôs
      demand_origin_x, demand_origin_y ]

- coordenadas em metros, espaço esperado ~[-COORD_BOUND, COORD_BOUND]
- busy = tempo restante de ocupação normalizado por BUSY_REF (0 = livre)
"""

import numpy as np

# Limites do espaço de observação. A arena do CERISE fica em ~[-1.5, 1.5];
# COORD_BOUND com folga acomoda jitter e o cenário ampliado.
COORD_BOUND = 3.0
# Tempo de referência (s) para normalizar busy_remaining para ~[0, 1].
BUSY_REF = 60.0


def obs_dim(num_robots):
    """Tamanho do vetor de observação para N robôs."""
    return 3 * num_robots + 2


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


def encode_obs(robot_positions, robot_busy_remaining, demand_origin):
    """Monta o vetor de observação a partir do estado bruto.

    robot_positions: sequência de (x, y) por robô, em metros.
    robot_busy_remaining: sequência de tempo restante de ocupação (s) por robô.
    demand_origin: (x, y) da origem da demanda corrente, em metros.

    Retorna np.ndarray float32 de tamanho 3*N + 2. Coordenadas são recortadas a
    [-COORD_BOUND, COORD_BOUND] e busy normalizado/recortado a [0, 1].
    """
    n = len(robot_positions)
    assert len(robot_busy_remaining) == n, 'positions e busy com tamanhos diferentes'
    out = np.empty(obs_dim(n), dtype=np.float32)
    for r in range(n):
        x, y = robot_positions[r]
        out[3 * r + 0] = _clip_coord(x)
        out[3 * r + 1] = _clip_coord(y)
        out[3 * r + 2] = _clip01(robot_busy_remaining[r] / BUSY_REF)
    out[3 * n + 0] = _clip_coord(demand_origin[0])
    out[3 * n + 1] = _clip_coord(demand_origin[1])
    return out


def decode_obs(obs, num_robots):
    """Inverso de encode_obs (para testes/round-trip).

    Retorna (robot_positions, robot_busy_remaining_normalizado, demand_origin).
    Nota: busy é retornado normalizado (não em segundos), pois encode descarta
    a escala original ao normalizar.
    """
    positions = []
    busy = []
    for r in range(num_robots):
        positions.append((float(obs[3 * r + 0]), float(obs[3 * r + 1])))
        busy.append(float(obs[3 * r + 2]))
    origin = (float(obs[3 * num_robots + 0]), float(obs[3 * num_robots + 1]))
    return positions, busy, origin


def _clip_coord(v):
    return float(np.clip(v, -COORD_BOUND, COORD_BOUND))


def _clip01(v):
    return float(np.clip(v, 0.0, 1.0))
