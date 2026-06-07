"""Políticas-baseline para comparação com o agente RL.

`nearest_free_policy` replica EXATAMENTE a lógica de `nearest_free()` do
task_allocator.py (linhas 56-61): entre os robôs livres, escolhe o mais próximo
da origem da demanda. É função pura sobre a observação codificada, para ser
avaliada no MESMO AllocationEnv que o PPO, garantindo comparação justa.
"""

import math

from . import obs_encoding


def nearest_free_policy(obs, num_robots):
    """Escolhe o robô livre mais próximo da origem da demanda.

    obs: vetor de observação no layout de obs_encoding.
    num_robots: N.
    Retorna o índice do robô (int), espelhando nearest_free().

    Empate/ocupação: considera livre quem tem busy_remaining ~ 0. Se todos
    ocupados (situação que o env trata com espera), retorna o de menor
    busy_remaining como melhor esforço — no env, alocar ocupado dispara a
    penalização e a espera, igual ao comportamento real de "todos ocupados".
    """
    positions, busy, origin = obs_encoding.decode_obs(obs, num_robots)

    free = [r for r in range(num_robots) if busy[r] <= _FREE_EPS]
    if free:
        return min(free, key=lambda r: _dist(positions[r], origin))

    # Todos ocupados: escolhe quem libera primeiro (menor busy_remaining).
    return min(range(num_robots), key=lambda r: busy[r])


# busy normalizado; ~0 significa livre. Tolerância pequena para ruído numérico.
_FREE_EPS = 1e-3


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
