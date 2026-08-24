"""Políticas-baseline para comparação com o agente RL.

Hierarquia esperada de desempenho (custo decrescente):
  random < round_robin < nearest_free < PPO < oráculo

Todas são funções puras sobre a observação codificada (obs_encoding), avaliadas
no MESMO AllocationEnv que o PPO, garantindo comparação justa.
"""

import math

from . import obs_encoding

# busy normalizado; ~0 significa livre. Tolerância para ruído numérico.
_FREE_EPS = 1e-3

# Estado mutable de round-robin (compartilhado via closure em make_round_robin).
# Não usar diretamente — usar make_round_robin() para criar instância isolada.
_RR_COUNTER = [0]


def nearest_free_policy(obs, num_robots):
    """Escolhe o robô livre mais próximo da origem da demanda.

    Míope por design: ignora destino e demandas futuras — é o que o PPO supera.
    Se todos ocupados, escolhe quem libera primeiro.

    Mesmo núcleo de seleção que task_allocator.py:TaskAllocator.nearest_free
    (nó ROS2 de produção), não compartilhado: aqui opera sobre observação
    vetorizada do Gymnasium e sempre retorna alguém (o AllocationEnv não
    modela "aguardar"); lá opera sobre odometria real via rclpy e retorna
    None se ninguém livre. Se o critério de desempate mudar, replicar
    manualmente nos dois.
    """
    positions, busy, origin, _dest, _future = obs_encoding.decode_obs(obs, num_robots)
    free = [r for r in range(num_robots) if busy[r] <= _FREE_EPS]
    if free:
        return min(free, key=lambda r: _dist(positions[r], origin))
    return min(range(num_robots), key=lambda r: busy[r])


def random_policy(obs, num_robots, rng=None):
    """Aloca para um robô uniformemente aleatório.

    Lower bound do desempenho: não usa nenhuma informação de estado.
    rng: np.random.Generator ou None (usa random do Python).
    """
    import random as _random
    if rng is not None:
        return int(rng.integers(0, num_robots))
    return _random.randrange(num_robots)


def make_round_robin(num_robots):
    """Retorna uma política round-robin com contador isolado.

    Aloca demandas em sequência circular (0, 1, 2, 0, 1, 2, ...).
    Não usa nenhuma informação de posição ou carga — baseline estrutural.
    Chamar make_round_robin() para cada avaliação para resetar o contador.
    """
    state = [0]

    def policy(obs, _env_or_n=None):
        r = state[0] % num_robots
        state[0] += 1
        return r

    return policy


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])
