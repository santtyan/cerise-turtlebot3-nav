"""Modelo analítico de tempo de navegação (distância -> tempo).

Núcleo da velocidade de treino: substitui a navegação real do Nav2/Gazebo
(15-30s por goal) por uma estimativa aritmética instantânea, permitindo
milhões de timesteps em minutos.

A fidelidade sim->real vem da calibração de `v_nominal` e `noise_std` com os
tempos reais (`latency_s`) coletados em ~/cerise_log.csv ao rodar o
task_allocator atual no Gazebo. Ver `calibrate_from_csv`.
"""

import csv
import math
import os
from dataclasses import dataclass

# Valores default — substituir pelos calibrados via calibrate_from_csv().
# v_nominal em m/s; deduzido da velocidade efetiva porta-a-porta do TurtleBot3
# Waffle no Nav2 (inclui aceleração, planejamento, recuperações).
DEFAULT_V_NOMINAL = 0.12
DEFAULT_NOISE_STD = 0.15   # fração do tempo nominal (ruído multiplicativo)
DEFAULT_FIXED_OVERHEAD = 2.0  # s: custo fixo por goal (planejamento Nav2)


@dataclass
class NavModel:
    """Estima o tempo de navegação de um ponto a outro.

    travel_time(robot_pos, origin, dest) modela a tarefa completa:
    o robô vai da posição atual até a origem da demanda e depois até o destino.
    """

    v_nominal: float = DEFAULT_V_NOMINAL
    noise_std: float = DEFAULT_NOISE_STD
    fixed_overhead: float = DEFAULT_FIXED_OVERHEAD

    def path_distance(self, robot_pos, origin, dest):
        """Distância total percorrida: robô -> origem -> destino (metros)."""
        return _dist(robot_pos, origin) + _dist(origin, dest)

    def travel_time(self, robot_pos, origin, dest, rng=None):
        """Tempo estimado (s) para o robô atender a demanda origin->dest.

        rng: gerador numpy/Random opcional para o ruído. Se None, retorna o
        tempo determinístico (útil para baseline/avaliação reprodutível).
        """
        dist = self.path_distance(robot_pos, origin, dest)
        base = self.fixed_overhead + dist / self.v_nominal
        if rng is None or self.noise_std <= 0.0:
            return base
        # Ruído multiplicativo, truncado para não gerar tempos negativos.
        factor = 1.0 + rng.normalvariate(0.0, self.noise_std) \
            if hasattr(rng, 'normalvariate') else 1.0 + float(rng.normal(0.0, self.noise_std))
        return base * max(0.1, factor)


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def calibrate_from_csv(csv_path=None, waypoints=None):
    """Calibra v_nominal a partir de tempos reais em ~/cerise_log.csv.

    O CSV tem colunas: timestamp, demand_id, origin, dest, robot, latency_s.
    Como não registramos a posição do robô no início de cada tarefa, usamos a
    aproximação origin->dest (a posição inicial costuma estar próxima da origem
    porque o robô anterior terminou perto) e ajustamos v_nominal pela mediana
    da razão distância/tempo.

    waypoints: dict {nome: (x, y)} para mapear origin/dest -> coordenadas.
    Retorna um NavModel calibrado (ou default se o CSV não existir/estiver vazio).
    """
    if csv_path is None:
        csv_path = os.path.expanduser('~/cerise_log.csv')
    if waypoints is None:
        # Mesmos waypoints de demand_generator.py
        waypoints = {
            'A': (0.0, 0.0), 'B': (1.5, 0.0),
            'C': (0.0, -1.5), 'D': (1.5, -1.5),
        }
    if not os.path.exists(csv_path):
        return NavModel()

    speeds = []
    latencies = []
    with open(csv_path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                origin = waypoints[row['origin']]
                dest = waypoints[row['dest']]
                latency = float(row['latency_s'])
            except (KeyError, ValueError):
                continue
            if latency <= 0:
                continue
            d = _dist(origin, dest)
            if d > 0:
                speeds.append(d / latency)
                latencies.append(latency)

    if not speeds:
        return NavModel()

    v = _median(speeds)
    # noise_std = dispersão relativa das latências como proxy da variabilidade.
    mean_lat = sum(latencies) / len(latencies)
    if mean_lat > 0 and len(latencies) > 1:
        var = sum((x - mean_lat) ** 2 for x in latencies) / (len(latencies) - 1)
        noise = min(0.5, math.sqrt(var) / mean_lat)
    else:
        noise = DEFAULT_NOISE_STD
    return NavModel(v_nominal=max(0.02, v), noise_std=noise)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])
