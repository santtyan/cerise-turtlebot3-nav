"""Ambiente Gymnasium leve para alocação de tarefas multi-robô (CERISE).

MDP: a cada demanda (origem -> destino), o agente escolhe qual robô a executa.
Os robôs ficam ocupados pelo tempo de navegação estimado por `NavModel`. O
episódio termina após K demandas. O objetivo é minimizar a latência acumulada
(e, opcionalmente, balancear a carga entre robôs).

É um ambiente ANALÍTICO (sem ROS/Gazebo): cada step é aritmética pura, o que
permite treinar PPO por milhões de timesteps em minutos. A política treinada é
depois transferida para o nó ROS2 `rl_task_allocator` (mesma codificação de
observação via obs_encoding).

Parametrizado por `num_robots` e `waypoints` para suportar o cenário ampliado
(3-4 robôs, 6-8 waypoints) usado no paper.
"""

import math
import random

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from . import obs_encoding
from .nav_model import NavModel

# Waypoints default (mesmos de demand_generator.py). O cenário ampliado passa
# uma lista maior via construtor.
# Waypoints nos espaços livres entre os cilindros do turtlebot3_world
# (grade em {-1.1,0,1.1}^2). Alinhados com demand_generator.py.
DEFAULT_WAYPOINTS = {
    'A': (-0.55,  0.55), 'B': ( 0.55,  0.55),
    'C': (-0.55, -0.55), 'D': ( 0.55, -0.55),
}

# Cenário ampliado de alta carga (6 waypoints) — adiciona mid-N e mid-S.
EXPANDED_WAYPOINTS = {
    'A': (-0.55,  0.55), 'B': ( 0.55,  0.55),
    'C': (-0.55, -0.55), 'D': ( 0.55, -0.55),
    'E': ( 0.00,  0.55), 'F': ( 0.00, -0.55),
}

# Registry para seleção por nome via CLI dos scripts.
WAYPOINT_SETS = {
    'default': DEFAULT_WAYPOINTS,
    'expanded': EXPANDED_WAYPOINTS,
}

# Penalidade (em unidades de reward) por escolher um robô ocupado.
INVALID_ACTION_PENALTY = 1.0
# Peso do termo de balanceamento de carga no reward (0 = desliga).
LOAD_BALANCE_WEIGHT = 0.3

# Perfis de ruído de posição para a ablação YOLO vs odometria.
# A observação vista pela política recebe esse ruído; a dinâmica real (tempo de
# navegação) usa a posição verdadeira. Modela a diferença entre estimar a
# posição via gêmeo digital YOLO (erro fixo pequeno ~4,7cm) e via odometria
# (drift que cresce com o nº de tarefas executadas pelo robô).
#   'yolo': ruído gaussiano fixo de POS_NOISE_YOLO metros.
#   'odom': drift acumulado = POS_DRIFT_ODOM metros por tarefa concluída.
#   'none': sem ruído (posição perfeita; útil para sanidade).
POS_NOISE_YOLO = 0.047   # erro médio medido do gêmeo digital (4,7 cm)
POS_DRIFT_ODOM = 0.03    # drift por tarefa (cresce sem limite ao longo do ep.)


class AllocationEnv(gym.Env):
    """Ambiente de alocação sequencial de tarefas.

    Observation: Box(3*N + 4 + 4*LOOKAHEAD) via obs_encoding (posições, busy, demanda atual, lookahead).
    Action: Discrete(N) — qual robô executa a demanda corrente.
    Reward: -((wait_time + travel_time) / t_ref) [- penalidade ação inválida] [+ balanceamento].
    """

    metadata = {'render_modes': []}

    def __init__(self, num_robots=3, waypoints=None, episode_len=20,
                 nav_model=None, t_ref=None, load_balance=True,
                 inter_arrival=30.0, obs_source='yolo', seed=None,
                 nav_models=None):
        super().__init__()
        self.num_robots = int(num_robots)
        self.waypoints = dict(waypoints) if waypoints else dict(DEFAULT_WAYPOINTS)
        self._wp_names = list(self.waypoints.keys())
        assert len(self._wp_names) >= 2, 'precisa de >= 2 waypoints'
        self.episode_len = int(episode_len)
        # nav_models: lista de NavModel por robô (heterogeneidade de velocidade).
        # Se None, usa nav_model (ou default) para todos os robôs.
        if nav_models is not None:
            assert len(nav_models) == self.num_robots, 'nav_models deve ter num_robots elementos'
            self._nav_per_robot = list(nav_models)
        else:
            base_nav = nav_model if nav_model is not None else NavModel()
            self._nav_per_robot = [base_nav] * self.num_robots
        # Compat: self.nav aponta para o primeiro modelo (scripts legados usam t_ref via _estimate_t_ref).
        self.nav = self._nav_per_robot[0]
        self.load_balance = load_balance
        # Fonte do estado de posição para a ablação: 'yolo', 'odom' ou 'none'.
        assert obs_source in ('yolo', 'odom', 'none'), obs_source
        self.obs_source = obs_source
        # Intervalo entre chegadas de demandas (s). Modela o timer de 30s do
        # demand_generator: entre demandas, os robôs progridem e podem liberar.
        # Sem isso a fila satura e nenhuma política se distingue.
        self.inter_arrival = float(inter_arrival)

        # t_ref normaliza o reward; default = tempo médio de uma tarefa típica.
        self.t_ref = float(t_ref) if t_ref else self._estimate_t_ref()

        low, high = obs_encoding.obs_bounds(self.num_robots)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space = spaces.Discrete(self.num_robots)

        self._py_rng = random.Random(seed)
        self._np_rng = np.random.default_rng(seed)

        # Estado, definido em reset().
        self._positions = None        # list[(x, y)] posição atual de cada robô
        self._busy = None             # list[float] tempo restante de ocupação (s)
        self._task_count = None       # nº de tarefas já atribuídas a cada robô
        self._idle_time = None        # tempo total livre acumulado por robô (s)
        self._demands = None          # sequência de (origin_xy, dest_xy)
        self._step_idx = None

    # ------------------------------------------------------------------ API
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._py_rng = random.Random(seed)
            self._np_rng = np.random.default_rng(seed)

        # Posiciona robôs em waypoints aleatórios; zera ocupação e contadores.
        start_names = [self._py_rng.choice(self._wp_names)
                       for _ in range(self.num_robots)]
        self._positions = [self.waypoints[n] for n in start_names]
        self._busy = [0.0] * self.num_robots
        self._task_count = [0] * self.num_robots
        self._idle_time = [0.0] * self.num_robots

        # Gera a sequência de demandas com a MESMA lógica do demand_generator:
        # random.sample de 2 waypoints distintos.
        self._demands = []
        for _ in range(self.episode_len):
            o, d = self._py_rng.sample(self._wp_names, 2)
            self._demands.append((self.waypoints[o], self.waypoints[d]))
        self._step_idx = 0

        return self._build_obs(), {}

    def step(self, action):
        action = int(action)
        origin_xy, dest_xy = self._demands[self._step_idx]

        # O estado (busy) já reflete o tempo decorrido até esta demanda: o
        # avanço de inter_arrival é aplicado ao FIM do step anterior, de modo
        # que a observação vista pela política é coerente com a decisão.
        invalid = self._busy[action] > 0.0
        reward = 0.0
        wait_time = 0.0

        if invalid:
            # Robô ocupado: penaliza e avança o relógio até ele liberar.
            reward -= INVALID_ACTION_PENALTY
            wait_time = self._busy[action]
            self._advance_clock(wait_time)

        # Tempo de navegação da tarefa (robô -> origem -> destino).
        # Usa o modelo de navegação específico do robô (suporta heterogeneidade).
        travel = self._nav_per_robot[action].travel_time(
            self._positions[action], origin_xy, dest_xy, rng=self._np_rng)

        # Atualiza estado do robô escolhido.
        self._busy[action] = travel
        self._positions[action] = dest_xy
        self._task_count[action] += 1

        # Objetivo = response_time (espera + travel), não apenas travel: sob
        # alta carga a espera na fila é o que distingue as políticas, e é a
        # métrica contra a qual o oráculo é avaliado. wait_time já foi computado
        # acima (tempo que o robô escolhido ficou ocupado; 0 se estava livre).
        reward += -((wait_time + travel) / self.t_ref)

        if self.load_balance:
            reward -= LOAD_BALANCE_WEIGHT * self._load_imbalance()

        self._step_idx += 1
        terminated = self._step_idx >= self.episode_len
        truncated = False

        # Prepara a próxima demanda: avança o relógio pelo intervalo entre
        # chegadas, liberando robôs que terminaram nesse meio-tempo.
        if not terminated:
            self._advance_clock(self.inter_arrival)

        info = {
            'robot': action,
            'travel_time': travel,
            'wait_time': wait_time,
            'response_time': wait_time + travel,
            'invalid': invalid,
            'task_count': list(self._task_count),
            # Per-robot logging — vira figura de balanceamento no paper.
            'idle_time_per_robot': list(self._idle_time),
            'task_count_per_robot': list(self._task_count),
        }

        # Próxima observação (ou a última repetida se terminou).
        obs = self._build_obs() if not terminated else self._build_obs(last=True)
        return obs, float(reward), terminated, truncated, info

    def action_masks(self):
        """Máscara de ações válidas (True = robô livre, pode ser escolhido).

        Mesmo limiar estrito (`> 0.0`) usado no check de `invalid` em `step()`,
        para que a máscara seja semanticamente idêntica à restrição hoje imposta
        via penalidade suave. Consumida por sb3-contrib's MaskablePPO/ActionMasker.
        """
        return [b <= 0.0 for b in self._busy]

    # -------------------------------------------------------------- helpers
    def _build_obs(self, last=False):
        idx = min(self._step_idx, self.episode_len - 1)
        origin_xy, dest_xy = self._demands[idx]
        # Janela de lookahead: as próximas demandas já conhecidas do episódio.
        future = self._demands[idx + 1: idx + 1 + obs_encoding.LOOKAHEAD]
        noisy_pos = self._observed_positions()
        return obs_encoding.encode_obs(
            noisy_pos, self._busy, origin_xy, dest_xy, future)

    def _observed_positions(self):
        """Posições COMO O AGENTE AS VÊ (com ruído do perfil obs_source).

        A posição verdadeira (self._positions) governa a dinâmica; aqui só
        modelamos o erro de percepção que entra na decisão — base da ablação.
        """
        if self.obs_source == 'none':
            return list(self._positions)
        out = []
        for r in range(self.num_robots):
            x, y = self._positions[r]
            if self.obs_source == 'yolo':
                sx = self._np_rng.normal(0.0, POS_NOISE_YOLO)
                sy = self._np_rng.normal(0.0, POS_NOISE_YOLO)
            else:  # 'odom': drift cresce com o nº de tarefas do robô
                scale = POS_DRIFT_ODOM * self._task_count[r]
                sx = self._np_rng.normal(0.0, scale) if scale > 0 else 0.0
                sy = self._np_rng.normal(0.0, scale) if scale > 0 else 0.0
            out.append((x + sx, y + sy))
        return out

    def _advance_clock(self, dt):
        """Decrementa o tempo de ocupação de todos os robôs em dt (s).

        Acumula idle_time para os robôs que já estavam livres (busy=0) durante dt.
        Para robôs que ficam livres no meio do intervalo, acumula só o excedente.
        """
        if dt <= 0:
            return
        for r in range(self.num_robots):
            free_dt = max(0.0, dt - self._busy[r])
            self._idle_time[r] += free_dt
            self._busy[r] = max(0.0, self._busy[r] - dt)

    def _load_imbalance(self):
        """Desvio-padrão normalizado do nº de tarefas por robô (>= 0)."""
        counts = self._task_count
        mean = sum(counts) / len(counts)
        if mean == 0:
            return 0.0
        var = sum((c - mean) ** 2 for c in counts) / len(counts)
        return math.sqrt(var) / (mean + 1.0)

    def _estimate_t_ref(self):
        """Tempo médio aproximado de uma tarefa típica, para normalizar reward."""
        pts = list(self.waypoints.values())
        if len(pts) < 2:
            return 1.0
        # Distância média entre pares de waypoints, ida e volta aproximada.
        dists = []
        for i in range(len(pts)):
            for j in range(len(pts)):
                if i != j:
                    dists.append(math.hypot(pts[i][0] - pts[j][0],
                                            pts[i][1] - pts[j][1]))
        avg = sum(dists) / len(dists)
        # robô->origem (~avg) + origem->destino (~avg)
        return self.nav.fixed_overhead + 2 * avg / self.nav.v_nominal
