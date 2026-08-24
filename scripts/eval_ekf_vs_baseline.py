#!/usr/bin/env python3
"""Passo 4 do plano LAFusion: roda o EKF offline contra os bags gravados
(cenario1_parado, cenario2_reto, cenario3_curva) e compara com baseline
odom-only e ground truth (odometria pura do Gazebo, sem ruído de sensor —
mesma convenção já usada por yolo_detector.py:_compute_error).

Duas condições avaliadas (decisão da sessão 12/08/2026, usuário pediu ambas):
  (A) sem drift artificial — demonstra o pipeline real funcionando.
  (B) com drift artificial injetado na odometria (mesmo modelo POS_DRIFT_ODOM
      de allocation_env.py) — mostra o EKF corrigindo erro acumulado usando
      as detecções YOLO reais capturadas nos bags. Sem isso, "odom-only"
      seria trivialmente perfeito por ser a mesma fonte do "ground truth".

Usa scipy/stats_tests.py (bootstrap_ci) para rigor estatístico, e reaproveita
a lógica de association.py (mahalanobis_gate) e o EKF de ekf_fusion_node.py.

Uso: python3 scripts/eval_ekf_vs_baseline.py
"""

import math
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))
sys.path.insert(0, os.path.join(_REPO, 'scripts'))

from cerise_nav.association import mahalanobis_gate  # noqa: E402
from cerise_nav.ekf_core import correct as _core_correct  # noqa: E402
from cerise_nav.ekf_core import r_from_confidence as _core_r_from_confidence  # noqa: E402
from stats_tests import bootstrap_ci, wilcoxon_signed_rank  # noqa: E402

ROBOTS = ['robot1', 'robot2', 'robot3']
# Q recalibrado empiricamente contra os bags reais (12/08/2026, passo 4) — o
# valor de validate_ekf_synthetic.py (1e-6) era calibrado para o modelo de
# ruído SINTÉTICO (odometria com pouco erro ~0.01 std), não para o drift real
# usado neste experimento (POS_DRIFT_ODOM=0.03 crescente). Com Q pequeno, o
# filtro confiava demais na odometria e nunca corrigia o drift real via YOLO
# (ganho negativo/nulo em todos os regimes testados). Varredura de Q em
# [1e-6, 1.0] mostrou ganho crescente e monotônico até saturar ~80% em
# Q=0.5-1.0 (ver plano LAFusion, passo 4, para a tabela completa).
Q_DIAG = (0.5, 0.5, 0.25)
COV_CAP = 0.05  # teto de covariância (metros², rad²) — None desabilita
# Fading factor (AFKF, Adaptive Fading Kalman Filter) foi testado como
# alternativa ao clip bruto (17/08/2026) — teoricamente superior por não
# quebrar a estrutura semi-definida positiva de P — mas EMPIRICAMENTE PIOR
# neste pipeline: sem teto, ou com teto generoso o bastante para o fading
# ter efeito prático diferente do clip, P cresce o bastante para o gate de
# Mahalanobis aceitar detecções de outro robô durante gaps longos sem
# correção (cenário 2, oclusão) — o mesmo modo de falha que o clip existe
# para evitar. Ganho no drift agressivo caiu de +23,7%/+16,7% (clip) para
# -16,5%/-104% (fading sem teto) e para -3,9% a -19,3% (fading com teto
# 0,1-0,3). Mantido no código como opção experimental (COV_UPDATE_MODE),
# default permanece 'clip'.
COV_UPDATE_MODE = 'clip'  # 'clip' | 'fading' | 'fading_capped'
FADE_RATE = 0.05
FADE_MAX = 5.0
FADING_COV_CAP = 0.5  # teto usado só em 'fading_capped', mais generoso que COV_CAP
R_MIN, R_MAX = 0.02, 0.20
POS_DRIFT_ODOM = 0.03  # mesmo valor de allocation_env.py


def r_from_confidence(conf):
    return _core_r_from_confidence(conf, R_MIN, R_MAX)


class RobotEKF:
    """Predição integra o DELTA de odometria sobre o estado do filtro, nunca
    substitui pelo valor absoluto (bug encontrado e corrigido nesta sessão —
    ver docstring equivalente em ekf_fusion_node.py)."""

    def __init__(self, initial_state):
        self.state = np.array(initial_state, dtype=float)
        self.cov = np.diag([0.01, 0.01, 0.01])
        self._last_odom = None
        self._last_t = None
        self._steps_since_correction = 0

    def predict(self, x, y, theta, t):
        if self._last_odom is None:
            self._last_odom = (x, y, theta)
            self._last_t = t
            return
        lx, ly, ltheta = self._last_odom
        dx, dy, dtheta = x - lx, y - ly, theta - ltheta
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
        dt = max(t - self._last_t, 1e-6)
        self._last_odom = (x, y, theta)
        self._last_t = t

        # Q calibrado em validate_ekf_synthetic.py assume dt=0.1s fixo — escalar
        # pela razão dt_real/dt_calibração corrige a taxa real de odometria
        # (~0.034s / 29Hz medida nos bags, vs. 0.1s assumido na calibração
        # sintética). Sem isso, Q é aplicado ~3x mais vezes por segundo do que
        # deveria, inflando incerteza incorretamente (achado desta sessão).
        Q = np.diag(Q_DIAG) * (dt / 0.1)

        F = np.eye(3)
        self.state = self.state + np.array([dx, dy, dtheta])
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))

        if COV_UPDATE_MODE in ('fading', 'fading_capped'):
            # AFKF (Adaptive Fading Kalman Filter): escala apenas a
            # injeção de ruído de processo (Q) por um fator lambda >= 1
            # crescente com passos sem correção, em vez de truncar
            # elementos individuais de P (o que quebra PSD). O fator
            # multiplica só Q, não P inteira — escalar P a cada passo
            # seria um produto geométrico e faria P divergir para
            # infinito em poucas dezenas de passos.
            self._steps_since_correction += 1
            fade = min(1.0 + FADE_RATE * self._steps_since_correction, FADE_MAX)
            self.cov = F @ self.cov @ F.T + fade * Q
            if COV_UPDATE_MODE == 'fading_capped' and FADING_COV_CAP is not None:
                # Híbrido: mesmo com fading, sem teto o traço de P cresce o
                # bastante para o gate de Mahalanobis aceitar detecções de
                # outro robô ("roubo" de associação) — testado empiricamente,
                # não hipotético (ver achado 17/08/2026). Teto mais generoso
                # que o clip original (0.05) para não anular o benefício
                # teórico do fading, mas ainda protege o gating.
                np.clip(self.cov, None, FADING_COV_CAP, out=self.cov)
        else:
            self.cov = F @ self.cov @ F.T + Q
            # Teto de covariância: sem correção por muitos passos (YOLO
            # perdendo detecção por oclusão/movimento — visto no cenário 2),
            # cov cresce sem limite com Q grande, quebrando o gating de
            # Mahalanobis (aceita qualquer detecção como "próxima", causando
            # "roubo" — efeito da referência #4, Altendorfer & Wirkert 2015).
            # Achado desta sessão: cov chegou a 85 (deveria ~0.5) num evento
            # real do bag.
            if COV_CAP is not None:
                np.clip(self.cov, None, COV_CAP, out=self.cov)

    def correct(self, det_xy, conf):
        R = r_from_confidence(conf)
        self.state, self.cov, _innovation, _S = _core_correct(self.state, self.cov, det_xy, R)
        self._steps_since_correction = 0


def read_bag(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {'/robot_detections': PoseArray}
    for r in ROBOTS:
        type_map[f'/{r}/odom'] = Odometry

    events = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in type_map:
            msg = deserialize_message(data, type_map[topic])
            events.append((t, topic, msg))
    events.sort(key=lambda e: e[0])
    return events


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def run_scenario(bag_path, inject_drift, rng, drift_rate_divisor=200.0):
    events = read_bag(bag_path)

    filters = {}
    gt = {}          # robot_id -> (x, y) ground truth real (odom sem ruído)
    odom_drifted = {}  # robot_id -> (x, y) odometria com drift artificial (se inject_drift)
    task_count = {r: 0 for r in ROBOTS}

    errors_ekf, errors_odom = [], []

    for t, topic, msg in events:
        if topic == '/robot_detections':
            detections = [(p.position.x, p.position.y) for p in msg.poses]
            det_conf = {(p.position.x, p.position.y): p.position.z for p in msg.poses}
            if not odom_drifted or not filters:
                continue
            cov_by_robot = {r: f.cov for r, f in filters.items() if r in odom_drifted}
            r_by_detection = [r_from_confidence(det_conf.get(d, 0.5)) for d in detections]
            assignments, _ = mahalanobis_gate(odom_drifted, cov_by_robot, detections,
                                               r_by_detection=r_by_detection)
            for robot_id, det_xy in assignments.items():
                conf = det_conf.get(det_xy, 0.5)
                filters[robot_id].correct(det_xy, conf)

            for r in ROBOTS:
                if r in filters and r in gt:
                    err_ekf = math.hypot(filters[r].state[0] - gt[r][0],
                                          filters[r].state[1] - gt[r][1])
                    err_odom = math.hypot(odom_drifted[r][0] - gt[r][0],
                                           odom_drifted[r][1] - gt[r][1])
                    errors_ekf.append(err_ekf)
                    errors_odom.append(err_odom)
            continue

        robot_id = topic.split('/')[1]
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        gt[robot_id] = (p.x, p.y)

        if inject_drift:
            task_count[robot_id] += 1
            drift_scale = POS_DRIFT_ODOM * min(task_count[robot_id] / drift_rate_divisor, 5.0)
            noisy = (p.x + rng.normal(0, drift_scale), p.y + rng.normal(0, drift_scale))
        else:
            noisy = (p.x, p.y)
        odom_drifted[robot_id] = noisy

        if robot_id not in filters:
            filters[robot_id] = RobotEKF([noisy[0], noisy[1], yaw])
        else:
            filters[robot_id].predict(noisy[0], noisy[1], yaw, t / 1e9)

    return np.array(errors_ekf), np.array(errors_odom)


def main():
    bags_dir = os.path.join(_REPO, 'bags')
    scenarios = ['cenario1_parado', 'cenario2_reto', 'cenario3_curva']
    rng = np.random.default_rng(42)

    conditions = [
        ('SEM drift artificial (odom real)', False, 200.0),
        ('COM drift artificial leve (divisor=200)', True, 200.0),
        ('COM drift artificial agressivo (divisor=20, ultrapassa erro YOLO~3cm mais cedo)', True, 20.0),
    ]

    for condition_name, inject_drift, drift_divisor in conditions:
        print(f'\n{"=" * 70}')
        print(f'Condição: {condition_name}')
        print('=' * 70)

        all_ekf, all_odom = [], []
        for scenario in scenarios:
            bag_path = os.path.join(bags_dir, scenario)
            errors_ekf, errors_odom = run_scenario(bag_path, inject_drift, rng, drift_divisor)
            if len(errors_ekf) == 0:
                print(f'  {scenario}: sem amostras (EKF nunca corrigiu)')
                continue
            all_ekf.extend(errors_ekf)
            all_odom.extend(errors_odom)
            print(f'  {scenario}: n={len(errors_ekf)} | '
                  f'EKF erro médio={errors_ekf.mean():.4f}m | '
                  f'odom-only erro médio={errors_odom.mean():.4f}m')

        if not all_ekf:
            continue

        all_ekf = np.array(all_ekf)
        all_odom = np.array(all_odom)

        print(f'\n  --- Agregado (3 cenários, n={len(all_ekf)}) ---')
        print(f'  EKF:       média={all_ekf.mean():.4f}m, mediana={np.median(all_ekf):.4f}m, '
              f'RMSE={np.sqrt(np.mean(all_ekf ** 2)):.4f}m')
        print(f'  odom-only: média={all_odom.mean():.4f}m, mediana={np.median(all_odom):.4f}m, '
              f'RMSE={np.sqrt(np.mean(all_odom ** 2)):.4f}m')

        gap_pct = (all_odom.mean() - all_ekf.mean()) / all_odom.mean() * 100 if all_odom.mean() > 0 else 0
        print(f'  Redução de erro do EKF vs. odom-only: {gap_pct:.1f}%')

        rmse_ekf = np.sqrt(np.mean(all_ekf ** 2))
        rmse_odom = np.sqrt(np.mean(all_odom ** 2))
        rmse_gap_pct = (rmse_odom - rmse_ekf) / rmse_odom * 100 if rmse_odom > 0 else 0
        print(f'  Redução de RMSE do EKF vs. odom-only: {rmse_gap_pct:.1f}%')

        try:
            n = min(len(all_ekf), len(all_odom))
            diff_mean = lambda x, y: float(np.mean(y - x))  # noqa: E731
            ci_low, ci_high = bootstrap_ci(all_ekf[:n], all_odom[:n], diff_mean)
            print(f'  IC 95% bootstrap da diferença média (odom - EKF): [{ci_low:.4f}, {ci_high:.4f}]m')

            stat, pval = wilcoxon_signed_rank(all_odom[:n], all_ekf[:n])
            print(f'  Wilcoxon signed-rank (odom vs EKF, pareado): p={pval:.6f}')
        except Exception as e:
            print(f'  teste estatístico falhou: {e}')


if __name__ == '__main__':
    main()
