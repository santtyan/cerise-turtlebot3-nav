#!/usr/bin/env python3
"""
Teste de regressão (golden master) entre as duas implementações de
predict() do EKF que deveriam concordar bit-a-bit, mas não são a mesma
função: RobotEKF.predict_from_odom() em ekf_fusion_node.py (produção) e
RobotEKF.predict() no caminho COV_UPDATE_MODE='clip' em
scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py (avaliação offline).

Por que não unificadas: ver CLAUDE.md ("EKF math: correct() unified,
predict() intentionally stays split") — a avaliação carrega um modo
experimental extra (fading factor) que produção não deve ter, e unificar
a função forçaria produção a carregar esse branch. correct() já é
compartilhada via ekf_core.py; predict() no caminho comum (clip) não é,
e nada detecta se as duas divergirem silenciosamente.

Este teste não importa ekf_fusion_node.py (exige rclpy) nem
eval_ekf_vs_baseline.py (exige rosbag2_py) diretamente — nenhum dos dois
é garantido disponível fora de um ambiente ROS sourced, e a lógica de
predict() em si não usa nada de ROS. Reimplementa aqui, verbatim, as duas
classes RobotEKF a partir do texto atual dos dois arquivos-fonte, com o
propósito explícito de comparar uma contra a outra.

Se este teste falhar, ou se um dos dois arquivos-fonte mudar a fórmula
do predict() no caminho comum sem que ninguém perceba, primeiro confira:
  - produção: src/cerise_nav/cerise_nav/ekf_fusion_node.py,
    RobotEKF.predict_from_odom() (Q_DIAG, COV_CAP no topo do arquivo)
  - avaliação: scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py,
    RobotEKF.predict() caminho 'clip' (Q_DIAG, COV_CAP no topo do arquivo)
e atualize as reimplementações abaixo para refletir a mudança.

Uso: python3 test_ekf_predict_regression.py
"""

import math

import numpy as np


# Reimplementação verbatim de RobotEKF.predict_from_odom() em
# src/cerise_nav/cerise_nav/ekf_fusion_node.py (produção).
PROD_Q_DIAG = (0.5, 0.5, 0.25)
PROD_COV_CAP = 0.05


class ProductionEKF:
    def __init__(self, initial_state):
        self.state = np.array(initial_state, dtype=float).copy()
        self.cov = np.diag([0.01, 0.01, 0.01])
        self._last_odom = None
        self._last_t = None

    def predict_from_odom(self, x, y, theta, t):
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

        Q = np.diag(PROD_Q_DIAG) * (dt / 0.1)
        F = np.eye(3)
        self.state = self.state + np.array([dx, dy, dtheta])
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        self.cov = F @ self.cov @ F.T + Q
        np.clip(self.cov, None, PROD_COV_CAP, out=self.cov)


# Reimplementação verbatim do caminho 'clip' de RobotEKF.predict() em
# scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py (avaliação
# offline), usando as mesmas constantes Q_DIAG/COV_CAP daquele arquivo
# (não as de produção) para provar que ambos concordam mesmo calibrados
# independentemente.
EVAL_Q_DIAG = (0.5, 0.5, 0.25)
EVAL_COV_CAP = 0.05


class EvalClipEKF:
    def __init__(self, initial_state):
        self.state = np.array(initial_state, dtype=float)
        self.cov = np.diag([0.01, 0.01, 0.01])
        self._last_odom = None
        self._last_t = None

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

        Q = np.diag(EVAL_Q_DIAG) * (dt / 0.1)
        F = np.eye(3)
        self.state = self.state + np.array([dx, dy, dtheta])
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        self.cov = F @ self.cov @ F.T + Q
        if EVAL_COV_CAP is not None:
            np.clip(self.cov, None, EVAL_COV_CAP, out=self.cov)


def run_sequence(ekf_cls, predict_method_name, odom_sequence):
    """Roda a sequência e retorna (state, cov) a CADA passo, não só o
    final — cov satura no teto (COV_CAP) depois de poucos passos sem
    correção, o que mascararia uma divergência na fórmula de Q se só o
    estado final fosse comparado."""
    ekf = ekf_cls(np.array([0.0, 0.0, 0.0]))
    predict = getattr(ekf, predict_method_name)
    history = []
    for x, y, theta, t in odom_sequence:
        predict(x, y, theta, t)
        history.append((ekf.state.copy(), ekf.cov.copy()))
    return history


def make_sequence(n=50, dt=1 / 29, seed=42):
    rng = np.random.default_rng(seed)
    x, y, theta, t = 0.0, 0.0, 0.0, 0.0
    seq = [(x, y, theta, t)]
    for _ in range(n):
        x += rng.normal(0.02, 0.01)
        y += rng.normal(0.0, 0.01)
        theta += rng.normal(0.0, 0.02)
        t += dt
        seq.append((x, y, theta, t))
    return seq


# Com o COV_CAP=0.05 real do projeto, qualquer Q plausível já satura o
# teto num único passo em dt~1/29s (verificado: Q*dt/0.1 > COV_CAP desde
# o primeiro passo) — nesse regime, cov fica insensível a uma divergência
# na FÓRMULA de Q, só detecta divergência em Q_DIAG/COV_CAP em si (via
# assert acima). Um dt bem menor mantém Q abaixo do teto por mais passos,
# dando sensibilidade real à fórmula (dt/0.1, o wrap de ângulo, etc.).
SUB_CAP_DT = 1e-4


def main():
    assert PROD_Q_DIAG == EVAL_Q_DIAG, (
        f"Q_DIAG divergiu: produção={PROD_Q_DIAG} avaliação={EVAL_Q_DIAG} "
        "(atualize EVAL_Q_DIAG neste teste para refletir "
        "scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py)"
    )
    assert PROD_COV_CAP == EVAL_COV_CAP, (
        f"COV_CAP divergiu: produção={PROD_COV_CAP} avaliação={EVAL_COV_CAP}"
    )

    failures = []
    n_steps_checked = 0
    for dt, label in [(1 / 29, "dt real (~29Hz)"), (SUB_CAP_DT, "dt sub-teto")]:
        for seed in range(5):
            seq = make_sequence(dt=dt, seed=seed)

            prod_history = run_sequence(ProductionEKF, 'predict_from_odom', seq)
            eval_history = run_sequence(EvalClipEKF, 'predict', seq)

            for step, ((prod_state, prod_cov), (eval_state, eval_cov)) in enumerate(
                    zip(prod_history, eval_history)):
                n_steps_checked += 1
                if not np.allclose(prod_state, eval_state, atol=1e-12):
                    failures.append(
                        f"{label} seed={seed} step={step}: state diverge — "
                        f"prod={prod_state} eval={eval_state}")
                    break
                if not np.allclose(prod_cov, eval_cov, atol=1e-12):
                    failures.append(
                        f"{label} seed={seed} step={step}: cov diverge — "
                        f"prod=\n{prod_cov}\neval=\n{eval_cov}")
                    break

    if failures:
        print("FALHOU — predict() de produção e avaliação (caminho clip) divergiram:")
        for f in failures:
            print(f"  {f}")
        raise SystemExit(1)

    print(f"OK — predict_from_odom() (produção) e RobotEKF.predict() caminho "
          f"'clip' (avaliação) concordam bit-a-bit em {n_steps_checked} passos "
          f"across 5 sequências de teste.")


if __name__ == '__main__':
    main()
