"""Etapa de correção do EKF, compartilhada entre produção (ekf_fusion_node.py),
avaliação offline (scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py, eval_ekf_continuous_error.py)
e validação sintética (scripts/lafusion/1.validation/validate_ekf_synthetic.py).

Extraído em 2026-08-24 (branch refactor/unify-ekf-core): as 4 implementações
tinham correct()/r_from_confidence() bit-a-bit idênticas (forma de Joseph,
gating com S=P+R), enquanto a etapa de predição usa dois modelos de
movimento genuinamente diferentes entre produção (delta de odometria) e
validação sintética (unicycle v/w) — por isso só a correção foi unificada;
predict() permanece definido em cada consumidor. Ver CLAUDE.md, seção
"Known duplication".
"""

import numpy as np


def r_from_confidence(conf: float, r_min: float, r_max: float) -> np.ndarray:
    """R (covariância de medição) decresce com a confiança do detector YOLO.

    r_min/r_max não têm default aqui de propósito: cada consumidor injeta o
    par calibrado para o seu contexto (produção/eval usam 0.02-0.20; a
    validação sintética historicamente usou os mesmos valores, mas mantém a
    injeção explícita para não acoplar essa lib a uma calibração específica).
    """
    conf = np.clip(conf, 0.05, 1.0)
    sigma = r_min + (1.0 - conf) * (r_max - r_min)
    return np.diag([sigma ** 2, sigma ** 2])


def correct(state: np.ndarray, cov: np.ndarray, measurement, R: np.ndarray):
    """Passo de correção do EKF: medição de posição [px, py] (H fixo, 2x3).

    Retorna (state_novo, cov_novo, innovation, S) — innovation e S expostos
    porque scripts/lafusion/1.validation/validate_ekf_synthetic.py precisa deles para NEES/NIS.

    Forma de Joseph: preserva simetria e positividade semi-definida de cov
    mesmo sob erro numérico, ao contrário da forma simplificada (I-KH)P, que
    só é exata para K ótimo sem arredondamento (Bar-Shalom et al.,
    Estimation with Applications to Tracking and Navigation). Gating de
    Mahalanobis a montante (association.py) usa S=P+R, não só P.
    """
    H = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    innovation = np.asarray(measurement) - H @ state
    S = H @ cov @ H.T + R
    K = cov @ H.T @ np.linalg.inv(S)

    state_new = state + K @ innovation
    I_KH = np.eye(3) - K @ H
    cov_new = I_KH @ cov @ I_KH.T + K @ R @ K.T
    return state_new, cov_new, innovation, S
