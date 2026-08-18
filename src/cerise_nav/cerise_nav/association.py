"""Associação de detecções a robôs (greedy nearest-neighbor e gating por Mahalanobis).

Extraído de yolo_detector.py:_compute_error e rl_task_allocator.py:estimate_positions,
que duplicavam a mesma lógica greedy euclidiana (etapa 0 do plano LAFusion, 12/08/2026).
"""

import math

import numpy as np


def greedy_nearest_neighbor(reference: dict, detections: list, max_dist: float):
    """Associa cada item de `reference` (robot_id -> (x,y)) à detecção mais
    próxima em `detections` (lista de (x,y)), sem repetir detecções.

    Retorna (assignments, unmatched_refs), onde assignments é
    {robot_id: (det_x, det_y)} só para os robôs que encontraram uma
    detecção dentro de max_dist, e unmatched_refs é a lista de robot_ids
    sem detecção associada (fallback do chamador, ex. usar odometria).
    """
    assignments = {}
    used = set()
    unmatched = []

    for robot_id, (rx, ry) in reference.items():
        best_d, best_j = float('inf'), -1
        for j, (dx, dy) in enumerate(detections):
            if j in used:
                continue
            d = math.hypot(dx - rx, dy - ry)
            if d < best_d:
                best_d, best_j = d, j

        if best_j >= 0 and best_d <= max_dist:
            used.add(best_j)
            assignments[robot_id] = detections[best_j]
        else:
            unmatched.append(robot_id)

    return assignments, unmatched


def mahalanobis_gate(reference: dict, cov_by_robot: dict, detections: list,
                      gate_threshold: float = 9.21, r_by_detection: list = None):
    """Associa detecções a robôs por distância de Mahalanobis, usando a
    covariância de inovação S = P + R de cada par robô-detecção (mais
    rigoroso que distância euclidiana pura — ver Altendorfer & Wirkert 2015,
    referência #4 do plano LAFusion, que documenta o limite conhecido do
    gating puramente euclidiano/Mahalanobis ingênuo: covariâncias grandes
    podem "roubar" associações incorretas).

    S = P + R (não só P) é a covariância de inovação padrão para gating
    (Bar-Shalom et al.; mesma fórmula usada em correct_with_detection) — sem
    somar R, o gate ignora a incerteza da própria detecção (confiança do
    YOLO), ficando artificialmente estreito quando R é grande.
    `r_by_detection`, se informado, é uma lista de matrizes R (2x2) alinhada
    a `detections`; se None, mantém o comportamento anterior (S=P) para
    compatibilidade com chamadores que não têm R disponível.

    gate_threshold=9.21 é o valor padrão do teste qui-quadrado com 2 graus de
    liberdade e 99% de confiança (posição x,y) — reprovar associações além
    desse limiar evita "roubo" de detecções por robôs com covariância inflada.

    Retorna (assignments, unmatched_refs), mesmo formato de greedy_nearest_neighbor.
    """
    assignments = {}
    used = set()
    unmatched = []

    for robot_id, (rx, ry) in reference.items():
        cov = cov_by_robot[robot_id]
        cov_pos = np.asarray(cov)[:2, :2]

        best_d, best_j = float('inf'), -1
        for j, (dx, dy) in enumerate(detections):
            if j in used:
                continue
            R = r_by_detection[j] if r_by_detection is not None else 0.0
            S = cov_pos + R
            S_inv = np.linalg.inv(S)
            err = np.array([dx - rx, dy - ry])
            d = float(err @ S_inv @ err)
            if d < best_d:
                best_d, best_j = d, j

        if best_j >= 0 and best_d <= gate_threshold:
            used.add(best_j)
            assignments[robot_id] = detections[best_j]
        else:
            unmatched.append(robot_id)

    return assignments, unmatched
