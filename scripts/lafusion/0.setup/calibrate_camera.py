#!/usr/bin/env python3
"""Passo 2.5 do plano LAFusion: calibra a câmera overhead do Gazebo com
cv2.calibrateCamera (método de Zhang, 2000 — referência #11 do plano),
substituindo a projeção heurística de pixel_to_world_simple por intrínsecos
reais, reaproveitados por world_to_pixel_with_camera() (projection.py).

Spawna um tabuleiro de xadrez (geometria pura, sem textura externa) em
múltiplas poses numa área livre calculada matematicamente (fora do alcance
dos 3 robôs e do cilindro central do turtlebot3_world), captura frames,
detecta cantos, e roda a calibração.

Uso: python3 scripts/calibrate_camera.py
Requer: Gazebo rodando com world_with_camera.world (3 robôs + câmera overhead).
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from sensor_msgs.msg import Image

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_REPO, 'src', 'cerise_nav'))
from cerise_nav.projection import pixel_to_world_simple  # noqa: E402


BOARD_COLS, BOARD_ROWS, SQUARE_SIZE = 5, 4, 0.10  # 5x4 quadrados = 4x3 cantos internos
INNER_CORNERS = (BOARD_COLS - 1, BOARD_ROWS - 1)

# Obstáculos conhecidos do world_with_camera.world (mundo, metros, raio)
OBSTACLES = [
    (0.0, 0.0, 0.15),    # cilindro central do turtlebot3_world
    (0.80, 0.40, 0.20),  # robot1
    (-0.80, 0.40, 0.20),  # robot2
    (0.00, -0.70, 0.20),  # robot3
]


def board_sdf(offset_x, offset_y, yaw=0.0):
    half_w, half_h = BOARD_COLS * SQUARE_SIZE / 2, BOARD_ROWS * SQUARE_SIZE / 2
    links = []
    for r in range(BOARD_ROWS):
        for c in range(BOARD_COLS):
            color = '0 0 0 1' if (r + c) % 2 == 0 else '1 1 1 1'
            x = -half_w + SQUARE_SIZE / 2 + c * SQUARE_SIZE + offset_x
            y = -half_h + SQUARE_SIZE / 2 + r * SQUARE_SIZE + offset_y
            links.append(f'''
      <visual name="sq_{r}_{c}">
        <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
        <geometry><box><size>{SQUARE_SIZE} {SQUARE_SIZE} 0.001</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>''')
    return f'''<sdf version="1.6">
  <model name="calibration_checkerboard">
    <static>true</static>
    <pose>0 0 0.002 0 0 {yaw}</pose>
    <link name="board_link">{''.join(links)}
    </link>
  </model>
</sdf>
'''


def find_free_positions(n_positions=8, min_margin=0.30, min_spacing=0.15):
    """Busca posições livres no espaço de pixel normalizado, convertendo para
    mundo via pixel_to_world_simple — evita erro de mapeamento de eixos entre
    world/câmera (ver sessão 12/08/2026: tentativa direta em coordenadas
    mundo causou 8 posicionamentos incorretos antes desta correção).

    Prioriza cobertura espacial (espalhar por todo o frame, não só perto do
    centro) — poses concentradas numa região pequena deixam o sistema de
    equações da calibração mal condicionado, mesmo com erro de reprojeção
    baixo (ver checagem de sanidade fx no final do script)."""
    board_half_diag = math.hypot(BOARD_COLS * SQUARE_SIZE / 2, BOARD_ROWS * SQUARE_SIZE / 2)
    candidates = []
    # Evitar extremos do frame (distorção de perspectiva quebra a detecção de
    # cantos) — manter dentro de [0.15, 0.85] normalizado, não [0,1] completo.
    for pxn in [i / 40 for i in range(6, 35)]:
        for pyn in [i / 40 for i in range(6, 35)]:
            wx, wy = pixel_to_world_simple(pxn, pyn)
            min_d = min(math.hypot(wx - ox, wy - oy) - orad for ox, oy, orad in OBSTACLES)
            if min_d > min_margin + board_half_diag:
                candidates.append((pxn, pyn, wx, wy))

    # Greedy farthest-point sampling: cada nova posição é a mais distante das
    # já escolhidas, para maximizar cobertura do frame em vez de concentrar
    # perto do centro (que dava erro de reprojeção baixo mas fx incorreto).
    if not candidates:
        return []
    chosen = [candidates[0]]
    remaining = candidates[1:]
    while len(chosen) < n_positions and remaining:
        best_idx, best_dist = -1, -1
        for idx, (pxn, pyn, wx, wy) in enumerate(remaining):
            min_d_to_chosen = min(math.hypot(pxn - cpx, pyn - cpy) for cpx, cpy, _, _ in chosen)
            if min_d_to_chosen > best_dist:
                best_dist, best_idx = min_d_to_chosen, idx
        if best_dist < min_spacing:
            break
        chosen.append(remaining.pop(best_idx))

    return [(wx, wy) for _, _, wx, wy in chosen]


def main():
    rclpy.init()
    node = rclpy.create_node('camera_calibrator')
    bridge = CvBridge()

    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    delete_client = node.create_client(DeleteEntity, '/delete_entity')
    spawn_client.wait_for_service(timeout_sec=10.0)
    delete_client.wait_for_service(timeout_sec=10.0)

    frame_holder = {}

    def img_cb(msg):
        frame_holder['frame'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    node.create_subscription(Image, '/camera/image_raw', img_cb, 1)

    positions = find_free_positions(n_positions=10)
    print(f'{len(positions)} posições livres calculadas para calibração')

    objp = np.zeros((INNER_CORNERS[0] * INNER_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:INNER_CORNERS[0], 0:INNER_CORNERS[1]].T.reshape(-1, 2) * SQUARE_SIZE

    objpoints, imgpoints = [], []
    img_shape = None

    for i, (wx, wy) in enumerate(positions):
        for yaw in (0.0, 0.3, 0.6, 0.9):  # 4 orientações por posição, mais variação
            xml = board_sdf(wx, wy, yaw=yaw)

            del_req = DeleteEntity.Request()
            del_req.name = 'calibration_checkerboard'
            fut = delete_client.call_async(del_req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

            spawn_req = SpawnEntity.Request()
            spawn_req.name = 'calibration_checkerboard'
            spawn_req.xml = xml
            fut = spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
            result = fut.result()
            if not result or not result.success:
                print(f'  pose {i},{yaw}: spawn falhou, pulando')
                continue

            time.sleep(1.0)
            frame_holder.clear()
            t0 = time.time()
            while 'frame' not in frame_holder and time.time() - t0 < 5:
                rclpy.spin_once(node, timeout_sec=0.3)

            if 'frame' not in frame_holder:
                print(f'  pose {i},{yaw}: sem frame, pulando')
                continue

            frame = frame_holder['frame']
            img_shape = frame.shape[:2][::-1]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, INNER_CORNERS, None)

            if found:
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                objpoints.append(objp)
                imgpoints.append(corners_refined)
                print(f'  pose {i},{yaw} @ ({wx:.2f},{wy:.2f}): cantos OK ({len(objpoints)} total)')
            else:
                print(f'  pose {i},{yaw} @ ({wx:.2f},{wy:.2f}): cantos NÃO encontrados')

    del_req = DeleteEntity.Request()
    del_req.name = 'calibration_checkerboard'
    fut = delete_client.call_async(del_req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

    print(f'\nTotal de poses válidas: {len(objpoints)}')
    if len(objpoints) < 3:
        print('RESULTADO: poses insuficientes para calibração confiável (mínimo 3, recomendado 5-10+).')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Calibração completa dos 4 intrínsecos (fx,fy,cx,cy) é mal-condicionada
    # neste setup: câmera fixa olhando reto para baixo (pitch=pi/2) e o
    # tabuleiro sempre no plano do chão (z~0) significa que TODAS as poses
    # têm o tabuleiro fronto-paralelo ao sensor (só variação de yaw/XY no
    # próprio plano, nunca de tilt relativo à câmera) — caso degenerado
    # clássico de Zhang (2000)/Sturm & Maybank (1999): sem variação de
    # inclinação relativa, fx e a distância percebida ficam quase
    # linearmente dependentes, permitindo erro de reprojeção baixo com fx/cx
    # fisicamente incorretos (achado real desta sessão: fx 33% inflado, cx
    # deslocado ~44px, erro de reprojeção ainda assim 0.162px).
    #
    # Correção padrão-ouro quando a geometria é conhecida a priori (aqui:
    # altura e FOV nominais da câmera simulada são exatos, não estimados):
    # fixar fx/fy/cx/cy no valor nominal geométrico e calibrar SÓ a
    # distorção radial/tangencial (Szeliski, "Computer Vision: Algorithms
    # and Applications", cap. 11; prática padrão em visão robótica quando
    # extrínsecos/intrínsecos nominais já são confiáveis). Evita o
    # mal-condicionamento em vez de tentar compensá-lo com mais poses no
    # mesmo plano degenerado.
    fov_h = 1.047  # rad, ver world_with_camera.world
    fx_nominal = (img_shape[0] / 2) / math.tan(fov_h / 2)
    fy_nominal = fx_nominal  # pixels quadrados, mesma escala em x/y
    cx_nominal, cy_nominal = img_shape[0] / 2.0, img_shape[1] / 2.0
    mtx_guess = np.array([[fx_nominal, 0, cx_nominal],
                           [0, fy_nominal, cy_nominal],
                           [0, 0, 1]], dtype=np.float64)

    calib_flags = (cv2.CALIB_USE_INTRINSIC_GUESS
                   | cv2.CALIB_FIX_FOCAL_LENGTH
                   | cv2.CALIB_FIX_PRINCIPAL_POINT
                   | cv2.CALIB_FIX_ASPECT_RATIO)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, mtx_guess, None, flags=calib_flags)

    print(f'\n=== Resultado da calibração (cv2.calibrateCamera, Zhang 2000, '
          f'fx/fy/cx/cy fixados no nominal geométrico — só distorção calibrada) ===')
    print(f'RMS reprojection error: {ret:.4f}')
    print(f'Matriz intrínseca (K):\n{mtx}')
    print(f'Coeficientes de distorção: {dist.ravel()}')

    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error
    mean_error = total_error / len(objpoints)
    print(f'Erro médio de reprojeção por ponto: {mean_error:.4f} px')

    # Checagem de sanidade: com CALIB_FIX_FOCAL_LENGTH/FIX_PRINCIPAL_POINT,
    # fx/cx são travados no nominal por construção — esta checagem confirma
    # que os flags realmente pegaram (guarda-corpo contra erro de versão do
    # OpenCV), não mais uma medida de mal-condicionamento como antes.
    fx_ratio = mtx[0, 0] / fx_nominal
    cx_shift_px = abs(mtx[0, 2] - cx_nominal)
    print(f'\nChecagem de sanidade: fx calibrado/nominal={fx_ratio:.4f} '
          f'(esperado 1.0000), deslocamento de cx={cx_shift_px:.2f}px (esperado 0px)')
    assert abs(fx_ratio - 1.0) < 1e-6 and cx_shift_px < 1e-6, \
        'fx/cx não ficaram fixos no nominal — flags CALIB_FIX_* não tiveram efeito'

    if mean_error < 0.2:
        print('RESULTADO: calibração BOA (distorção calibrada com reprojeção baixa, '
              'fx/fy/cx/cy no nominal geométrico por construção).')
    elif mean_error < 1.0:
        print('RESULTADO: calibração ACEITÁVEL, mas fora da faixa ideal (0-0.2px) — '
              'revisar detecção de cantos antes de aplicar ao pipeline.')
    else:
        print('RESULTADO: calibração RUIM — revisar poses/detecção antes de aplicar ao pipeline.')

    calibration_path = os.path.join(_REPO, 'config', 'camera_calibration.npz')
    np.savez(calibration_path,
              mtx=mtx, dist=dist, rms_error=ret, mean_reprojection_error=mean_error,
              fx_sanity_ratio=fx_ratio)
    print(f'\nSalvo em {calibration_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
