#!/usr/bin/env python3
"""
Teste end-to-end do pipeline dataset_collector.

Valida:
  1. Projeção world→pixel com geometria real da câmera
  2. Formato YOLO das anotações (class cx cy w h normalizados)
  3. Estrutura de pastas train/val
  4. Split 80/20 aplicado corretamente
"""

import cv2
import os
import shutil
import numpy as np
from pathlib import Path
from cerise_nav.projection import (
    world_to_pixel_simple,
    world_to_pixel_with_camera,
    robot_bbox_normalized,
    pixel_to_world_simple,
)

CAMERA_HEIGHT  = 3.0
HORIZONTAL_FOV = 1.047   # rad — deve bater com world_with_camera.model
IMG_W, IMG_H   = 640, 480
ROBOT_RADIUS   = 0.17    # metros — TurtleBot3 Waffle

# Meia-largura do campo visível: camera_height * tan(fov/2)
MAP_HALF_W = CAMERA_HEIGHT * np.tan(HORIZONTAL_FOV / 2.0)
MAP_HALF_H = MAP_HALF_W * (IMG_H / IMG_W)


def synthetic_frame(poses: dict) -> np.ndarray:
    """
    Gera frame sintético realista:
    - Fundo cinza simulando chão da arena
    - Círculos coloridos nas posições projetadas dos robôs
    """
    img = np.full((IMG_H, IMG_W, 3), 80, dtype=np.uint8)

    # Grade de referência (chão da arena)
    for i in range(0, IMG_W, 40):
        cv2.line(img, (i, 0), (i, IMG_H), (70, 70, 70), 1)
    for j in range(0, IMG_H, 40):
        cv2.line(img, (0, j), (IMG_W, j), (70, 70, 70), 1)

    colors = [(0, 200, 0), (0, 100, 255)]  # robot1=verde, robot2=laranja
    for idx, (name, (x, y)) in enumerate(poses.items()):
        cx_n, cy_n = world_to_pixel_simple(x, y, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
        px, py = int(cx_n * IMG_W), int(cy_n * IMG_H)
        radius_px = max(5, int((ROBOT_RADIUS / MAP_HALF_W) * IMG_W / 2))
        cv2.circle(img, (px, py), radius_px, colors[idx], -1)
        cv2.putText(img, name, (px + radius_px + 2, py), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return img


def generate_dataset(n_frames: int = 60) -> str:
    """Gera dataset sintético com n_frames frames e split train/val 80/20."""
    base = Path('dataset')
    for split in ('train', 'val'):
        (base / 'images' / split).mkdir(parents=True, exist_ok=True)
        (base / 'labels' / split).mkdir(parents=True, exist_ok=True)

    # Trajetórias sintéticas dentro do campo visível da câmera
    max_x = MAP_HALF_W * 0.85
    max_y = MAP_HALF_H * 0.85
    robot1_path = [
        (max_x * np.cos(t), max_y * np.sin(t))
        for t in np.linspace(0, 2 * np.pi, n_frames)
    ]
    # robot2 defasado em pi/2 para nunca coincidir com robot1
    robot2_path = [
        (max_x * 0.6 * np.cos(t + np.pi / 2), max_y * 0.6 * np.sin(t + np.pi / 2))
        for t in np.linspace(0, 2 * np.pi, n_frames)
    ]

    # Bbox fixo baseado na geometria da câmera
    w_pct = (ROBOT_RADIUS * 2 / MAP_HALF_W) / 2.0
    h_pct = (ROBOT_RADIUS * 2 / MAP_HALF_H) / 2.0

    n_train = int(n_frames * 0.8)

    for frame_id in range(n_frames):
        poses = {
            'robot1': robot1_path[frame_id],
            'robot2': robot2_path[frame_id],
        }
        split = 'train' if frame_id < n_train else 'val'
        img = synthetic_frame(poses)

        img_path = base / 'images' / split / f'{frame_id:06d}.jpg'
        ann_path = base / 'labels' / split / f'{frame_id:06d}.txt'

        cv2.imwrite(str(img_path), img)

        # Todos os robôs usam class_id=0 ('robot') — YOLO não distingue idênticos
        with open(ann_path, 'w') as f:
            for name, (x, y) in poses.items():
                cx, cy = world_to_pixel_simple(x, y, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
                f.write(f'0 {cx:.6f} {cy:.6f} {w_pct:.6f} {h_pct:.6f}\n')

    return str(base)


def validate_dataset(base: str):
    """Valida estrutura, formato YOLO e consistência do dataset gerado."""
    base = Path(base)

    for split in ('train', 'val'):
        imgs = sorted((base / 'images' / split).glob('*.jpg'))
        anns = sorted((base / 'labels' / split).glob('*.txt'))

        assert len(imgs) > 0, f'Nenhuma imagem em {split}'
        assert len(imgs) == len(anns), f'Mismatch imagens/labels em {split}: {len(imgs)} vs {len(anns)}'

        for ann_path in anns:
            with open(ann_path) as f:
                lines = f.readlines()
            assert len(lines) == 2, f'{ann_path}: esperado 2 robos, obtido {len(lines)}'
            for line in lines:
                parts = line.strip().split()
                assert len(parts) == 5, f'Formato YOLO invalido em {ann_path}: {parts}'
                class_id, cx, cy, w, h = map(float, parts)
                assert class_id == 0.0, f'class_id deve ser 0 (unica classe robot): {class_id}'
                assert 0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0, f'Coords fora de range: cx={cx}, cy={cy}'
                assert 0.0 < w <= 1.0 and 0.0 < h <= 1.0, f'Bbox invalido: w={w}, h={h}'

        print(f'  [{split}] {len(imgs)} frames OK')

    # Valida proporção train/val (~80/20)
    n_train = len(list((base / 'images' / 'train').glob('*.jpg')))
    n_val   = len(list((base / 'images' / 'val').glob('*.jpg')))
    ratio   = n_train / (n_train + n_val)
    assert 0.75 <= ratio <= 0.85, f'Split fora do esperado 80/20: {ratio:.2f}'
    print(f'  Split: {n_train} train / {n_val} val ({ratio:.0%}/{1-ratio:.0%})')


class _FakeCameraInfo:
    """Mock de sensor_msgs/CameraInfo com intrínsecos derivados do FOV."""
    def __init__(self, img_w=IMG_W, img_h=IMG_H, fov=HORIZONTAL_FOV):
        self.width = img_w
        self.height = img_h
        # fx derivado do FOV horizontal: fx = (img_w/2) / tan(fov/2)
        fx = (img_w / 2.0) / np.tan(fov / 2.0)
        fy = fx  # pixels quadrados
        self.k = [fx, 0.0, img_w / 2.0,
                  0.0, fy, img_h / 2.0,
                  0.0, 0.0, 1.0]


def test_projection_geometry():
    """Valida projecao simple (FOV) e with_camera (intrinsecos) produzem resultados consistentes."""
    # --- Teste 1: projecao simples ---
    cx, cy = world_to_pixel_simple(0.0, 0.0, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
    assert abs(cx - 0.5) < 0.01 and abs(cy - 0.5) < 0.01, f'Simple centro falhou: ({cx:.3f},{cy:.3f})'

    cx, cy = world_to_pixel_simple(MAP_HALF_W, 0.0, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
    assert cx > 0.98, f'Simple borda direita falhou: {cx:.3f}'

    # Roundtrip
    for wx, wy in [(0.5, 0.3), (-0.8, 0.6), (1.0, -0.5)]:
        cx, cy = world_to_pixel_simple(wx, wy, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
        rx, ry = pixel_to_world_simple(cx, cy, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
        assert abs(rx - wx) < 0.01 and abs(ry - wy) < 0.01, f'Roundtrip falhou: ({wx},{wy})->({rx:.3f},{ry:.3f})'

    # --- Teste 2: projecao with_camera (usada no dataset_collector real) ---
    ci = _FakeCameraInfo()
    result = world_to_pixel_with_camera(0.0, 0.0, CAMERA_HEIGHT, ci)
    assert result is not None, 'with_camera centro retornou None'
    cx, cy = result
    assert abs(cx - 0.5) < 0.01 and abs(cy - 0.5) < 0.01, f'with_camera centro falhou: ({cx:.3f},{cy:.3f})'

    # --- Teste 3: simple e with_camera devem concordar em pontos internos ---
    for wx, wy in [(0.5, 0.3), (-0.8, 0.6), (1.0, -0.5), (0.0, 0.0)]:
        s_cx, s_cy = world_to_pixel_simple(wx, wy, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
        wc = world_to_pixel_with_camera(wx, wy, CAMERA_HEIGHT, ci)
        assert wc is not None, f'with_camera retornou None para ({wx},{wy})'
        assert abs(s_cx - wc[0]) < 0.01 and abs(s_cy - wc[1]) < 0.01, \
            f'Divergencia em ({wx},{wy}): simple=({s_cx:.3f},{s_cy:.3f}) vs cam=({wc[0]:.3f},{wc[1]:.3f})'

    # --- Teste 4: with_camera retorna None fora do frame ---
    assert world_to_pixel_with_camera(10.0, 0.0, CAMERA_HEIGHT, ci) is None, 'Ponto fora do FOV deveria retornar None'

    # --- Teste 5: bbox dinamico ---
    w_n, h_n = robot_bbox_normalized(CAMERA_HEIGHT, ci, ROBOT_RADIUS)
    assert 0.0 < w_n < 0.2 and 0.0 < h_n < 0.2, f'Bbox fora do esperado: ({w_n:.3f},{h_n:.3f})'

    print('  Projecao simple + with_camera + bbox: OK')


if __name__ == '__main__':
    print('=== Teste E2E: Dataset Collector ===')

    print('\n[1/3] Validando geometria de projeção...')
    test_projection_geometry()

    print('\n[2/3] Gerando dataset sintético (60 frames)...')
    base = generate_dataset(n_frames=60)

    print('\n[3/3] Validando estrutura e anotações...')
    validate_dataset(base)

    print('\nDataset pronto para treino YOLO:')
    print('  yolo detect train data=dataset.yaml model=yolov8n.pt epochs=50 imgsz=640')
