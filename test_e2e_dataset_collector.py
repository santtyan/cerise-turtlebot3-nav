#!/usr/bin/env python3
"""
Test end-to-end do dataset_collector.
Mock: imagens aleatórias + poses fixas de robôs.
"""

import cv2, os, numpy as np
from pathlib import Path
from cerise_nav.projection import world_to_pixel

def test_dataset_collection():
    """Simula coleta de dataset com poses conhecidas."""
    
    # Setup
    os.makedirs('dataset/images', exist_ok=True)
    os.makedirs('dataset/annotations', exist_ok=True)
    
    # Poses conhecidas dos robôs durante navegação
    poses_seq = [
        {'robot1': (0.0, 0.5), 'robot2': (0.0, -0.5)},   # início
        {'robot1': (0.5, 0.5), 'robot2': (0.2, -0.7)},   # movimento 1
        {'robot1': (1.0, 1.0), 'robot2': (0.4, -1.0)},   # goal robot1
        {'robot1': (1.0, 1.0), 'robot2': (0.6, -0.5)},   # robot2 em movimento
        {'robot1': (1.0, 1.0), 'robot2': (1.0, 0.0)},    # fim
    ]
    
    for frame_id, poses in enumerate(poses_seq):
        # Gera frame fake (em simulação real seria /camera/image_raw)
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Desenha círculos nas posições dos robôs (para inspeção visual)
        for robot_id, (x, y) in poses.items():
            cx_norm, cy_norm = world_to_pixel(x, y)
            px = int(cx_norm * 640)
            py = int(cy_norm * 480)
            cv2.circle(img, (px, py), 20, (0, 255, 0), 2)  # robot1 verde
        
        # Salva imagem
        img_path = f'dataset/images/{frame_id:06d}.jpg'
        cv2.imwrite(img_path, img)
        
        # Salva anotação YOLO
        ann_path = f'dataset/annotations/{frame_id:06d}.txt'
        with open(ann_path, 'w') as f:
            for robot_id, (x, y) in enumerate(poses.values()):
                cx, cy = world_to_pixel(x, y)
                w, h = 0.05, 0.05
                f.write(f'{robot_id} {cx:.4f} {cy:.4f} {w:.4f} {h:.4f}\n')
        
        print(f'Frame {frame_id}: robot1{poses["robot1"]} → pixel {world_to_pixel(*poses["robot1"])}')
    
    # Validação
    n_imgs = len(list(Path('dataset/images').glob('*.jpg')))
    n_anns = len(list(Path('dataset/annotations').glob('*.txt')))
    
    assert n_imgs == len(poses_seq), f'Imagens: esperado {len(poses_seq)}, obtido {n_imgs}'
    assert n_anns == len(poses_seq), f'Anotações: esperado {len(poses_seq)}, obtido {n_anns}'
    
    # Lê uma anotação para validar formato
    with open('dataset/annotations/000000.txt') as f:
        lines = f.readlines()
        assert len(lines) == 2, f'Esperado 2 robôs, obtido {len(lines)}'
        for line in lines:
            parts = line.strip().split()
            assert len(parts) == 5, f'YOLO format inválido: {parts}'
            class_id, cx, cy, w, h = map(float, parts)
            assert 0 <= cx <= 1 and 0 <= cy <= 1, f'Coords fora do range: {cx}, {cy}'
    
    print(f'✅ Dataset válido: {n_imgs} frames + {n_anns} anotações YOLO')
    return True

if __name__ == '__main__':
    test_dataset_collection()
