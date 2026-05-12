#!/usr/bin/env python3
"""
Visualiza frames do dataset com bboxes YOLO desenhados.
Útil para validar que o bbox está alinhado sobre o robô durante movimento.

Uso:
    python3 scripts/visualize_dataset.py [frame_id]

Exemplos:
    python3 scripts/visualize_dataset.py 0         # Primeiro frame
    python3 scripts/visualize_dataset.py 500       # Frame 500
    python3 scripts/visualize_dataset.py random    # Frame aleatório
"""

import sys
import os
import cv2
import numpy as np
from pathlib import Path
import random


def denormalize_bbox(cx_norm, cy_norm, w_norm, h_norm, img_w, img_h):
    """Converte bbox YOLO normalizado para pixel coordinates (x1, y1, x2, y2)."""
    x_center = cx_norm * img_w
    y_center = cy_norm * img_h
    w_pixels = w_norm * img_w
    h_pixels = h_norm * img_h

    x1 = int(x_center - w_pixels / 2)
    y1 = int(y_center - h_pixels / 2)
    x2 = int(x_center + w_pixels / 2)
    y2 = int(y_center + h_pixels / 2)

    return x1, y1, x2, y2


def visualize_frame(frame_id):
    """Desenha bboxes YOLO sobre a imagem e mostra."""
    img_path = f'dataset/raw/images/{frame_id:06d}.jpg'
    ann_path = f'dataset/raw/annotations/{frame_id:06d}.txt'

    if not os.path.exists(img_path) or not os.path.exists(ann_path):
        print(f"❌ Frame {frame_id:06d} não encontrado")
        return False

    img = cv2.imread(img_path)
    img_h, img_w = img.shape[:2]

    # Ler anotações YOLO
    with open(ann_path, 'r') as f:
        lines = f.readlines()

    print(f"✅ Frame {frame_id:06d}: {len(lines)} robôs visíveis")

    # Desenhar bboxes
    colors = [(0, 255, 0), (0, 165, 255)]  # Verde, Laranja
    for i, line in enumerate(lines):
        parts = line.strip().split()
        class_id = int(parts[0])
        cx_norm = float(parts[1])
        cy_norm = float(parts[2])
        w_norm = float(parts[3])
        h_norm = float(parts[4])

        x1, y1, x2, y2 = denormalize_bbox(cx_norm, cy_norm, w_norm, h_norm, img_w, img_h)

        color = colors[i % len(colors)]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f'robot{i+1}', (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Mostrar (ou salvar para debug)
    out_path = f'/tmp/annotated_{frame_id:06d}.jpg'
    cv2.imwrite(out_path, img)
    print(f"📸 Anotado salvo: {out_path}")

    # Tentar abrir com visualizador (opcional)
    try:
        import subprocess
        subprocess.Popen(['eog', out_path])  # Eye of GNOME
    except:
        print(f"   (Abra manualmente: eog {out_path} ou display {out_path})")

    return True


def main():
    os.chdir('/home/yan/Documentos/Projetos/cerise-turtlebot3-nav')

    if not os.path.exists('dataset/raw/images'):
        print("❌ dataset/raw/images não encontrado")
        return

    # Contar frames
    frames = sorted([f for f in os.listdir('dataset/raw/images') if f.endswith('.jpg')])
    if not frames:
        print("❌ Nenhum frame encontrado")
        return

    max_frame = len(frames)
    print(f"📊 Total de frames coletados: {max_frame}")

    # Determinar qual frame visualizar
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg == 'random':
            frame_id = random.randint(0, max_frame - 1)
            print(f"🎲 Frame aleatório: {frame_id}")
        else:
            frame_id = int(arg)
            if frame_id >= max_frame:
                print(f"❌ Frame {frame_id} fora do range [0, {max_frame-1}]")
                return
    else:
        # Visualizar alguns frames estratégicos
        print("\n📋 Visualizando frames estratégicos:")
        for frame_id in [0, max_frame // 4, max_frame // 2, max_frame - 1]:
            visualize_frame(frame_id)
            print()
        return

    visualize_frame(frame_id)


if __name__ == '__main__':
    main()
