#!/usr/bin/env python3
"""
Debug script: verifica se as poses computadas fazem sentido.
Lê os labels YOLO salvos e reconverte para mundo para verificar consistência.
"""
import numpy as np
from pathlib import Path
from cerise_nav.projection import pixel_to_world_simple, world_to_pixel_simple

# Parâmetros câmera
CAMERA_HEIGHT = 3.0
HORIZONTAL_FOV = 1.047  # 60°
IMG_W = 640
IMG_H = 480

# Rodar uns testes round-trip
print("=== Teste round-trip: world → pixel → world ===\n")

test_poses = [
    (0.0, 0.0, "Centro (origem)"),
    (0.5, 0.0, "Robot1 esperado"),
    (-0.5, 0.0, "Robot2 esperado"),
    (0.0, 0.5, "Deslocamento +Y"),
    (0.0, -0.5, "Deslocamento -Y"),
]

for world_x, world_y, desc in test_poses:
    # Forward: world → pixel
    px, py = world_to_pixel_simple(world_x, world_y, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)

    # Reverse: pixel → world
    wx, wy = pixel_to_world_simple(px, py, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)

    print(f"{desc}:")
    print(f"  World: ({world_x:.2f}, {world_y:.2f})")
    print(f"  Pixel: ({px:.3f}, {py:.3f}) = ({int(px*IMG_W)}, {int(py*IMG_H)}) pixels")
    print(f"  Back→World: ({wx:.2f}, {wy:.2f})")
    print(f"  Error: {abs(wx-world_x):.4f}, {abs(wy-world_y):.4f}")
    print()

print("\n=== Análise dos labels salvos ===\n")

label_dir = Path("dataset/raw/annotations")
if label_dir.exists():
    labels = sorted(list(label_dir.glob("*.txt")))[:3]  # Primeiros 3

    for label_path in labels:
        with open(label_path) as f:
            lines = f.readlines()

        print(f"{label_path.name}:")
        for line in lines:
            parts = line.strip().split()
            class_id = int(parts[0])
            cx, cy = float(parts[1]), float(parts[2])
            w, h = float(parts[3]), float(parts[4])

            # Reverter para mundo
            world_x, world_y = pixel_to_world_simple(cx, cy, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
            print(f"  BBox norm: ({cx:.3f}, {cy:.3f}) | Size: ({w:.3f}x{h:.3f})")
            print(f"  ← Mundo: ({world_x:.3f}, {world_y:.3f})")
        print()
else:
    print("❌ dataset/raw/annotations/ não encontrado")
