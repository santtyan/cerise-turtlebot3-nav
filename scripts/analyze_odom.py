#!/usr/bin/env python3
"""
Analisa o arquivo de anotações raw para entender como o odômetro está se comportando.
Se ROBOT_SPAWN está correto, então p.x + spawn_x deveria dar a pose correta.
"""
from pathlib import Path
import json

print("=== Análise: Como os robôs se movem no odômetro ===\n")

label_dir = Path("dataset/raw/annotations")
if not label_dir.exists():
    print("❌ Nenhum dataset coletado ainda")
    exit(1)

# Ler todos os frames em sequência
labels = sorted(list(label_dir.glob("*.txt")))
print(f"Analisando {len(labels)} frames...\n")

from cerise_nav.projection import pixel_to_world_simple

CAMERA_HEIGHT = 3.0
HORIZONTAL_FOV = 1.047
IMG_W = 640
IMG_H = 480

# Reconstruir trajetória dos robôs
trajectories = {'robot1': [], 'robot2': []}

for frame_idx, label_path in enumerate(labels[:10]):  # Primeiros 10 frames
    with open(label_path) as f:
        lines = f.readlines()

    print(f"Frame {frame_idx:03d}: {len(lines)} robôs visíveis")
    for robot_idx, line in enumerate(lines):
        parts = line.strip().split()
        cx, cy = float(parts[1]), float(parts[2])

        # Reverter para mundo
        world_x, world_y = pixel_to_world_simple(cx, cy, CAMERA_HEIGHT, HORIZONTAL_FOV, IMG_W, IMG_H)
        print(f"  Robô {robot_idx}: pixel({cx:.3f}, {cy:.3f}) → mundo({world_x:.3f}, {world_y:.3f})")

print("\n" + "="*60)
print("INTERPRETAÇÃO:")
print("="*60)
print("""
Se você vê que o robô fica em mundo(-0.6, -1.3), mas deveria estar em (0.5, 0.0),
então há 2 possibilidades:

1. **Odômetro começa em (0, 0) → offset não está sendo somado**
   - Solução: Verificar se dataset_collector.py está realmente somando spawn_x, spawn_y

2. **Odômetro está desorientado (frame diferente)**
   - A pose do odômetro é em relação ao chassi, não ao mundo
   - Solução: Desabilitar a soma de offset

3. **Spawn positions estão erradas**
   - Lançamento do Gazebo não está spawneando em (0, 0.5)
   - Solução: Verificar launch file

4. **Projeção está invertida**
   - BBox sendo calculado com inversão de eixo incorreta
   - Solução: Ajustar inversão Y na projeção
""")
