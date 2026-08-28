#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path

# Configurações
img_dir = Path("dataset/images/train")
label_dir = Path("dataset/labels/train")
output_dir = Path("dataset/bbox_preview")
output_dir.mkdir(exist_ok=True)

# Selecionar 6 imagens aleatórias para preview
images = sorted(list(img_dir.glob("*.jpg")))[:6]

for img_path in images:
    label_path = label_dir / img_path.stem / ".txt"
    label_path = label_dir / (img_path.stem + ".txt")

    # Ler imagem
    img = cv2.imread(str(img_path))
    if img is None:
        continue

    h, w = img.shape[:2]

    # Ler labels
    if label_path.exists():
        with open(label_path) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 5:
                    continue

                class_id = int(parts[0])
                x_center = float(parts[1]) * w
                y_center = float(parts[2]) * h
                box_w = float(parts[3]) * w
                box_h = float(parts[4]) * h

                # Desenhar bbox
                x1 = int(x_center - box_w / 2)
                y1 = int(y_center - box_h / 2)
                x2 = int(x_center + box_w / 2)
                y2 = int(y_center + box_h / 2)

                # Clampar aos limites da imagem
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(w, x2), min(h, y2)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"robot {class_id}", (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Salvar preview
    output_path = output_dir / f"{img_path.stem}_bbox.jpg"
    cv2.imwrite(str(output_path), img)
    print(f"✅ {output_path}")

print(f"\n📊 Previews salvos em: {output_dir}")
