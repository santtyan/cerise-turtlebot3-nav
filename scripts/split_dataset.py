#!/usr/bin/env python3
"""
Separa dataset bruto coletado em train/val com ratio configurável.

Entrada esperada:
  dataset/raw/images/NNN.jpg
  dataset/raw/annotations/NNN.txt

Saída:
  dataset/images/train/NNN.jpg
  dataset/images/val/NNN.jpg
  dataset/labels/train/NNN.txt    (YOLO espera 'labels/', não 'annotations/')
  dataset/labels/val/NNN.txt

Uso:
  python scripts/split_dataset.py [--ratio 0.8] [--seed 42]
"""

import argparse
import random
import shutil
import sys
from pathlib import Path

import cv2


def split(ratio: float, seed: int, base: Path, rotate: int) -> tuple:
    raw_images = base / 'raw' / 'images'
    raw_labels = base / 'raw' / 'annotations'

    if not raw_images.exists() or not raw_labels.exists():
        print(f'[ERRO] Esperava {raw_images} e {raw_labels}', file=sys.stderr)
        print('Colete dados primeiro com: ros2 run cerise_nav dataset_collector', file=sys.stderr)
        sys.exit(1)

    images = sorted(raw_images.glob('*.jpg'))
    if not images:
        print(f'[ERRO] Nenhuma imagem em {raw_images}', file=sys.stderr)
        sys.exit(1)

    # Garante que cada imagem tem anotação correspondente
    paired = []
    for img in images:
        ann = raw_labels / f'{img.stem}.txt'
        if ann.exists():
            paired.append((img, ann))
        else:
            print(f'[WARN] Sem anotacao para {img.name} — ignorado')

    random.Random(seed).shuffle(paired)
    n_train = int(len(paired) * ratio)
    train = paired[:n_train]
    val   = paired[n_train:]

    rot_map = {90: cv2.ROTATE_90_CLOCKWISE,
               180: cv2.ROTATE_180,
               270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    rot_cv = rot_map[rotate]

    # YOLO v8 espera: {dataset}/images/{split}/ e {dataset}/labels/{split}/
    for split_name, pairs in (('train', train), ('val', val)):
        img_dir = base / 'images' / split_name
        lbl_dir = base / 'labels' / split_name
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)

        for img, ann in pairs:
            im = cv2.imread(str(img))
            im_rot = cv2.rotate(im, rot_cv)
            cv2.imwrite(str(img_dir / img.name), im_rot)

            new_lines = []
            for line in ann.read_text().strip().splitlines():
                parts = line.split()
                if not parts:
                    continue
                cls = parts[0]
                x, y, w, h = map(float, parts[1:5])
                if rotate == 90:
                    nx, ny, nw, nh = 1 - y, x, h, w
                elif rotate == 180:
                    nx, ny, nw, nh = 1 - x, 1 - y, w, h
                else:  # 270
                    nx, ny, nw, nh = y, 1 - x, h, w
                new_lines.append(f'{cls} {nx:.6f} {ny:.6f} {nw:.6f} {nh:.6f}')
            (lbl_dir / ann.name).write_text('\n'.join(new_lines) + '\n')

    return len(train), len(val)


def main():
    p = argparse.ArgumentParser(description='Split dataset YOLO em train/val')
    p.add_argument('--ratio', type=float, default=0.8, help='Fração para treino (default: 0.8)')
    p.add_argument('--seed', type=int, default=42, help='Seed random (default: 42)')
    p.add_argument('--dataset', type=Path, default=Path('dataset'), help='Diretorio do dataset')
    p.add_argument('--rotate', type=int, default=90, choices=[90, 180, 270],
                   help='Rotacao em graus aplicada a imagem e bbox (default: 90)')
    args = p.parse_args()

    if not 0.1 <= args.ratio <= 0.95:
        print(f'[ERRO] ratio deve estar em [0.1, 0.95], recebido {args.ratio}', file=sys.stderr)
        sys.exit(1)

    n_train, n_val = split(args.ratio, args.seed, args.dataset, args.rotate)
    total = n_train + n_val
    print(f'Split concluido: {n_train} train / {n_val} val (total {total}, ratio {n_train/total:.2f})')
    print(f'Estrutura YOLO pronta em: {args.dataset}/images/ e {args.dataset}/labels/')


if __name__ == '__main__':
    main()
