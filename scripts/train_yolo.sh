#!/bin/bash
# Treino YOLO otimizado para detecção de robôs overhead.
# Hyperparams escolhidos para 1 classe + dataset pequeno (~300 frames) + reprodutibilidade.

cd /home/yan/Documentos/Projetos/cerise-turtlebot3-nav

yolo train \
  data=dataset.yaml \
  model=yolov8n.pt \
  epochs=50 \
  patience=15 \
  batch=8 \
  imgsz=416 \
  lr0=0.001 \
  lrf=0.01 \
  warmup_epochs=3 \
  freeze=10 \
  optimizer=AdamW \
  cos_lr=True \
  seed=42 \
  deterministic=True \
  mosaic=0.5 \
  mixup=0.0 \
  degrees=180 \
  flipud=0.5 \
  fliplr=0.5 \
  hsv_h=0.015 \
  hsv_s=0.7 \
  hsv_v=0.4 \
  device=cpu \
  workers=6 \
  project=runs/detect \
  name=train_optimized \
  exist_ok=True
