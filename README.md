# CERISE Multi-Robot Nav2 + YOLO Dataset

**Gêmeo Digital para Cenário Multi-Robô via Detecção YOLO**

Simula 2 TurtleBot3 Waffle com navegação autônoma (Nav2), coleta dataset de imagens com anotações de posição e treina YOLO v8 para estimar posição dos robôs somente pela imagem da câmera.

## Objetivo (Prof. Alisson)

1. ✅ Simular 2 TurtleBot3 Waffle com navegação autônoma Nav2
2. ✅ Câmera overhead capturando posições dos robôs com projeção correta
3. ✅ Pipeline de coleta de dataset com anotações YOLO
4. ⏳ Treinar YOLO v8 com dataset real e validar inferência end-to-end

## Status

| Fase | Descrição | Status |
|------|-----------|--------|
| 1 | Simulação 2-robot + Nav2 | ✅ Funcional |
| 2 | Câmera overhead + projeção world→pixel | ✅ Implementado |
| 3 | Dataset collector + anotações YOLO | ✅ Implementado |
| 4 | Treino YOLO + validação gêmeo digital | ⏳ Requer dataset real |

## Stack

| Componente | Versão |
|-----------|--------|
| OS | Ubuntu 22.04 (WSL2 ou nativo) |
| ROS2 | Humble |
| Gazebo | Classic 11.10.2 |
| Nav2 | Humble |
| YOLO | Ultralytics v8 |

## Quick Start (WSL2)

### 1. Instalar dependências e build

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo
pip install ultralytics opencv-python numpy

git clone https://github.com/santtyan/cerise-turtlebot3-nav.git
cd cerise-turtlebot3-nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Rodar simulação com câmera

```bash
# Terminal 1 — Gazebo + 2 robots + câmera overhead
./launch_2robots_with_camera.sh

# Terminal 2 — Definir poses iniciais (aguardar ~10s)
./set_initialposes.sh

# Terminal 3 — Coletar dataset
ros2 run cerise_nav dataset_collector
```

O collector salva automaticamente em `dataset/raw/images/` e `dataset/raw/annotations/` a 1 fps.

### 3. Testar pipeline sem simulação

```bash
PYTHONPATH=src/cerise_nav python test_e2e_dataset_collector.py
```

Gera 60 frames sintéticos com trajetórias circulares, valida projeção (simple + with_camera) e formato YOLO.

### 4. Split train/val e treino

```bash
# Separa dataset/raw/ em dataset/images/{train,val} + dataset/labels/{train,val}
python scripts/split_dataset.py --ratio 0.8

# Treina YOLOv8
yolo detect train data=dataset.yaml model=yolov8n.pt epochs=100 imgsz=640
```

### 5. Validar inferência

```bash
yolo detect predict model=runs/detect/train/weights/best.pt source=dataset/images/val/
```

## Arquitetura

```
gzserver (Gazebo)
  ├─ /robot1/odom, scan, amcl_pose
  ├─ /robot2/odom, scan, amcl_pose
  └─ /camera/image_raw, camera_info   ← câmera overhead z=3m FOV=60°

Nav2 (robot1 + robot2 namespaces)
  └─ NavigateToPose → trajetórias autônomas

dataset_collector (ROS2 node)
  ├─ Subscreve: image_raw + camera_info + amcl_pose
  ├─ Projeta poses world→pixel (pinhole com intrínsecos reais)
  └─ Salva: dataset/images/ + dataset/annotations/ (formato YOLO)

YOLO v8 (treinamento offline)
  └─ best.pt → inferência: câmera → posição estimada dos robôs
```

## Projeção World → Pixel

A câmera está em `z=3m` com FOV horizontal de 60° (1.047 rad), o que cobre ~**3.46m** ao redor do centro. A projeção usa os intrínsecos reais do sensor (`/camera/camera_info`) via modelo pinhole:

```python
u = fx * (world_x / camera_height) + cx_principal
v = fy * (-world_y / camera_height) + cy_principal  # Y invertido
```

O bounding box é calculado dinamicamente com base no raio real do robô (0.17m) e na altura da câmera, garantindo anotações precisas independente da posição.

## Estrutura do Projeto

```
cerise-turtlebot3-nav/
├── src/cerise_nav/cerise_nav/
│   ├── dataset_collector.py   # Node ROS2: coleta imagens + anotações
│   ├── projection.py          # Projeção world→pixel (pinhole + fallback)
│   ├── demand_generator.py    # Gerador de missões de navegação
│   └── task_allocator.py      # Alocador de tarefas multi-robô
├── world_with_camera.model    # Mundo Gazebo com câmera overhead
├── waffle_nodepth.model       # TurtleBot3 sem câmera de profundidade (WSL2)
├── launch_2robots.sh          # Headless — sem câmera
├── launch_2robots_with_camera.sh  # Headless + câmera overhead
├── run_gui.sh                 # Interface gráfica Gazebo
├── set_initialposes.sh        # Inicializa poses AMCL
├── test_e2e_dataset_collector.py  # Teste sintético do pipeline
└── dataset.yaml               # Configuração YOLO (train/val split)
```

## Erros Comuns

### `spawn_entity timeout`
```bash
export GAZEBO_MODEL_DATABASE_URI=""   # desativa download de modelos
```

### `gzserver SIGSEGV` no WSL2
Câmera RGB-D tenta inicializar OpenGL sem display.
Solução: o modelo `waffle_nodepth.model` já remove a câmera de profundidade.

### `ros2 run cerise_nav dataset_collector: not found`
```bash
colcon build --symlink-install && source install/setup.bash
```

### Tópico `/camera/image_raw` vazio
Verificar se `world_with_camera.model` está sendo usado (não o mundo padrão do TurtleBot3).

## Limitação Conceitual: Identificação de Robôs Idênticos

YOLO detecta "há um robô aqui" e "há outro ali", mas **não distingue** robot1 de robot2 se são fisicamente idênticos. Por isso o projeto usa uma única classe `robot` no YAML. Para recuperar a identidade de cada robô a partir da detecção, há três abordagens possíveis:

1. **Tracking temporal** (mais simples): associar cada detecção ao robô cuja última pose conhecida está mais próxima
2. **Coloração diferenciada**: modificar o SDF para pintar robot1 de azul e robot2 de vermelho, depois treinar com 2 classes
3. **Marcadores visuais**: ArUco/AprilTag em cada robô, detectado junto com YOLO

Para o artigo no LARS, a abordagem (1) é suficiente e preserva a hipótese de "robôs reais idênticos em campo".

## Próximas Etapas

- [ ] Coletar dataset real com robôs em navegação ativa (meta: 500+ frames)
- [ ] Treinar YOLOv8 com dataset real e avaliar mAP
- [ ] Nó de inferência em tempo real: `/camera/image_raw` → posição estimada no mapa
- [ ] Associação temporal detecção→identidade robô via Hungarian matching com pose anterior
- [ ] Comparar posição estimada (YOLO) vs posição real (AMCL) — erro médio como métrica do gêmeo digital

## Referências

- [Nav2 Multi-Robot Tutorial](https://navigation.ros.org/)
- [Ultralytics YOLOv8 Docs](https://docs.ultralytics.com/)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

## License

Apache 2.0
