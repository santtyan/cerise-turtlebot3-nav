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

### 2. Rodar simulação com câmera + Nav2

```bash
# Terminal 1 — Gazebo + 2 robots + Nav2 + initial poses (automático, ~60s)
./launch_2robots_with_camera.sh
# Aguarda "[OK] Nav2 pronto!" antes de abrir outros terminais

# Terminal 2 — Goals aleatórios (robôs navegam autonomamente)
source /opt/ros/humble/setup.bash
python3 scripts/random_nav_goals.py

# Terminal 3 — Coletar dataset (~10 min)
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run cerise_nav dataset_collector

# Terminal 4 — Gazebo GUI (opcional, só quando pipeline estável)
gzclient
```

O `launch_2robots_with_camera.sh` já publica as initial poses automaticamente — **não** é necessário rodar `set_initialposes.sh` separadamente.

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
├── world_with_camera.model    # Mundo Gazebo com câmera overhead (ATENÇÃO: causa segfault — ver Erros Comuns)
├── world_simple.model         # Mundo estável para visualização GUI (sem plugin câmera)
├── waffle_nodepth.model       # TurtleBot3 sem câmera de profundidade (WSL2)
├── launch_2robots.sh          # Headless — sem câmera
├── launch_2robots_with_camera.sh  # Headless + câmera overhead
├── run_gui.sh                 # Interface gráfica Gazebo
├── set_initialposes.sh        # Inicializa poses AMCL
├── test_e2e_dataset_collector.py  # Teste sintético do pipeline
└── dataset.yaml               # Configuração YOLO (train/val split)
```

## Visualização Gazebo (Linux Nativo)

Para visualizar com GUI no Linux nativo (com mapa + robôs visíveis):

```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -e ode ./world_simple.model &
sleep 20
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file ./waffle_nodepth.model -robot_namespace robot1 -x 0.0 -y 0.5 -z 0.01 &
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file ./waffle_nodepth.model -robot_namespace robot2 -x 0.0 -y -0.5 -z 0.01 &
sleep 10
export DISPLAY=:0
gzclient &
```

## Lições Aprendidas (Multi-Robot Nav2)

### TF Multi-Robot: cadeia obrigatória

Para Nav2 aceitar goals, a cadeia TF dentro de `/robot1/tf` deve ser completa:
```
map → odom → base_footprint → base_link
```
- `odom → base_footprint`: publicado pelo diff_drive plugin do Gazebo
- `base_footprint → base_link`: publicado pelo robot_state_publisher
- `map → odom`: publicado pelo AMCL **somente após receber initial_pose**

### AMCL não publica map→odom sem initial_pose

O AMCL fica aguardando indefinidamente sem publicar `map→odom` até receber uma mensagem em `/robot1/initialpose`. Goals são rejeitados até essa mensagem chegar.

```bash
ros2 topic pub --once /robot1/initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

O `launch_2robots_with_camera.sh` já faz isso automaticamente após 45s.

### Câmera overhead: pitch correto

- `pitch=0` → câmera aponta para frente (+x) ❌
- `pitch=1.5708` → câmera aponta para baixo (-z) ✅

```xml
<pose>0 0 3 0 1.5708 0</pose>
```

### Modelo do robô para spawn

Usar **sempre** `/opt/ros/humble/share/nav2_bringup/worlds/waffle.model` para spawn. O `waffle_nodepth.model` tem bug no plugin LiDAR (`libgazebo_ros_ray_sensor`) que causa publisher_count=0 no `/robot1/scan`, impedindo AMCL de localizar.

### Nav2 com namespace: remapping TF

RSP com namespace `robot1` deve remapear `/tf` e `/tf_static`:
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -r __ns:=/robot1 -r /tf:=tf -r /tf_static:=tf_static \
  -p robot_description:="$(cat $URDF)" -p use_sim_time:=true
```

### Goals aleatórios Nav2

`scripts/random_nav_goals.py` envia goals via `NavigateToPose` action para ambos os robôs em loop. Waypoints definidos dentro do labirinto `turtlebot3_world`.

### Projeção world→pixel: mapeamento de eixos com pitch=π/2

Com a câmera em `<pose>0 0 3 0 1.5708 0</pose>` (pitch=π/2), a rotação R_y(π/2) transforma os eixos assim:

- `X_cam = +Y_world` → `cam_x = world_y`
- `Y_cam = -X_world` → `cam_y = -world_x`
- `Z_cam = camera_height` (profundidade)

```python
# projection.py — correto para pitch=π/2
cam_x = world_y
cam_y = -world_x
cam_z = camera_height
u = fx * (cam_x / cam_z) + cx
v = fy * (cam_y / cam_z) + cy
```

**Erro comum:** usar `cam_x = world_x` e `cam_y = -world_y` (mapeamento sem rotação). Os bboxes aparecem espelhados horizontalmente em relação aos robôs reais.

### Dataset: invalidação por bug de projeção

Se o `projection.py` estava errado durante a coleta, **todo o dataset deve ser descartado e recoletado** — as anotações `.txt` ficam erradas mesmo que as imagens `.jpg` estejam corretas. Para re-coletar:

```bash
rm -rf dataset/raw/images/* dataset/raw/annotations/*
ros2 run cerise_nav dataset_collector
```

### Parâmetros de treino YOLO validados

Para datasets pequenos (150–500 frames) com uma única classe:

```bash
yolo detect train data=dataset.yaml model=yolov8n.pt \
  epochs=50 batch=8 freeze=10 patience=20 imgsz=640
```

- `freeze=10`: congela backbone pré-treinado, treina só o head (evita overfitting)
- `patience=20`: early stopping se não melhorar em 20 épocas
- `batch=8`: compatível com GPUs com 4–8GB VRAM

## Erros Comuns

### `spawn_entity timeout`
```bash
export GAZEBO_MODEL_DATABASE_URI=""   # desativa download de modelos
```

### `gzserver SIGSEGV` (Segmentation Fault) com `world_with_camera.model`
**Causa:** O plugin `libgazebo_ros_camera.so` definido em `world_with_camera.model` causa segmentation fault no gzserver ao tentar inicializar o driver de câmera. Isso ocorre tanto no WSL2 (sem display) quanto em Linux nativo, provavelmente por incompatibilidade entre a versão do Gazebo Classic 11 e o plugin ROS2 Humble.

**Solução:** Usar `world_simple.model` para visualização GUI. Ele contém o mapa `turtlebot3_world` com câmera posicionada overhead, mas **sem o plugin ROS** que causava o crash.

```bash
# Em vez de world_with_camera.model, use:
gzserver ... ./world_simple.model
```

### Mapa não aparece no Gazebo (mundo vazio)
**Causa:** O modelo `turtlebot3_world` não é encontrado porque `GAZEBO_MODEL_PATH` não inclui o diretório de modelos do TurtleBot3.

**Solução:**
```bash
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
```

### `gzserver EXCEPTION: Unable to start server [bind: Address already in use]`
**Causa:** Instância anterior do gzserver ainda em execução.

**Solução:**
```bash
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"
pkill -9 -f spawn_entity
sleep 8
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

## Resultados Obtidos

| Métrica | Valor |
|---------|-------|
| Frames coletados (Nav2 autônomo) | ~891 frames |
| Split treino/val | 80/20 |
| Projeção world→pixel | Pinhole com intrínsecos reais (`/camera/camera_info`) |
| Raio do robô no bbox | 0.17m (TurtleBot3 Waffle) |
| Diversidade de poses | Nav2 navegação autônoma com goals aleatórios |

## Próximas Etapas

- [ ] Re-coletar dataset com projeção corrigida e validar bboxes com `verify_bboxes_gui.py`
- [ ] Treinar YOLOv8 (epochs=50, batch=8, freeze=10, patience=20)
- [ ] Avaliar mAP no conjunto de validação
- [ ] Nó de inferência em tempo real: `/camera/image_raw` → posição estimada no mapa
- [ ] Associação temporal detecção→identidade robô via Hungarian matching com pose anterior
- [ ] Comparar posição estimada (YOLO) vs posição real (AMCL) — erro médio como métrica do gêmeo digital

## Referências

- [Nav2 Multi-Robot Tutorial](https://navigation.ros.org/)
- [Ultralytics YOLOv8 Docs](https://docs.ultralytics.com/)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

## License

Apache 2.0
