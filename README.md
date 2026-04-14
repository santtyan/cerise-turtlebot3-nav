# CERISE Multi-Robot Nav2 + YOLO Dataset

Multi-robot autonomous navigation com ROS2 Humble + Nav2 + TurtleBot3.
Branch `feature/yolo-dataset`: coleta de dataset para detecção de posição de robôs via YOLO.

## Objetivo (Prof. Alisson)

1. Simular cenário multi-robô (2x TurtleBot3 Waffle)
2. Capturar imagens com posições anotadas dos robôs
3. Treinar YOLO v8 para detectar posição dos robôs pela imagem
4. Validar gêmeo digital: câmera → inferência → posição estimada

## Quick Start (WSL Headless)

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_DATABASE_URI=""
source /opt/ros/humble/setup.bash

# Terminal 1 - Simulação
./launch_2robots.sh

# Terminal 2 - Poses iniciais
./set_initialposes.sh

# Terminal 3 - Dataset collector
ros2 run cerise_nav dataset_collector
```

## Stack

| Componente | Versão |
|---|---|
| OS | Ubuntu 22.04 (WSL2 / nativo) |
| ROS2 | Humble |
| Gazebo | Classic 11.10.2 |
| Nav2 | Humble |
| YOLO | Ultralytics v8 (próx. etapa) |

## Arquitetura

```
gzserver (headless) → /robot1/odom, scan
                    → /robot2/odom, scan
                            │
                    Nav2 (robot1 + robot2 namespaces)
                            │
              demand_generator → task_allocator → NavigateToPose
                            │
                  dataset_collector → dataset/images/ + annotations/
                            │
                      YOLO v8 training
```

## Avanços

- [x] WSL2 com ROS2 Humble + Gazebo + Nav2
- [x] gzserver headless funcional (GAZEBO_MODEL_DATABASE_URI fix)
- [x] Spawn 2 TurtleBots com namespaces /robot1, /robot2
- [x] Nav2 action server ativo
- [x] Script launch_2robots.sh (sequência correta gzserver→spawn→nav2)
- [x] Estrutura dataset_collector.py
- [ ] Navegação e2e (gzserver crash WSL por OpenGL/ALSA - ver Erros)
- [ ] Camera overhead no mundo Gazebo
- [ ] Projeção mapa→pixel para anotações YOLO

## Principais Erros e Aprendizados

### 1. spawn_entity timeout
**Problema**: gzserver baixa modelos da internet, excede timeout 30s do spawn_entity.
**Fix**: `export GAZEBO_MODEL_DATABASE_URI=` + sequenciar gzserver antes do spawn.

### 2. headless:=True não funciona
**Problema**: o launch file usa variável `simulator`, não `headless`.
**Fix**: usar `simulator:=gzserver` OU `launch_2robots.sh`.

### 3. TF namespace multi-robô
**Problema**: `--ros-args -r __ns:=/robot1` namespeia tópicos mas não frames TF.
**Fix**: `tb3_simulation_launch.py` com `namespace:=robot1` + remapping `('/tf', 'tf')`.

### 4. gzserver SIGSEGV no WSL2
**Problema**: câmera de profundidade do waffle tenta inicializar OpenGL sem display.
**Fix parcial**: modelo `waffle_nodepth.model` sem câmera. Recomendado testar em máquina Ubuntu nativa (INCOMM).

### 5. CycloneDDS ausente
**Problema**: `rmw_cyclonedds_cpp` não instalado por padrão.
**Fix**: remover `RMW_IMPLEMENTATION`, usar FastRTPS (padrão Humble).

## Dataset YOLO (Estrutura Planejada)

```
dataset/
├── images/          # frames .jpg (1 fps durante navegação)
└── annotations/     # YOLO format: class cx cy w h (normalizado)

classes.txt → 0: robot
```

**TODO**: adicionar câmera overhead (`camera_overhead.world`) no mundo Gazebo e implementar projeção mapa→pixel em `dataset_collector.py`.

## Gitflow

```
main ──────── 4-robot WIP
antiga ─────── 2-robot validado (base desta branch)
feature/yolo-dataset ← VOCÊ ESTÁ AQUI
```

## Referências

- [Nav2 Multi-Robot Tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_multiple_robots.html)
- [Ultralytics YOLOv8 Docs](https://docs.ultralytics.com/)
- [TurtleBot3 Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
