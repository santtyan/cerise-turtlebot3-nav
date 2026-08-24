# Diagnóstico: Problema de Projeção de Câmera em Gazebo + ROS2

**Relatório de pesquisa profunda sobre bbox desalinhadas em imagens**

Data: 2026-05-01  
Status: Investigação ativa de problema recorrente (2x em histórico de commits)

---

## Resumo Executivo

O problema de bboxes desalinhadas ocorreu 2 vezes no histórico:

1. **Commit 560558b (2026-04-28)**: Adicionava `ROBOT_SPAWN` offset às poses de odometria
   - **Causa**: Interpretação errada de que odom era relativo ao spawn
   - **Realidade**: Odom JÁ publica coordenadas MUNDO, não locais
   - **Resultado do erro**: Bbox desenhado 2x mais longe + offset incorreto

2. **Commit HEAD (atual)**: Reverteu o offset, mas comentário sugere dúvida persistente
   - Status atual: Parece correto, mas precisão não validada experimentalmente

**O problema raiz é que NÃO HÁ MECANISMO DE VALIDAÇÃO implementado para verificar se a projeção está correta.**

---

## Hipóteses Investigadas e Status

### H1: Câmera está realmente em pitch=0?
**Status: CONFIRMADO ✓**

Arquivo `/home/yan/Documentos/Projetos/cerise-turtlebot3-nav/world_with_camera.world`:
```xml
<model name="camera_overhead">
  <pose>0 0 3 0 0 0</pose>  <!-- roll=0, pitch=0, yaw=0 ✓ -->
  <static>true</static>
  <link name="camera_link">
    <sensor name="camera_overhead" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60° ✓ -->
```

- Câmera em z=3 metros ✓
- Sem rotação (pitch=0) ✓
- FOV horizontal = 1.047 rad (60°) ✓

**Porém**: Não há `<pose>` explícita no `<link name="camera_link">`, então herda do `<model>`.

**RISCO**: Se `gazebo_ros_camera` plugin tiver rotação implícita ou frame_name incorreto, câmera pode estar rotacionada.

---

### H2: Coordenadas do Gazebo vs ROS (odom vs mundo)
**Status: CRÍTICO - Possível fonte recorrente de erros**

#### Comportamento esperado:

**A. Em simulação Gazebo pura (sem ROS):**
- Mundo tem origem em (0, 0, 0)
- Robô spawn em X = 0, Y = 0.5 → posição ABSOLUTA = (0, 0.5)

**B. Em ROS2 + Gazebo:**
- `gazebo_ros_odometry` plugin publica `/robot*/odom`
- `pose.pose.position` = posição ABSOLUTA no mundo

#### O que acontece na prática:

```
gazebo_2robots.launch.py:
  Node spawn_entity.py -x 0.0 -y 0.5 -z 0.01 -robot_namespace robot1

Resultado esperado em /robot1/odom:
  pose.pose.position.x ≈ 0.0
  pose.pose.position.y ≈ 0.5
```

**PROBLEMA IDENTIFICADO**: O Gazebo pode estar publicando poses de duas formas:

| Cenário | pose.pose.position | Comportamento |
|---------|-------------------|--------------|
| A (esperado) | Coordenadas MUNDO | Correto com código atual |
| B (bugado) | Coordenadas LOCAIS (relativas ao spawn) | Com movimento y=0, sem movimento bbox fica fora |

#### Verificação necessária:

```bash
# Executar em terminal separado:
ros2 topic echo /robot1/odom --no-arr

# Esperado (Cenário A):
pose:
  pose:
    position:
      x: 0.0
      y: 0.5    # Posição absoluta no mundo
      z: 0.01

# Bugado (Cenário B):
pose:
  pose:
    position:
      x: 0.0
      y: 0.0    # Relativo ao spawn! Bug!
      z: 0.01
```

---

### H3: Parâmetros intrínsecos da câmera
**Status: PARCIALMENTE VERIFICADO ⚠**

Código atual em `dataset_collector.py`:
```python
fx = camera_info.k[0]
fy = camera_info.k[4]
cx = camera_info.k[2]
cy = camera_info.k[5]
```

Arquivo `world_with_camera.world`:
```xml
<horizontal_fov>1.047</horizontal_fov>
<image>
  <width>640</width>
  <height>480</height>
</image>
```

**Valores esperados** (calculados):
- FOV horizontal = 60°
- `fx = width / (2 * tan(FOV/2))`
- `fx = 640 / (2 * tan(30°)) = 640 / 1.155 ≈ 554.3 pixels`
- `fy ≈ 554.3 * (480/640) ≈ 415.7` (aspect ratio)

**VERIFICAÇÃO NECESSÁRIA**: Gazepo publica `camera_info` com estes valores?

```bash
ros2 topic echo /camera/camera_info --no-arr | grep -A 20 "k:"
```

**RISCO IDENTIFICADO**: Se Gazebo publica `k` array com valores default (zeros ou 1.0), projeção estará completamente errada.

---

### H4: Transformação de frames (TF)
**Status: INVESTIGAÇÃO NECESSÁRIA ⚠**

ROS2 usa Transform Frames (TF) para relacionar:
- `world` (origem do mundo Gazebo)
- `odom` (origem de odometria do robô)
- `base_footprint` / `base_link` (centro do robô)
- `camera_link` (posição da câmera)

#### Questões críticas:

1. **Qual frame é a "origem"?**
   - Em Gazebo puro: `world`
   - Em ROS2 com Nav2: pode ser `map` ou `odom` (DIFERENTE!)

2. **Como a pose é publicada?**
   ```python
   # Em dataset_collector.py:
   p = msg.pose.pose.position  # Qual frame exatamente?
   self.poses[name].x = p.x
   self.poses[name].y = p.y
   ```
   
   A pose está em qual frame? O código assume `world`, mas pode ser `odom` local do robô!

3. **Câmera também precisa estar em frame `world`!**
   - No .world: `<model name="camera_overhead">` implicitamente em frame `world`
   - Mas ROS2 publica frame_name = "camera_link"
   - TF deve conectar `world` → `camera_link`

#### Diagnóstico via TF:

```bash
# Ver árvore de transforms
ros2 run tf2_tools view_frames

# Verificar transformação específica
ros2 run tf2_ros tf2_echo world robot1/base_footprint

# Esperado:
Translation: [0.000, 0.500, 0.010]
Rotation in Quaternion: [0.000, 0.000, 0.000, 1.000]

# Bugado (frame local):
Translation: [0.000, 0.000, 0.010]
Rotation in Quaternion: [0.000, 0.000, 0.000, 1.000]
```

---

## Padrão-Ouro: Validação de Projeção em ROS2+Gazebo

### Método 1: Validação Visual (Recomendado para desenvolvimento)

**Ferramenta**: RViz2 + Marcadores de debug

```bash
# 1. Lançar simulação
ros2 launch cerise_nav gazebo_2robots.launch.py

# 2. Em outro terminal, lançar node de debug
ros2 run cerise_nav debug_camera_validation.py

# 3. Abrir RViz2
rviz2

# 4. Adicionar:
#    - Topic: /camera/image_raw (Imagem raw)
#    - Topic: /debug_markers (Esferas verdes nos pontos mundo calculados)
#    - Compare visualmente: as esferas devem estar sob os robôs nas imagens
```

**Script fornecido**: `/scripts/debug_camera_validation.py`
- Publica marcadores em `/debug_markers` (MarkerArray)
- Desenha bboxes calculados na imagem
- Valida se poses estão em coordenadas mundo ou locais
- Salva frames de debug em `/tmp/debug_frame_*.jpg`

---

### Método 2: Validação Matemática (Unit Tests)

Já implementado em `test_e2e_dataset_collector.py` e `debug_projection.py`.

**Limitação**: Testes passam, mas não validam dados reais de Gazebo.

```python
# Test pinhole projection com valores hardcoded
cx, cy = world_to_pixel_simple(0.0, 0.0)
assert abs(cx - 0.5) < 0.01  # Centro deve estar em (0.5, 0.5)

# Mas não testa se camera_info realmente tem estes valores!
```

---

### Método 3: Inspeção de Topics (Debugging rápido)

```bash
# Terminal 1: Simulação
ros2 launch cerise_nav gazebo_2robots.launch.py

# Terminal 2: Ver camera_info
ros2 topic echo /camera/camera_info --once

# Terminal 3: Ver poses
ros2 topic echo /robot1/odom --once
ros2 topic echo /robot2/odom --once

# Terminal 4: Validar frames TF
ros2 run tf2_tools view_frames
```

---

### Método 4: Validação Temporal (Ground Truth)

Se Gazebo publica poses via `gazebo_ros_pose_publisher` plugin:

```bash
# Ver modelo true pose (não via odom)
ros2 topic echo /gazebo/model_states

# Comparar com /robot1/odom — devem ser idênticos (sim, sem filtro)
```

---

## Ferramentas e Scripts de Debug Recomendados

### 1. **Script Principal: `debug_camera_validation.py`** (NOVO)

Localização: `/scripts/debug_camera_validation.py`

Funcionalidades:
- ✓ Inspeciona `camera_info` publicada
- ✓ Lê poses em tempo real
- ✓ Publica marcadores RViz
- ✓ Detecta frames de referência incorretos
- ✓ Valida orientação de câmera
- ✓ Salva frames com bboxes desenhados

Uso:
```bash
ros2 run cerise_nav debug_camera_validation.py
# Abrir RViz2 em outro terminal para visualizar
```

### 2. **Inspect Camera Info**

Cria quick script para inspecionar `camera_info`:

```bash
#!/bin/bash
echo "=== Camera Info ==="
ros2 topic echo /camera/camera_info --once | head -30
```

### 3. **View TF Tree**

```bash
ros2 run tf2_tools view_frames
# Gera: /tmp/frames.pdf com árvore de transformações
```

### 4. **Compare odom vs gazebo/model_states** (se disponível)

```bash
#!/bin/bash
# Ver modelo real no Gazebo
ros2 topic echo /gazebo/model_states --once | grep -A 3 "robot1"
# Ver pose publicada em ROS
ros2 topic echo /robot1/odom --once | grep -A 3 "position"
# Devem ser idênticos (com Z offset apenas)
```

---

## Checklist de Diagnóstico Prático

Execute em ordem se bboxes ainda estiverem desalinhadas:

- [ ] **Passo 1**: Confirmar que câmera está em pitch=0
  ```bash
  grep -A 2 "camera_overhead" world_with_camera.world | grep -E "<pose>|pitch"
  # Esperado: <pose>0 0 3 0 0 0</pose>
  ```

- [ ] **Passo 2**: Inspecionar camera_info publicada
  ```bash
  ros2 launch cerise_nav gazebo_2robots.launch.py
  # Em outro terminal:
  ros2 topic echo /camera/camera_info --once
  # Verificar: fx, fy, width, height são os esperados?
  ```

- [ ] **Passo 3**: Validar poses em coordenadas mundo
  ```bash
  ros2 topic echo /robot1/odom --once | grep -A 3 "position"
  # Esperado sem movimento: y ≈ 0.5
  # Bugado (frame local): y ≈ 0.0
  ```

- [ ] **Passo 4**: Usar RViz para inspeção visual
  ```bash
  # Terminal novo:
  rviz2
  # Adicionar:
  #   /camera/image_raw
  #   /debug_markers
  # Verificar se esferas verdes estão sob os robôs
  ```

- [ ] **Passo 5**: Verificar árvore de transforms
  ```bash
  ros2 run tf2_tools view_frames
  # Abrir /tmp/frames.pdf
  # Verificar se world → camera_link → robot/base_footprint estão conectados
  ```

---

## Como Lidar com Frames de Referência

### Mapeamento Correto: map vs odom vs base_footprint

```
                        ┌─────────────────────┐
                        │  World (Gazebo)     │
                        │  Origem: (0,0,0)    │
                        └──────────┬──────────┘
                                   │
                            (TF transform)
                                   │
                        ┌──────────▼──────────┐
                        │  /world (ROS frame) │
                        │  ou /map            │
                        └──────────┬──────────┘
                                   │
                            (TF transform)
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
  ┌─────▼─────┐            ┌─────▼─────┐            ┌─────▼─────┐
  │  /robot1/ │            │  /robot2/ │            │ /camera_  │
  │  odom     │            │  odom     │            │ link      │
  └─────┬─────┘            └─────┬─────┘            └───────────┘
        │                        │
  ┌─────▼─────┐            ┌─────▼─────┐
  │ /robot1/  │            │ /robot2/  │
  │base_link  │            │base_link  │
  └───────────┘            └───────────┘
```

### O Erro do Commit 560558b

```
Cenário ERRADO (que foi tentado):
  /robot1/odom publica: position = (0, 0)  [relativo ao spawn]
  Código tentava: position + ROBOT_SPAWN = (0, 0) + (0, 0.5) = (0, 0.5) ✓

Cenário CORRETO (atual):
  /robot1/odom publica: position = (0, 0.5)  [absoluto no mundo]
  Código NÃO soma offset: position = (0, 0.5) ✓

Erro ocorre se:
  Gazebo publica em frame LOCAL por algum motivo
  → Código usa posição local como se fosse mundo
  → Bbox desenhado no lugar errado
```

### Solução Robusta: Usar TF para Validar

```python
# Em vez de ler pose direto de odom:
p = msg.pose.pose.position  # Qual frame?

# Melhor prática: usar TF para ter certeza
import tf2_py
from tf2_geometry_msgs import do_transform_point

tf_buffer = tf2_py.Buffer()
transformer = tf2_ros.TransformListener(tf_buffer, self)

try:
    # Transformar sempre para 'world' frame
    transform = tf_buffer.lookup_transform(
        'world', 'robot1/base_footprint', rclpy.time.Time()
    )
    point = PointStamped()
    point.point.x = 0
    point.point.y = 0
    point.point.z = 0
    point.header.frame_id = 'robot1/base_footprint'
    
    point_world = do_transform_point(point, transform)
    pose_x = point_world.point.x
    pose_y = point_world.point.y
except tf2_ros.ExtrapolationException:
    self.get_logger().warn('TF not available')
```

---

## Resumo das Recomendações

| Item | Situação | Ação |
|------|----------|------|
| **Câmera pitch=0** | ✓ Confirmado | Nenhuma |
| **camera_info publicada** | ⚠ Não validada | → Executar `/scripts/debug_camera_validation.py` |
| **Poses em frame correto** | ⚠ Não validada | → Verificar `/robot*/odom` com `ros2 topic echo` |
| **TF transforms** | ⚠ Não documentado | → Executar `ros2 run tf2_tools view_frames` |
| **Unit tests** | ✓ Implementados | Passam, mas não validam dados reais |
| **Validação visual** | ⚠ Manual com RViz | → Script de debug criado para automatizar |
| **Documentação** | ⚠ Incompleta | → Este relatório + CAMERA_PROJECTION_DEBUG.md |

---

## Próximos Passos (Recomendados)

### Curto prazo (hoje):
1. Executar `debug_camera_validation.py` com simulação rodando
2. Abrir RViz2 e verificar alinhamento visual de marcadores
3. Inspecionar `/camera/camera_info` com `ros2 topic echo`
4. Verificar poses de `/robot1/odom` e `/robot2/odom`

### Médio prazo (antes de coletar dataset):
1. Adicionar validação automática em `dataset_collector.py` (verificar se poses fazem sentido)
2. Salvar debug frames com bboxes calculados
3. Criar teste de regressão que valida alinhamento com Gazebo rodando
4. Documentar frame de referência esperado em docstring

### Longo prazo:
1. Considerar usar TF para validação robusta (contra futuros bugs)
2. Integrar visualização RViz permanentemente no launch file
3. Criar métricas de qualidade de dataset (% bboxes bem-alinhadas)

---

## Referências e Recursos

### ROS2 + Gazebo + Transforms:
- [ROS2 Gazebo Classic Plugin Docs](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)
- [Understanding TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Gazebo Camera Sensor Plugin](https://classic.gazebosim.org/tutorials?tut=ros_camera_use)

### Computer Vision (Projeção Pinhole):
- [Hartley & Zisserman, Multiple View Geometry, Cap. 6.2 (Camera Intrinsics)]
- [OpenCV Camera Calibration](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)

### Debugging em ROS2:
- [RViz2 Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html)
- [ros2 topic command reference](https://docs.ros.org/en/humble/How-To-Guides/Topics-CLI.html)

---

**Documento criado em**: 2026-05-01  
**Status**: Pesquisa completada, aguardando validação experimental  
**Próxima etapa**: Executar scripts de debug com simulação rodando
