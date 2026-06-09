# Quick Debug: Validar Projeção de Câmera em 10 minutos

**Se bboxes estão desalinhadas, siga este roteiro passo-a-passo.**

---

## ⚡ Execução Rápida

### Terminal 1: Lançar simulação
```bash
ros2 launch cerise_nav gazebo_2robots.launch.py
# Aguarde "Camera initialized" nas logs
```

### Terminal 2: Lançar validação
```bash
cd /home/yan/Documentos/Projetos/cerise-turtlebot3-nav
ros2 run cerise_nav debug_camera_validation.py
```

### Terminal 3: Abrir RViz
```bash
rviz2
```

Em RViz:
1. **Panels → Add → ROS2 Image** → Topic `/camera/image_raw`
2. **Panels → Add → Marker Array** → Topic `/debug_markers`
3. **Verificar**: As esferas VERDES devem estar sobre os robôs na imagem
   - Se SIM: Projeção está OK ✓
   - Se NÃO: Ir para "Diagnóstico Detalhado"

---

## 🔍 Diagnóstico Detalhado (5-10 min)

Se RViz mostrou bboxes desalinhadas, execute isto:

### Check 1: Camera Info
```bash
# Terminal novo
ros2 topic echo /camera/camera_info --once
```

**Esperado**:
```
k:
- 554.254256515508  # fx
- 0.0
- 320.0              # cx
- 0.0
- 554.254256515508  # fy
- 240.0              # cy
- 0.0
- 0.0
- 1.0
```

**Verifier**: Valores fx/fy devem estar próximos de 554 (60° FOV em 640px)

---

### Check 2: Poses em odom
```bash
# Terminal novo, com simulação rodando
ros2 topic echo /robot1/odom --once
```

**Esperado** (SEM movimento):
```
pose:
  pose:
    position:
      x: 0.0
      y: 0.5       # ← CRÍTICO: Deve ser ~0.5, não 0.0!
      z: 0.01
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

**Problema SE**: `y ≈ 0.0` (deveria ser 0.5)
- → Gazebo está publicando em frame LOCAL
- → Reativar offset em `dataset_collector.py` (ROBOT_SPAWN)
- → OU problema em `gazebo_ros_odometry` plugin

---

### Check 3: Verificar TF Frames
```bash
# Terminal novo
ros2 run tf2_tools view_frames
# Aguarde ~10 segundos
# Arquivo gerado: /tmp/frames.pdf
```

**Abrir** `/tmp/frames.pdf`:
- [ ] `world` → `camera_link` conectados?
- [ ] `world` → `robot1/base_footprint` conectados?
- [ ] `world` → `robot2/base_footprint` conectados?

Se algum está faltando → TF não está configurado corretamente.

---

### Check 4: Frames de referência no mundo vs local
```bash
# Ver qual frame odom está usando
ros2 topic echo /robot1/odom --once | grep "frame_id"
```

**Esperado**:
```
header:
  frame_id: odom
...
child_frame_id: base_footprint
```

**CRÍTICO**: Se `frame_id: robot1/odom` (local namespace), pode ser o problema!

---

## 🐛 Problemas Comuns e Soluções

### Problema A: Esferas em RViz estão todas no mesmo lugar (origem)
**Causa**: Poses estão todas em (0, 0)  
**Solução**: Check 2 acima mostrou o problema

### Problema B: Esferas desapareceram depois de alguns frames
**Causa**: Problema de sincronização entre câmera e odom  
**Solução**: 
```python
# Em dataset_collector.py, linha 103-107:
if not all(p.updated for p in self.poses.values()):
    return  # Aguarda todas as poses
```
Aumentar timeout ou verificar se `/robot*/odom` está publicando (não morreu)

### Problema C: Esferas estão rotacionadas 90° (não sob robôs)
**Causa**: Câmera pode estar rotacionada  
**Solução**: 
1. Verificar `<pose>` em world_with_camera.world
2. Verificar se `gazebo_ros_camera` tem `<orientation_only_cam>false</orientation_only_cam>` (compatibilidade)

### Problema D: Esferas fora do campo de visão
**Causa**: Campo de visão (FOV) menor que esperado  
**Solução**: Verificar `horizontal_fov` em .world (deve ser 1.047 rad = 60°)

---

## 📊 Validação Matemática Rápida

Se Check 1 e Check 2 passaram, fazer este teste:

```python
# Python3 interativo
import numpy as np

# Parâmetros da câmera (do Check 1)
fx, fy = 554.25, 554.25
cx, cy = 320.0, 240.0
img_w, img_h = 640, 480
cam_height = 3.0

# Posição esperada (do Check 2)
world_x, world_y = 0.0, 0.5

# Projetar para pixel
cam_x = world_x
cam_y = -world_y  # Invertido
cam_z = cam_height

u = fx * (cam_x / cam_z) + cx
v = fy * (cam_y / cam_z) + cy

print(f"Posição mundo: ({world_x}, {world_y})")
print(f"Pixel esperado: ({u:.1f}, {v:.1f})")
print(f"Normalizado YOLO: ({u/img_w:.3f}, {v/img_h:.3f})")

# Robot1 em (0, 0.5) deve estar em pixel (320, ~147-148)
# Se estiver diferente → erro em Check 1 ou 2
```

---

## 🔧 Quick Fixes Se Detectado Problema

### Se Check 2 falhou (y ≈ 0.0 em vez de 0.5):

**Arquivo**: `src/cerise_nav/cerise_nav/dataset_collector.py`

**Linha 84-88 ATUAL** (pode estar errado):
```python
def _odom_cb(self, msg: Odometry, name: str):
    p = msg.pose.pose.position
    # Odômetro já inicia na pose de spawn (mundo), não na origem
    # Não somar offset novamente
    self.poses[name].x = p.x
    self.poses[name].y = p.y
```

**Se poses estão com y=0**: Reativar offset
```python
def _odom_cb(self, msg: Odometry, name: str):
    p = msg.pose.pose.position
    spawn_x, spawn_y = ROBOT_SPAWN[name]
    self.poses[name].x = p.x + spawn_x
    self.poses[name].y = p.y + spawn_y
```

**Se poses estão com y=0.5**: Deixar como está ✓

---

### Se Check 3 falhou (TF não conectado):

Verificar `gazebo_2robots.launch.py`:
```python
# Deve incluir:
<arg name="publish_tf_static_transform">true</arg>
<arg name="publish_tf">true</arg>
```

---

## ✅ Checklist Final

- [ ] RViz mostra esferas verdes sobre robôs
- [ ] `camera_info.k[0]` ≈ 554
- [ ] `/robot1/odom` y ≈ 0.5 (sem movimento)
- [ ] `/robot2/odom` y ≈ -0.5 (sem movimento)
- [ ] `/tmp/frames.pdf` mostra TF conectado
- [ ] Projeção matemática bate com pixels em RViz

**Se todos OK**: Bbox está correto, problema resolvido! ✓

**Se algum falhou**: Ver seção "Problemas Comuns" acima

---

## 📞 Se Nada Funcionar

Colete logs para análise posterior:

```bash
# Terminal com simulação rodando
ros2 launch cerise_nav gazebo_2robots.launch.py > /tmp/gazebo.log 2>&1 &
sleep 10

# Terminal com validação
ros2 run cerise_nav debug_camera_validation.py > /tmp/debug.log 2>&1 &
sleep 10

# Colete dados
ros2 topic echo /camera/camera_info --once > /tmp/camera_info.txt
ros2 topic echo /robot1/odom --once > /tmp/robot1_odom.txt
ros2 topic echo /robot2/odom --once > /tmp/robot2_odom.txt
ros2 run tf2_tools view_frames

# Archive
tar czf /tmp/debug_logs.tar.gz /tmp/*.log /tmp/*.txt /tmp/frames.pdf

echo "Logs salvos em /tmp/debug_logs.tar.gz"
```

---

**⏱️ Tempo estimado: 10 minutos**  
**🎯 Objetivo: Validar ou identificar problema em bbox**
