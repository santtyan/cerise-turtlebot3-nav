# Frames de Referência em ROS2+Gazebo: Documentação Técnica

**Como validar e usar frames de referência corretamente para bbox em Gazebo**

---

## Conceitos Fundamentais

### O que é um Frame (TF) em ROS?

Um "frame" é um sistema de coordenadas. ROS usa TF (Transform Frames) para relacionar diferentes sistemas de coordenadas.

**Exemplo**:
```
Frame WORLD (origem global):
  Origem: (0, 0, 0)
  Usado por: Câmera fixa, mapa, simulação Gazebo

Frame ODOM (odometria do robô):
  Origem: Ponto de spawn do robô
  Publicado por: Plug-in gazebo_ros_odometry
  Problema: Pode ser RELATIVO ou ABSOLUTO!

Frame BASE_FOOTPRINT (centro do robô):
  Origem: Base do robô (projeção no chão)
  Publicado por: Modelo do robô SDF
  Usado por: Controle de movimento, sensores
```

---

## O Problema: Dois Modos de Publicar Odometria

### Modo A: Odometria ABSOLUTA (esperado em Gazebo clássico)

```
Gazebo:
  world_x = 0.0, world_y = 0.5 (spawn robot1)
  
ROS2 /robot1/odom:
  pose.pose.position = (0.0, 0.5)  ← MUNDO, não relativa ao spawn
  header.frame_id = "odom"
  child_frame_id = "base_footprint"
  
TF Result:
  world → robot1/odom: identity (no transform needed)
  world → robot1/base_footprint: (0.0, 0.5, 0.0)
```

**Código correspondente**:
```python
# dataset_collector.py (CORRETO para Modo A)
p = msg.pose.pose.position
self.poses[name].x = p.x      # Direto, já está em mundo
self.poses[name].y = p.y
```

---

### Modo B: Odometria RELATIVA (alguns plugins/configurações)

```
Gazebo:
  world_x = 0.0, world_y = 0.5 (spawn robot1)
  
ROS2 /robot1/odom:
  pose.pose.position = (0.0, 0.0)  ← RELATIVA AO SPAWN!
  header.frame_id = "robot1/odom"  ← Namespace local
  child_frame_id = "robot1/base_footprint"
  
TF Result:
  world → robot1/odom: (0.0, 0.5, 0.0)  ← Transform necessário!
  robot1/odom → robot1/base_footprint: (0.0, 0.0, 0.0)
  
Equivalente mundo: (0.0, 0.5, 0.0) + (0.0, 0.0, 0.0) = (0.0, 0.5, 0.0) ✓
```

**Código correspondente**:
```python
# dataset_collector.py (CORRETO para Modo B)
p = msg.pose.pose.position
spawn_x, spawn_y = ROBOT_SPAWN[name]
self.poses[name].x = p.x + spawn_x      # Somar offset necessário
self.poses[name].y = p.y + spawn_y
```

---

## Qual é a Realidade em Gazebo?

### Investigação: Como gazebo_ros_odometry Publica

**Arquivo**: `/home/yan/Documentos/Projetos/cerise-turtlebot3-nav/waffle_nodepth.model`

O modelo **não especifica** o plugin de odometria (foi removido conforme comentário).

**Isso significa**: Odometria é publicada pela **startup padrão de Gazebo**, que depende de:

1. **Qual plugin ROS está carregando?**
   - `gazebo_ros_odometry` (padrão)
   - `gazebo_ros_p3d` (pose de sensor virtual)
   - Nenhum (não publica)

2. **Como é a configuração?**
   - Pode estar em `.model` do robô
   - Ou em `.world` global
   - Ou não existe (sem odometria)

**Estado atual**: Não há plugin explícito no `waffle_nodepth.model`!

---

### Verificação Experimental

```bash
# Terminal 1: Lançar simulação
ros2 launch cerise_nav gazebo_2robots.launch.py

# Terminal 2: Verificar se /robot1/odom existe
timeout 10 ros2 topic list | grep odom
# Saída esperada:
#   /robot1/odom
#   /robot2/odom

# Terminal 3: Inspecionar conteúdo
ros2 topic echo /robot1/odom --once
```

**Se `/robot1/odom` NÃO existe**: Nenhum plugin está publicando → Problema!

**Se `/robot1/odom` exists**:
```bash
# Ver coordenadas com robô SEM movimento
ros2 topic echo /robot1/odom --once | grep -A 5 "position:"

# Resultado A (esperado Modo A):
position:
  x: 0.0
  y: 0.5   ← Coordenada MUNDO
  z: 0.01

# Resultado B (Modo B):
position:
  x: 0.0
  y: 0.0   ← Coordenada RELATIVA (spawn offset missing)
  z: 0.01
```

---

## Solução Robusta: Usar TF para Transformação

Em vez de assumir qual modo está sendo usado, **sempre transformar para MUNDO**:

```python
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

class DatasetCollector(Node):
    def __init__(self):
        # ... resto do init ...
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def _odom_cb(self, msg: Odometry, name: str):
        """Callback que transforma pose para mundo automaticamente."""
        try:
            # Aguardar TF estar disponível (com timeout)
            transform = self.tf_buffer.lookup_transform(
                'world',
                f'{name}/base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Pose agora está garantidamente em frame 'world'
            self.poses[name].x = transform.transform.translation.x
            self.poses[name].y = transform.transform.translation.y
            self.poses[name].updated = True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            # TF não disponível, fallback para odom direto
            p = msg.pose.pose.position
            self.poses[name].x = p.x
            self.poses[name].y = p.y
            self.poses[name].updated = True
```

**Vantagens**:
- ✓ Funciona em qualquer modo (A ou B)
- ✓ Robusta contra mudanças de configuração
- ✓ Explícita sobre qual frame está sendo usado
- ✓ Detecta problemas de TF (falha gracefully)

---

## Mappings de Frames: Diagramas Visuais

### Cenário 1: Gazebo + ROS (Correto)

```
┌────────────────────────────────────────────┐
│         WORLD (Gazebo global origin)       │
│              (0, 0, 0)                     │
└────────┬─────────────────┬──────────────┬──┘
         │                 │              │
      TF: (0, 0, 3)     TF: (0, 0.5)   TF: (0, -0.5)
    roll=0              roll=0            roll=0
    pitch=0             pitch=0           pitch=0
    yaw=0               yaw=0             yaw=0
         │                 │              │
    ┌────▼────┐        ┌────▼────┐   ┌────▼────┐
    │CAMERA   │        │ROBOT1   │   │ROBOT2   │
    │LINK     │        │/ODOM    │   │/ODOM    │
    │@z=3m    │        │@(0,0.5) │   │@(0,-0.5)│
    └────┬────┘        └────┬────┘   └────┬────┘
         │                  │              │
         │              TF: (0,0)      TF: (0,0)
         │                  │              │
         │            ┌─────▼──────┐ ┌─────▼──────┐
         │            │BASE_FOOT   │ │BASE_FOOT   │
         │            │PRINT 1     │ │PRINT 2     │
         └────────────┴────────────┴─┴────────────┘
                  [All frames connected]
```

### Cenário 2: Gazebo com plugin odometria RELATIVA (Bugado)

```
┌────────────────────────────────────────────┐
│         WORLD (Gazebo global origin)       │
│              (0, 0, 0)                     │
└────┬────────────────┬──────────────┬───────┘
     │                │              │
  TF: (0,0,3)      TF: (0,0.5)    TF:(0,-0.5)
     │                │              │
 ┌───▼──┐         ┌────▼──────┐  ┌────▼──────┐
 │CAMERA│         │ROBOT1/ODOM│  │ROBOT2/ODOM│
 │LINK  │         │(LOCAL)    │  │(LOCAL)    │
 └──────┘         └────┬──────┘  └────┬──────┘
                  TF:(0,0)        TF:(0,0)
                       │              │
                  ┌────▼──────┐  ┌────▼──────┐
                  │BASE_FOOT  │  │BASE_FOOT  │
                  │PRINT 1    │  │PRINT 2    │
                  │@(0,0)     │  │@(0,0)     │
                  └───────────┘  └───────────┘

  Result:
  - odom publica (0, 0) relativo ao spawn
  - base_footprint está em (0, 0) relativo a odom
  - Posição em mundo: (0, 0.5) + (0, 0) = (0, 0.5) ✓ Correto em TF
  - Mas sem usar TF, pareça estar em (0, 0) ✗ ERRADO!
```

---

## Verificação de Frame_ID nos Topics

### O que procurar:

```bash
ros2 topic echo /robot1/odom --once
```

**Esperado (Modo A - Simples)**:
```
header:
  frame_id: odom      ← Frame da odometria
...
child_frame_id: base_footprint  ← Frame do robô
pose:
  pose:
    position:
      x: 0.0
      y: 0.5     ← JÁ em mundo (simples)
      z: 0.01
```

**Bugado (Modo B - Complexo)**:
```
header:
  frame_id: robot1/odom  ← Namespace específico do robô!
...
child_frame_id: robot1/base_footprint
pose:
  pose:
    position:
      x: 0.0
      y: 0.0    ← Relativo ao spawn
      z: 0.01

# TF conecta:
#   world → robot1/odom: (0, 0.5)
#   robot1/odom → robot1/base_footprint: (0, 0)
# Total: (0, 0.5) ✓ Correto se usar TF
```

---

## Histórico de Erros: Por que 560558b falhou

**Commit**: 560558b `fix: converte odom local para coordenadas do mundo na projeção`

**O que fez**:
```python
# Tentou assumir Modo B
spawn_x, spawn_y = ROBOT_SPAWN[name]
self.poses[name].x = p.x + spawn_x
self.poses[name].y = p.y + spawn_y
```

**Por que falhou**:
- Sistema estava em Modo A (odom já em mundo)
- Somou offset desnecessário
- Bbox ficou no lugar errado (2x offset)

**Lição aprendida**: Não assumir o modo, validar ou usar TF!

---

## Checklist: Configurar Corretamente

### Pré-requisito 1: Plugin de Odometria

**Verificar** se existe em `waffle_nodepth.model`:
```xml
<joint name="child_frame_joint" type="fixed">
  <child>base_footprint</child>
  <parent>base_link</parent>
  <origin xyz="0 0 -0.055" rpy="0 0 0"/>
</joint>

<!-- Plugin: Publicar transforms -->
<plugin name='gazebo_ros_odometry' filename='libgazebo_ros_odometry.so'>
  <robotNamespace>/robot1</robotNamespace>  <!-- Correto: sem namespace duplo -->
  <frame_name>world</frame_name>
  <child_frame_name>base_footprint</child_frame_name>
</plugin>
```

### Pré-requisito 2: Launch file

**Verificar** `gazebo_2robots.launch.py`:
```python
Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
        "-entity", "robot1",
        "-file", str(pkg_dir / "waffle_nodepth.model"),
        "-robot_namespace", "robot1",  # Namespace correto
        "-x", "0.0",
        "-y", "0.5",  # Spawn position
        "-z", "0.01",
    ],
)
```

### Pré-requisito 3: Validar TF

```bash
ros2 run tf2_tools view_frames
# Procurar por:
# - world → robot1/base_footprint (direto ou indireto)
# - Não deve haver quebra na cadeia
```

---

## Recomendação Final: Usar TF (Safest)

**Se não há certeza sobre qual modo está sendo usado**, sempre use TF:

```python
def get_robot_pose_world(self, name: str) -> Tuple[float, float]:
    """Obtém pose garantidamente em frame 'world'."""
    try:
        # Procurar frame do robô (pode ser diferentes nomes)
        frame_candidates = [
            f'{name}/base_footprint',
            f'{name}/base_link',
            f'robot{name[-1]}/base_footprint',  # Para "robot1" → "robot1/base_footprint"
        ]
        
        for frame in frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                return float(transform.transform.translation.x), float(transform.transform.translation.y)
            except tf2_ros.LookupException:
                continue
        
        # Nenhum frame encontrado, fallback seguro
        self.get_logger().warn(f'{name}: Nenhum frame TF encontrado')
        return None, None
        
    except (tf2_ros.TransformException, tf2_ros.ConnectivityException) as e:
        self.get_logger().warn(f'TF error: {e}')
        return None, None
```

**Vantagens**:
- ✓ Robusto contra qualquer configuração
- ✓ Explícito sobre frame
- ✓ Automático (sem hardcoding de offset)
- ✓ Documentado (código mostra intenção)

---

## Resumo

| Aspecto | Detalhes |
|---------|----------|
| **Frame global** | `world` (origem Gazebo) |
| **Frame da câmera** | `camera_link` em `world` @ (0, 0, 3) |
| **Frame do robô** | `robot1/base_footprint` em `world` @ (0, 0.5) |
| **Odom publicado** | Pode ser Modo A (mundo) ou B (relativo) |
| **Solução correta** | Usar TF para garantir transformação para mundo |
| **Validação** | `ros2 run tf2_tools view_frames` + RViz |
| **Código seguro** | Use TF ao invés de assumir modo |

---

**Documento criado**: 2026-05-01  
**Próximas atualizações**: Após validação experimental com Gazebo rodando
