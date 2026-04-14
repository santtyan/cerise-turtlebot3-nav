# 🚀 CERISE TurtleBot3 - Quick Start GUI

## ✅ Status
- ✓ Gazebo rodando com interface gráfica
- ✓ 2 TurtleBot3 Waffle spawned automaticamente
- ✓ Câmera overhead configurada
- ✓ ROS2 nodes publicando dados

## 🎮 Como Usar

### Terminal 1: Iniciar Gazebo + Robots
```bash
cd ~/cerise-turtlebot3-nav
./run_gui.sh
```

**Resultado esperado:**
- Janela Gazebo abre com 2 robots na arena
- Mensagens de log mostram spawning bem-sucedido
- Aguarde ~5s até tudo estar pronto

### Terminal 2: Monitorar Tópicos (opcional)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list
ros2 topic echo /robot1/odom
```

### Terminal 3: Visualizar Câmera
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic echo /camera/image_raw --once
```

## 📊 Arquitetura Atual

```
[Gazebo Server]
    ├─ Robot1 (TurtleBot3 Waffle)
    │   ├─ /robot1/odom
    │   ├─ /robot1/scan
    │   └─ /robot1/tf
    │
    ├─ Robot2 (TurtleBot3 Waffle)
    │   ├─ /robot2/odom
    │   ├─ /robot2/scan
    │   └─ /robot2/tf
    │
    └─ Camera Overhead
        ├─ /camera/image_raw (640x480 RGB)
        └─ /camera/camera_info
```

## ⚙️ Configurações Importantes

### Variáveis de Ambiente
- `TURTLEBOT3_MODEL=waffle` → Modelo do robô
- `ROS_DOMAIN_ID=0` → DDS domain (padrão)
- `DISPLAY=:0` → Interface gráfica
- `GAZEBO_MODEL_DATABASE_URI=""` → Offline mode (evita downloads)

### Modelos Utilizados
- **Mundo**: `world_with_camera.model` (com câmera overhead)
- **Robot**: `waffle_nodepth.model` (sem câmera RGB)
- **Física**: ODE, 1000Hz update rate

## 🛠️ Troubleshooting

### Gazebo não abre janela
```bash
# Verificar DISPLAY
echo $DISPLAY

# Forçar X11
export DISPLAY=:0
./run_gui.sh
```

### Tópicos ROS2 não aparecem
```bash
# Verificar ROS_DOMAIN_ID sincronizado
echo $ROS_DOMAIN_ID  # Deve ser 0

# Reiniciar Gazebo
pkill -9 gzserver
./run_gui.sh
```

### Robots não spawnam
- Aguarde 3+ segundos após Gazebo iniciar
- Verifique logs: `tail -50 /tmp/run.log`
- Modelos devem existir em: `~/cerise-turtlebot3-nav/*.model`

## 📝 Próximos Passos

1. **Nav2 Integration** → Adicionar navegação autônoma
2. **Dataset Collection** → `ros2 run cerise_nav dataset_collector`
3. **YOLO Training** → Treinar modelo com dataset coletado
4. **Performance Tuning** → Otimizar render e physics

## 📚 Arquivos Principais

| Arquivo | Função |
|---------|--------|
| `run_gui.sh` | Script de inicialização |
| `world_with_camera.model` | Mundo Gazebo com câmera |
| `waffle_nodepth.model` | Modelo do robot (sem profundidade) |
| `launch/gazebo_2robots.launch.py` | Launch ROS2 (alternativo) |

---

**Última atualização**: 2026-04-14  
**Ambiente**: Linux native (Ubuntu 22.04) + ROS2 Humble + Gazebo 11.10.2
