# ✅ Checklist de Validação - Interface Gráfica

Antes de começar, execute os testes abaixo:

## Teste 1: Build do Projeto ✅
```bash
cd ~/cerise-turtlebot3-nav
colcon build --symlink-install
source install/setup.bash
```
**Esperado**: Build completa sem erros. Output final: `Summary: 1 package finished`

---

## Teste 2: Gazebo + GUI ✅
```bash
./run_gui.sh
```

**Checklist visual**:
- [ ] Janela Gazebo aparece em ~3 segundos
- [ ] Arena turtlebot3_world visível
- [ ] 2 robots (azuis) aparecem em posições diferentes (y=0.5 e y=-0.5)
- [ ] Câmera overhead visível (modelo cinza acima da arena)
- [ ] Logs mostram:
  - `✓ Gazebo rodando em PID XXXXX`
  - `Successfully spawned entity [robot1]`
  - `Successfully spawned entity [robot2]`

**Se falhar**:
```bash
pkill -9 gzserver  # Limpar processos órfãos
./run_gui.sh       # Tentar novamente
```

---

## Teste 3: ROS2 Nodes (Terminal 2)
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
```

**Esperado**: Lista de nodes com namespace robot1 e robot2
```
/robot1/controller_manager
/robot1/gazebo_ros_control
/robot1/turtlebot3_diff_drive
/robot1/turtlebot3_joint_state
/robot2/...
```

---

## Teste 4: Tópicos ROS2 (Terminal 2)
```bash
ros2 topic list | grep -E "robot|camera|odom"
```

**Esperado**: Topics incluem:
- `/robot1/odom` → Odometria do robot1
- `/robot2/odom` → Odometria do robot2  
- `/camera/image_raw` → Câmera overhead
- `/camera/camera_info` → Informações da câmera

---

## Teste 5: Monitorar Odometria (Terminal 2)
```bash
ros2 topic echo /robot1/odom --once
```

**Esperado**: JSON com pose (x, y, theta) do robot1

---

## Teste 6: Verificar Câmera (Terminal 2)
```bash
ros2 topic echo /camera/image_raw --once
```

**Esperado**: Frame de imagem 640x480 em formato RGB8

---

## Teste 7: Visualizar em RViz (Opcional - Terminal 3)
```bash
ros2 run rviz2 rviz2
```

**Configuração no RViz**:
1. **Global Options**:
   - Fixed Frame: `map`
   
2. **Adicionar Displays**:
   - TF (mostra frames dos robots)
   - Image → /camera/image_raw (mostra câmera)
   - LaserScan → /robot1/scan

---

## Status Atual ✅

| Componente | Status | Notas |
|-----------|--------|-------|
| Gazebo Server | ✅ OK | Rodando com GUI |
| Robot Spawn | ✅ OK | Ambos spawned com sucesso |
| Odometria | ✅ OK | Publicando dados |
| Câmera | ✅ OK | Sensor ativo, topics publicando |
| ROS2 Nodes | ⚠️ INVESTIGANDO | Possível issue de middleware ROS2 DDS |
| Nav2 Integration | ⏳ TODO | Próxima etapa |

---

## ⚡ Quick Commands

```bash
# Ver todos os processes do Gazebo
ps aux | grep gazebo

# Monitorar CPU/memoria
htop | grep gazebo

# Limpar tudo
pkill -9 gzserver ros2

# Rodar com logging detalhado
ROS_LOG_DIR=/tmp/ros_logs ./run_gui.sh

# Mudar ROS_DOMAIN_ID (isolamento entre múltiplas instâncias)
export ROS_DOMAIN_ID=1
./run_gui.sh
```

---

## 📋 Conclusão

**Você está pronto para:**
1. ✅ Visualizar simulação em tempo real
2. ✅ Monitorar dados dos robots (odom, scan)
3. ✅ Coletar frames de câmera
4. ✅ Integrar Nav2 para navegação

**Próxima fase**: 
- Implementar navegação autônoma com Nav2
- Criar dataset collector para YOLO
- Treinar modelo de detecção de robôs

---

*Última validação: 2026-04-14*
