# Documentação: Projeção de Câmera em Gazebo+ROS2

**Índice de recursos para debug e validação de bounding boxes**

---

## Problema Identificado

Bounding boxes calculadas não correspondem visualmente aos robôs nas imagens da câmera overhead em Gazebo.

Este fenômeno ocorreu **2 vezes** no histórico (commit 560558b + revert), indicando problema sistemático de ambiguidade em frame de referência.

---

## 📚 Documentação (Leia nesta ordem)

### 1. **RESEARCH_SUMMARY.md** ← COMECE AQUI
   - Sumário executivo (5 min de leitura)
   - Descobertas principais
   - Recomendações imediatas
   - **Próximo passo**: Escolher entre Quick Debug ou diagnóstico completo

### 2. **CAMERA_PROJECTION_QUICK_DEBUG.md**
   - Guia prático para validar em 10 minutos
   - Checklist passo-a-passo com comandos prontos
   - Problemas comuns e soluções rápidas
   - **Use se**: Bboxes estão errados agora e quer solução rápida

### 3. **CAMERA_PROJECTION_DIAGNOSIS.md**
   - Análise técnica completa (3.5 KB)
   - Todas as 5 hipóteses investigadas em detalhe
   - Explicação de cada confirmação/falha
   - Padrão-ouro de validação em ROS2+Gazebo
   - **Use se**: Quer entender raiz do problema profundamente

### 4. **TF_FRAMES_REFERENCE.md**
   - Documentação técnica sobre Transform Frames
   - Diagramas dos 2 modos de odometria
   - Como implementar solução robusta com TF
   - Por que commit 560558b falhou
   - **Use se**: Quer implementar solução permanente baseada em TF

---

## 🔧 Ferramentas e Scripts

### 1. **scripts/debug_camera_validation.py** (NOVO)
   - Script ROS2 de validação visual
   - Publica marcadores RViz para comparação
   - Desenha bboxes calculados nas imagens
   - Detecta frames de referência incorretos

**Como usar**:
```bash
# Terminal 1
ros2 launch cerise_nav gazebo_2robots.launch.py

# Terminal 2
ros2 run cerise_nav debug_camera_validation.py

# Terminal 3
rviz2
# Adicionar: /camera/image_raw + /debug_markers
# Verificar: esferas verdes devem estar sobre robôs
```

### 2. **scripts/test_frame_reference_modes.py** (NOVO)
   - Teste educacional que simula os 2 modos
   - Demonstra por que 560558b causou erro
   - Mostra como TF resolveria o problema

**Como usar**:
```bash
python3 scripts/test_frame_reference_modes.py
# Output: Tabela comparativa dos 3 cenários
```

---

## 🎯 Fluxo de Diagnóstico Recomendado

```
┌─────────────────────────────────────────┐
│  1. Ler RESEARCH_SUMMARY.md (5 min)     │
└──────────────────┬──────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
   ┌────▼─────┐         ┌─────▼──────┐
   │ Rápido?  │         │ Completo?  │
   └────┬─────┘         └─────┬──────┘
        │                     │
   ┌────▼────────────────┐    │
   │ QUICK_DEBUG.md      │    │
   │ + script debug      │    │
   │ (10 min)            │    │
   └────┬────────────────┘    │
        │              ┌──────▼──────────────┐
        │              │ DIAGNOSIS.md        │
        │              │ + TF_FRAMES.md      │
        │              │ + test script       │
        │              │ (60 min)            │
        │              └──────┬──────────────┘
        │                     │
        └──────────┬──────────┘
                   │
        ┌──────────▼──────────┐
        │  Problema resolvido? │
        └──────────┬──────────┘
                   │
          ┌────────┴────────┐
      ┌───▼──┐          ┌───▼──┐
      │ SIM  │          │ NÃO  │
      └───┬──┘          └───┬──┘
          │                 │
      Pronto!      Implementar TF
      Dataset      (robusto)
```

---

## 📋 Checklist: Se Bboxes Estão Desalinhadas

- [ ] Executar `debug_camera_validation.py`
- [ ] Abrir RViz2 e verificar esferas verdes
- [ ] Se NÃO alinhadas: Seguir `CAMERA_PROJECTION_QUICK_DEBUG.md`
- [ ] Executar verificações em ordem:
  - [ ] Camera info (fx, fy, cx, cy)
  - [ ] Poses em /robot*/odom
  - [ ] TF frames
  - [ ] Validação matemática
- [ ] Se nenhum falhar: Implementar TF para robustez

---

## 🐛 Análise de Erros Anteriores

### Commit 560558b: "fix: converte odom local..."

**O que fez**: Adicionou offset ROBOT_SPAWN às poses

**Por que falhou**:
- Assumiu que odom era relativo ao spawn (MODO B)
- Mas era realmente coordenadas mundo (MODO A)
- Resultado: duplo offset, bbox deslocado

**Lição**: Nunca assumir frame de referência, sempre validar!

**Ver**: `scripts/test_frame_reference_modes.py` para simulação do erro

---

## 🛠️ Soluções Recomendadas

### Curto Prazo (Validação Imediata)
```bash
ros2 run cerise_nav debug_camera_validation.py
# RViz visual check = padrão-ouro rápido
```

### Médio Prazo (Antes de produção)
```python
# Usar TF para transformação robusta
from tf2_ros import Buffer, TransformListener
transform = tf_buffer.lookup_transform('world', f'{name}/base_footprint')
pose_x = transform.transform.translation.x
# Funciona em qualquer configuração!
```

### Longo Prazo (Documentação)
- Documentar qual frame está sendo usado
- Adicionar testes com dados reais de Gazebo
- Integrar RViz permanentemente no launch

---

## 📊 Tabela de Referência: Estados Possíveis

| Estado | Sintoma | Causa | Solução |
|--------|---------|-------|---------|
| OK (Modo A) | Bbox sobre robô | Odom em mundo | Usar como está |
| Bugado (Modo B) | Bbox na origem | Odom em local, sem offset | Adicionar offset OU usar TF |
| Duplo erro | Bbox distante 2x | Odom mundo + offset | Remover offset |
| Rotação | Bbox rotacionado 90° | Câmera não em pitch=0 | Verificar .world |
| Fora do frame | Bbox desaparece | Pose fora de bounds | Verificar limites projeto |

---

## 🔍 Verificações Rápidas

```bash
# Check 1: Câmera info
ros2 topic echo /camera/camera_info --once | head -20

# Check 2: Poses
ros2 topic echo /robot1/odom --once | grep -A 3 position

# Check 3: TF
ros2 run tf2_tools view_frames

# Check 4: Visual (recomendado)
rviz2  # Abrir + adicionar /debug_markers
```

---

## 📞 FAQ

**P: Por que não basta usar testes unitários?**  
R: Testes usam valores hardcoded. Dados reais de Gazebo podem ser diferentes.

**P: TF é complexo demais?**  
R: Vale a pena. Um vez implementado, funciona em qualquer configuração de ROS.

**P: Quanto tempo para validar?**  
R: Visual check em RViz: 5 min. Diagnóstico completo: 30 min. TF impl: 30 min.

**P: E se camera_info estiver errada?**  
R: Gazebo publica automaticamente. Se estiver default, há plugin quebrado.

---

## 📖 Referências Técnicas

### ROS2 Official
- [ROS2 TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [RViz2 Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html)

### Gazebo + ROS
- [Gazebo Classic Plugins](https://classic.gazebosim.org/tutorials?tut=ros_plugins)
- [gazebo_ros_odometry plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki)

### Visão Computacional
- [Hartley & Zisserman, Ch. 6: Projective Geometry]
- [OpenCV Camera Calibration](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)

---

## 📁 Estrutura de Arquivos

```
/home/yan/Documentos/Projetos/cerise-turtlebot3-nav/
├── CAMERA_PROJECTION_README.md          ← Este arquivo (índice)
├── RESEARCH_SUMMARY.md                   ← Sumário executivo
├── CAMERA_PROJECTION_DIAGNOSIS.md        ← Análise técnica
├── CAMERA_PROJECTION_QUICK_DEBUG.md      ← Guia rápido
├── TF_FRAMES_REFERENCE.md                ← Documentação de frames
├── scripts/
│   ├── debug_camera_validation.py        ← Script ROS2 principal
│   ├── test_frame_reference_modes.py    ← Teste educacional
│   └── debug_projection.py               ← Script de debug antigo
├── src/cerise_nav/cerise_nav/
│   ├── dataset_collector.py              ← Coleta dataset
│   └── projection.py                     ← Cálculos de projeção
├── world_with_camera.world               ← Mundo Gazebo
└── launch/gazebo_2robots.launch.py       ← Launch file
```

---

## 🎓 Recomendação Final

**Se não tem certeza sobre qual modo está sendo usado**:
→ Use TF (solução robusta, funciona em qualquer caso)

**Se tem tempo para entender o problema**:
→ Ler `TF_FRAMES_REFERENCE.md` + executar `test_frame_reference_modes.py`

**Se precisa validar AGORA**:
→ Executar `debug_camera_validation.py` + abrir RViz2

---

**Data**: 2026-05-01  
**Status**: Pesquisa completa, ferramentas criadas, pronto para validação  
**Próxima ação**: Começar por RESEARCH_SUMMARY.md
