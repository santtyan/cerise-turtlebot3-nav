# CERISE — Gêmeo Digital com Detecção YOLO para Robôs Móveis

**Apresentação · 10 minutos**

---

## 1. Motivação

> *"Como saber onde cada robô está, em tempo real, sem depender só da odometria?"*

- Frotas multi-robô em ambientes indoor não têm GPS
- Odometria acumula drift ao longo do tempo
- **Proposta:** câmera overhead + YOLO → posição visual independente
- **Resultado:** gêmeo digital sincronizado com erro médio de **4,7 cm**

---

## 2. Arquitetura do Sistema

```
Gazebo (simulação)
  ├── TurtleBot3 Robot 1  ──┐
  └── TurtleBot3 Robot 2  ──┤──► Nav2 (navegação autônoma)
                             │
  Câmera overhead (3m, FOV 60°)
        │
        ▼ /camera/image_raw
  ┌─────────────────────┐
  │   yolo_detector.py  │  ← YOLOv8n, 36ms/frame, CPU
  └─────────────────────┘
        │
        ├── /robot_detections  (PoseArray — posições estimadas)
        ├── /detection_image   (frame anotado com bboxes)
        └── /detection_error   (erro euclidiano vs odometria)
```

**Stack:** ROS2 Humble · Gazebo Classic · Nav2 · YOLOv8n · Python 3

---

## 3. Pipeline de Coleta de Dataset

### Por que não gravar vídeo contínuo?
Sincronização temporal frágil entre imagem e odometria gera labels ruidosos.

### Solução: `collect_teleport.py`

1. Teleporta robôs para pose fixa (posição + orientação)
2. Aguarda estabilização → captura imagem
3. Gera label YOLO automaticamente via projeção câmera→mundo
4. Avança para próxima pose

| Parâmetro | Valor |
|---|---|
| Combos de posição | 30 |
| Variações de yaw | × 4 |
| Runs com jitter | × 3 |
| **Total de frames** | **≈ 375** |

---

## 4. Desafio Técnico: Projeção da Câmera Overhead

Câmera com pitch = π/2 (apontada para baixo) cria uma rotação de eixos não trivial.

### Bug encontrado durante validação

| | Mapeamento | Erro resultante |
|---|---|---|
| ❌ Antes do fix | `world_y = +raw_x` | **1,3 m** |
| ✅ Após o fix | `world_y = -raw_x` | **4,7 cm** |

### Mapeamento correto (pitch = π/2)

```
# forward:  world → pixel
cam_x = -world_y
cam_y = -world_x

# inverse:  pixel → world
world_x =  raw_y
world_y = -raw_x
```

> Lição: sempre validar projeção visualmente antes de treinar o modelo.

---

## 5. Treinamento YOLO

**Modelo:** YOLOv8n (nano) — 6 MB, sem GPU necessária

| Parâmetro | Valor escolhido | Motivo |
|---|---|---|
| `imgsz` | 416 | 640 = 6h de treino; 416 = 1,5h, mAP similar |
| `lr0` | 0,001 | 0,01 causava divergência com 1 classe |
| `batch` | 8 | batch=16 instável com ~300 frames |
| `epochs` | 50 | patience=15 parava antes do limite |
| `mosaic` | 0,5 | dataset homogêneo — mosaic alto prejudicava |
| `seed` | 42 | reprodutibilidade garantida |

**Tempo de treino:** ~1,5h em CPU (i7)

### Curvas de Treinamento

![Curvas de treino](results/results.png)

---

## 6. Resultados do Modelo

### Métricas de Validação

| Métrica | Valor |
|---|---|
| Precision | **0,999** |
| Recall | **1,000** |
| mAP@0.5 | **0,995** |
| mAP@0.5–0.95 | 0,880 |
| Inferência (CPU) | 36 ms/frame (~27 FPS) |

### F1-Confidence Curve

![F1 Curve](results/BoxF1_curve.png)

> F1 = 1,00 mantido de confidence=0 até 0,73 — modelo extremamente estável.

### Matriz de Confusão Normalizada

![Confusion Matrix](results/confusion_matrix_normalized.png)

> Zero falso negativo. Zero falso positivo.

### Detecções no Conjunto de Validação

![Validação](results/val_batch0_pred.jpg)

> 16 frames, 2 robôs por frame, confiança consistente entre 0,8 e 1,0.

---

## 7. Validação do Gêmeo Digital

**Experimento:** 2 robôs navegando via Nav2, goals aleatórios, coleta contínua online.

| Métrica | Valor |
|---|---|
| Amostras coletadas | 2.031 |
| Erro médio | **4,7 cm** |
| Mediana | 4,7 cm |
| Percentil 95 | 7,4 cm |
| Erro máximo | 11 cm |
| % abaixo de 15 cm | **100%** |

### Distribuição do Erro de Localização

![Digital Twin Error](../digital_twin_error.png)

> A CDF mostra que 100% das detecções ficaram abaixo do alvo de 15 cm — com robôs em movimento real via Nav2.

---

## 8. Conclusão

### O que foi entregue

| Componente | Status | Métrica-chave |
|---|---|---|
| Dataset de treino | ✅ | 375 frames, pipeline reproduzível |
| Modelo YOLO | ✅ | mAP@0.5 = 0,995, 36ms/frame, CPU |
| Gêmeo digital online | ✅ | Erro médio = 4,7 cm, 100% < 15 cm |

### Contribuições técnicas

- Pipeline de coleta via teleporte (elimina problema de sincronização temporal)
- Correção do mapeamento de eixos para câmera com pitch = π/2
- Nó ROS2 de inferência em tempo real com publicação de erro vs ground truth

### Próximos passos

1. Integrar `/robot_detections` ao `task_allocator` (fusão visual + odometria)
2. Validar em hardware real (TurtleBot3 Waffle)
3. Pipeline completo: `demand_generator → task_allocator → Nav2 → YOLO`

---

## Trabalhos Futuros — YOLO como Exit Condition em Aprendizado por Reforço

### Motivação

O pipeline YOLO entrega **4,7 cm de erro médio** na estimativa de posição dos robôs — independente da odometria. Isso abre uma oportunidade direta em RL: usar o gêmeo digital como **árbitro de verdade** no loop de aprendizado.

### O problema do RL tradicional com TurtleBot3

Nos frameworks DRL existentes (TD3, SAC, PPO com Nav2), o término de episódio é decidido pela **odometria interna do robô**:

```
exit condition atual:
  dist_odom(robot, goal) < threshold → episódio termina
```

Odometria acumula drift. O agente aprende com um sinal de término **ruidoso** — o robô acha que chegou, mas não chegou. Isso prejudica convergência e qualidade da política aprendida.

### Proposta: YOLO como Exit Condition (β)

No **Options Framework** (Sutton, Precup & Singh, 1999), uma opção é definida como:

```
Option = (I,  π,  β)
          ↑   ↑   ↑
    initiation  policy  termination = EXIT
```

**β (exit condition)** determina quando o episódio/opção termina. A proposta é substituir a odometria pelo gêmeo digital YOLO:

```
exit condition proposta:
  dist_yolo(robot_detected, goal) < 0.15m → episódio termina com sucesso
```

### Arquitetura completa

```
Gazebo + Nav2
     │
     ├── /odom           → features internas (LiDAR, velocidade)
     │
     └── /camera/image   → yolo_detector → /robot_detections
                                                  │
                                     ┌────────────┴───────────┐
                                     ↓                        ↓
                              reward_function           exit_condition (β)
                           r = −dist_yolo(robot, goal)   chegou? YOLO confirma
                                     │                        │
                                     └───────────┬────────────┘
                                                 ↓
                                       Agente RL (TD3 / PPO)
                                                 ↓
                                      ação: novo goal → Nav2
```

### Por que isso é uma contribuição nova

| | RL padrão (odometria) | RL com YOLO (proposta) |
|---|---|---|
| **Exit condition** | odometria com drift | YOLO visual, 4,7 cm |
| **Reward signal** | distância estimada | distância real do mundo |
| **Convergência** | sinal ruidoso | ground truth externo |
| **Novidade** | baseline conhecido | **sem precedente direto** |

Nenhum dos frameworks TurtleBot3+DRL existentes usa câmera overhead + YOLO como exit condition. Esta é a lacuna que o trabalho futuro preenche.

---

*CERISE · ROS2 Humble · YOLOv8n · Gazebo · Nav2*

---

## 9. Revisão Bibliográfica — Contexto e Ferramenta

### Ferramenta: Parsif.al

A revisão sistemática deste projeto será conduzida no **[Parsif.al](https://parsif.al)** — plataforma web que estrutura o processo em três fases: **planejamento → condução → relatório**, com suporte a equipes distribuídas e publicação do protocolo completo.

**Por que Parsif.al para este projeto:**
- Permite registrar formalmente as strings de busca, critérios de inclusão/exclusão e bases consultadas
- Facilita auditoria do processo pelo orientador (Prof. Alisson)
- Exporta referências e síntese no formato exigido por conferências como LARS

---

### Temas da Revisão

A revisão deve cobrir **4 eixos temáticos**, todos diretamente relacionados ao CERISE:

| Eixo | Pergunta de pesquisa |
|---|---|
| **1. Gêmeo digital para robótica móvel** | Como sincronizar estado físico e virtual em tempo real? |
| **2. Localização visual por câmera overhead** | Qual é a precisão alcançável com câmera externa vs. odometria? |
| **3. Detecção de objetos com YOLO** | YOLOv8 é state-of-the-art para detecção de robôs em simulação? |
| **4. Alocação de tarefas multi-robô (MRTA)** | Como a posição estimada pelo gêmeo digital melhora a alocação? |

---

### Literatura Relevante Identificada

#### Eixo 1 — Gêmeo Digital + Robótica Móvel

- **DTU (2022):** *Building Digital Twin of Mobile Robotics Testbed Using Centralized Localization System* — câmera overhead + EKF para estimar estado do robô físico. Abordagem mais próxima do CERISE.
  - [orbit.dtu.dk](https://orbit.dtu.dk/en/publications/building-digital-twin-of-mobile-robotics-testbed-using-centralize)

- **Springer (2025):** *Evaluating mobile robot navigation behavior in flexible assembly systems through digital twin and real-world experiments* — validação de gêmeo digital com experimentos reais e simulação.
  - [link.springer.com](https://link.springer.com/article/10.1007/s44430-025-00010-4)

- **Springer (2025):** *Distributed multi-robot task dynamic allocation in digital-twin factory towards industry 5.0* — fusão de gêmeo digital com MRTA em ambiente industrial.
  - [tandfonline.com](https://www.tandfonline.com/doi/full/10.1080/00207543.2025.2499866)

#### Eixo 2 — Câmera Overhead e Localização Visual

- **ScienceDirect (2015):** *A Mobile Robot Localization using External Surveillance Cameras at Indoor* — baseline clássico para câmera externa indoor.
  - [sciencedirect.com](https://www.sciencedirect.com/science/article/pii/S1877050915017238)

- **ResearchGate (2024):** *Multi-camera multi-robot visual localization system* — extensão com múltiplas câmeras e Kalman adaptativo.
  - [researchgate.net](https://www.researchgate.net/publication/384171340_Multi-camera_multi-robot_visual_localization_system)

- **ScienceDirect (2025):** *AI-based approaches for improving autonomous mobile robot localization in indoor environments* — revisão abrangente de métodos AI para localização indoor.
  - [sciencedirect.com](https://www.sciencedirect.com/science/article/pii/S2215098625000321)

#### Eixo 3 — Detecção com YOLO

- **Journal of Robotics and Control (2025):** *Object Detection for Wheeled Mobile Robot Based Using Deep Learning: Systematic Review* — revisão sistemática diretamente aplicável ao CERISE.
  - [journal.umy.ac.id](https://journal.umy.ac.id/index.php/jrc/article/view/24979)

- **arXiv (2022):** *Lightweight Multi-Drone Detection and 3D-Localization via YOLO* — detecção e localização 3D com YOLO, caso análogo ao CERISE com câmera overhead.
  - [arxiv.org](https://arxiv.org/pdf/2202.09097)

#### Eixo 4 — Multi-Robot Task Allocation (MRTA)

- **ACM Computing Surveys (2024):** *A Systematic Literature Review on Multi-Robot Task Allocation* — referência canônica para MRTA, cobre taxonomia completa.
  - [dl.acm.org](https://dl.acm.org/doi/10.1145/3700591)

- **arXiv (2025):** *Integrating LLMs and Digital Twins for Adaptive Multi-Robot Task Allocation in Construction* — tendência futura: LLM + gêmeo digital para alocação adaptativa.
  - [arxiv.org](https://arxiv.org/html/2506.18178v1)

---

### Strings de Busca para Parsif.al

As strings estão ordenadas do **núcleo do pipeline** (YOLO + detecção de robôs) para o contexto mais amplo (RL + gêmeo digital). Execute nessa ordem — comece pelo que você construiu.

---

#### String 1 — Núcleo do pipeline: detecção de robôs com YOLO
> Foco: o que existe de detecção visual de robôs móveis com redes neurais

```
("YOLO" OR "YOLOv8" OR "object detection")
AND ("mobile robot" OR "ground robot" OR "TurtleBot" OR "wheeled robot")
AND ("detection" OR "localization" OR "position estimation")
```
**Pergunta:** YOLOv8 é estado da arte para detectar robôs móveis em simulação/laboratório?

---

#### String 2 — Câmera overhead como sensor externo de localização
> Foco: câmera fixada no teto/ambiente para localizar robôs (exatamente o seu setup)

```
("overhead camera" OR "ceiling camera" OR "top-view camera" OR "external camera")
AND ("robot localization" OR "robot detection" OR "robot tracking")
AND ("indoor" OR "simulation" OR "Gazebo")
```
**Pergunta:** Qual a precisão reportada na literatura para localização por câmera overhead?

---

#### String 3 — Dataset sintético para treino de detecção em simulação
> Foco: validar sua escolha de coletar dataset em Gazebo em vez de ambiente real

```
("synthetic dataset" OR "simulation dataset" OR "Gazebo dataset")
AND ("object detection" OR "YOLO" OR "deep learning")
AND ("robot" OR "mobile robot" OR "autonomous vehicle")
AND ("sim-to-real" OR "domain adaptation" OR "transfer learning")
```
**Pergunta:** Modelos treinados em simulação generalizam para ambientes reais?

---

#### String 4 — Gêmeo digital para monitoramento de frotas robóticas
> Foco: sincronização digital twin ↔ robô físico em tempo real

```
("digital twin" OR "cyber-physical system")
AND ("mobile robot" OR "multi-robot" OR "robot fleet")
AND ("real-time" OR "synchronization" OR "state estimation")
AND ("localization" OR "position" OR "tracking")
```
**Pergunta:** Como sistemas de gêmeo digital monitoram posição de robôs em tempo real?

---

#### String 5 — Aprendizado por reforço com câmera/visão para navegação
> Foco: RL usando percepção visual como observação ou exit condition

```
("reinforcement learning" OR "deep reinforcement learning")
AND ("robot navigation" OR "mobile robot" OR "TurtleBot")
AND ("visual" OR "camera" OR "image" OR "object detection")
AND ("reward" OR "observation" OR "termination" OR "exit condition")
```
**Pergunta:** Qual o impacto de usar posição visual (vs. odometria) como sinal de recompensa/término em RL?

---

#### String 6 — Gêmeo digital + aprendizado por reforço (trabalho futuro)
> Foco: literatura que une as duas contribuições — o que já existe antes de você propor

```
("digital twin" OR "simulation")
AND ("reinforcement learning")
AND ("mobile robot" OR "multi-robot" OR "autonomous robot")
AND ("reward" OR "training" OR "policy" OR "exit")
```
**Pergunta:** Gêmeo digital já foi usado como exit condition ou fonte de recompensa em RL para robótica móvel?

---

**Bases recomendadas (por string):**

| String | Bases prioritárias |
|---|---|
| 1, 2, 3 | IEEE Xplore · arXiv · ACM DL |
| 4 | Scopus · Springer · Web of Science |
| 5, 6 | arXiv · IEEE Xplore · NeurIPS/ICML proceedings |

**Período:** 2019–2026 (strings 1–4) · 2022–2026 (strings 5–6)

---

**Critérios de inclusão:**
- Avalia erro de localização ou detecção quantitativamente
- Usa câmera como sensor primário (não só LiDAR)
- Ambiente simulado ou laboratorial controlado
- Publicado em conferência ou periódico revisado por pares

**Critérios de exclusão:**
- Foco exclusivo em UAV/drone sem analogia para ground robots
- Sem avaliação experimental
- Anterior a 2019 (exceto baseline clássico — e.g., Options Framework 1999)
