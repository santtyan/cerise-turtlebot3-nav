# Texto do Slide — Apresentação CERISE — Junho 2026

---

## Slide 1 — Título

**CERISE**
*Comunicação e Inteligência para Robótica Inteligente, Sensoriamento e Exploração*

**Atualizações de Junho — Aprendizado por Reforço para Alocação de Tarefas Multi-Robô**

*Yan Santos — Junho 2026 — Orientador: Prof. Alisson Brito*
*EMC / UFG — FAPEG*

---

## Slide 2 — Retomada: onde estávamos

**Fevereiro:** câmera overhead + streaming ROS2-Unity (ROS-TCP-Endpoint)
**Março:** pipeline multi-robô — `demand_generator → task_allocator → Nav2 → TurtleBot3`
**Maio:** gêmeo digital com detecção YOLO — **mAP@0.5 = 0,995**, erro médio **4,7 cm**
→ *"Próximo passo: usar as posições do gêmeo digital como estado para um agente de RL"*

**Junho — entregamos exatamente isso:**
→ Agente PPO treinado; pipeline completo integrado ao Gazebo; avaliação estatística rigorosa

**Pergunta de pesquisa (Junho):** Um agente RL (PPO) alimentado pelo gêmeo digital (YOLO)
supera a heurística gulosa *nearest_free* na alocação de tarefas? E a fonte do estado
(YOLO vs odometria) importa?

*[Imagem: A_yolo_detection_grid.jpg — grid de frames da câmera overhead com detecções YOLO (conf. 0,8–1,0)]*

---

## Slide 3 — Arquitetura do Sistema

```
Câmera overhead  →  YOLOv8n  →  /robot_detections (posições, erro médio 4,7 cm)
                                         ↓
                              AllocationEnv (estado: posições + ocupação + demandas futuras)
                                         ↓
                              Política PPO  →  qual robô atende a demanda
                                         ↓
                              Nav2 / ROS2  →  navegação autônoma (TurtleBot3 Waffle)
```

- 3 TurtleBots Waffle no Gazebo Classic — ROS2 Humble + Nav2
- Gêmeo digital via YOLO publica `/robot_detections` em tempo real
- Treinamento offline em ambiente analítico leve — sem Gazebo, sem GPU

*[Imagem: A_yolo_detection_grid.jpg — câmera overhead, detecções YOLO em vários frames]*

---

## Slide 4 — Gêmeo Digital: precisão validada (Maio → alimenta o RL)

**Dataset:** 375 frames anotados manualmente
**Modelo:** YOLOv8n — 6 MB

| Métrica | Valor |
|---|---|
| mAP@0.5 | **0,995** |
| Erro médio (rastreamento contínuo) | **4,7 cm** |
| Detecções < 15 cm | **100%** |

> Precisão suficiente para o alocador tomar decisões corretas — robô detectado sempre no quadrante certo

*[Imagem: B_digital_twin_4cm7_error.png]*

---

## Slide 5 — Formulação RL

**Problema:** MRTA (Multi-Robot Task Allocation) — MDP com agente centralizado único

**Observação (dim = 21):**
- Posições dos 3 robôs via YOLO → *x, y* por robô (6 coords)
- Tempo de ocupação de cada robô (3 valores)
- Demanda atual: origem + destino (*x, y*) (4 coords)
- Próximas 2 demandas em lookahead (8 coords) — **antecipação**

**Ação:** escolher qual robô (0, 1 ou 2) atende a demanda

**Recompensa:** −(tempo\_espera + tempo\_viagem) — minimiza *response_time*

**Treinamento:** PPO (Stable-Baselines3), 500k timesteps, ~2h CPU

**Ablação:** PPO(YOLO) vs PPO(odom) — isola o impacto do gêmeo digital

*[Imagem: H_mdp_diagram.png — diagrama do ciclo Estado→Agente→Ação→Ambiente→Recompensa]*

---

## Slide 6 — Curva de Aprendizado

*[Imagem: C2_rl_learning_curve.png — curva PPO(YOLO) vs PPO(odom) convergindo]*

- Ambas as variantes aprendem — reward sobe de ~−21 para ~−17 em 500k steps
- PPO(odom) converge mais rápido e termina **ligeiramente acima** do PPO(YOLO)
  → o ruído de 4,7 cm do YOLO custa pouco, mas **não é zero** (confirmado na ablação)
- Linha tracejada: nearest\_free (−16,7) — **o baseline ainda supera as duas variantes**

---

## Slide 7 — Resultados: Ablação e Comparação

*[Imagem: C_rl_ablation_atual.png — barras: latência média / p95 / makespan / desbalanceamento]*

| Política | Latência média | Latência p95 | Makespan | Desbalan. |
|---|---|---|---|---|
| **nearest\_free** | **16,83s** | **26,73s** | **336,58s** | **0,24** |
| PPO (YOLO) | 17,83s | 27,84s | 356,63s | 0,32 |
| PPO (odom) | 17,12s | 27,18s | 342,34s | 0,31 |

*[Imagem: G_rl_latency_boxplot.png — boxplot completo: Random / RoundRobin / nearest_free / PPO(YOLO) / PPO(odom) / Clairvoyant / Oráculo]*

**O achado — invalid\_rate (principal suspeito do gap):**
- PPO escolhe robô **ocupado em 4,0%** das decisões; nearest\_free em apenas **0,3%** (≈13×)
- PPO aprende a restrição *soft* (via penalidade); guloso a respeita por *construção* (hard)
- **Hipótese:** o custo de restrições soft explica boa parte do gap. A confirmação
  (causal) virá ao reformular com *action masking* e medir se o gap fecha — próximo passo
- **Hipótese complementar (discutida c/ orientador):** ambiente pequeno + apenas 4 waypoints
  limitam a vantagem de antecipação que o RL poderia explorar

---

## Slide 8 — Curva Carga × Resposta (robustez)

*[Imagem: F_load_response_curve.png]*

| inter\_arrival | Oracle | nearest\_free (gap) | PPO(YOLO) (gap) |
|---|---|---|---|
| 30s (baixa) | 324,7s | 333,8s (+2,8%) | 397,3s (+22,3%) |
| 12s (média) | 333,2s | 355,4s (+6,7%) | 399,1s (+19,8%) |
| 10s (alta) | 337,0s | 371,1s (+10,1%) | 399,8s (+18,6%) |

*(gap = % acima do oráculo clarividente; valores reproduzidos em `sweep_load.py`, 500 ep/ponto)*

- Sob carga alta o **nearest\_free degrada** (gap +2,8% → +10,1%) enquanto o **PPO fica plano** (~397s)
- A distância entre eles encolhe, mas **não há crossover** no intervalo testado (30→10s)

---

## Slide 9 — Conclusão e Próximos Passos

**Contribuições de Junho:**
✅ Gêmeo digital YOLO integrado como estado do agente RL (4,7 cm, mAP = 0,995)
✅ Pipeline completo: câmera → YOLO → AllocationEnv → PPO → Nav2 → TurtleBot3
✅ Ambiente analítico leve + oráculo beam-search como teto de referência
✅ Ablação YOLO vs odometria como variável experimental controlada
✅ Avaliação rigorosa (Wilcoxon + Cliff's δ + IC bootstrap, 1000 ep pareados)
✅ **Resultado negativo bem caracterizado:** PPO não supera o guloso (gap +20% vs +7,1%)
✅ **Achado mensurado:** invalid\_rate 4% vs 0,3% (≈13×) — principal suspeito do gap

**Próximos passos:**
- Reformular RL com **action masking** (restrição hard) — eliminar o invalid\_rate
- Coleta Gazebo ≥ 60 demandas para validação em planta real
- Submissão LARS — deadline: **15/07/2026**

**Trabalhos futuros:** alocação descentralizada (Dec-POMDP / MARL) — cada robô como agente
com observação local e leilão de tarefas (CTDE)

---

## Notas para a apresentação

**Fio narrativo com as apresentações anteriores:**
- Citar Fevereiro (câmera + Unity), Março (pipeline multi-robô), Maio (YOLO digital twin)
- Junho fecha o ciclo: entregamos o que o Maio prometia como próximo passo

**Sobre o PPO não superar o baseline — enquadramento correto (Prof. Alisson já concordou):**
- NÃO é falha — "é ciência, você tenta o melhor, caso não dê a gente escreve e relata os achados"
- Principal suspeito do gap: invalid\_rate (PPO soft vs guloso hard). Confirmação causal = action masking (próximo passo)
- "Quantificar a limitação e propor a correção" É a contribuição científica

**Se perguntarem "então o RL não serve?":**
> "Serve quando a restrição está codificada (action masking) ou há antecipação que o guloso não explora. Mostramos o limite atual — é essa quantificação que vai para o paper."

**Se perguntarem sobre MARL:**
> "Extensão natural como Dec-POMDP — trabalhos futuros com parágrafo já redigido."

**Números canônicos (não misturar) — TODOS reproduzidos em 26/06 com eval_policy.py:**
- Erro YOLO: **4,7 cm** (rastreamento contínuo) — usar sempre este
- 1,24 cm = benchmark estático de 8 poses, não usar nos slides
- PPO gap: **+20%** vs oráculo; nearest\_free: **+7,1%**; invalid\_rate **4,2% vs 0,3%**
- p < 0,001; Cliff δ grande; NF vence **72%** das divergências PPO vs NF

**⚠️ CRÍTICO antes da apresentação — NÃO re-rodar eval sem checar o csv:**
- O `~/cerise_log.csv` foi **sobrescrito pelos runs do Gazebo de hoje (09:36–09:49)** com
  latências de colisão (105s, 215s) → calibração quebra (v_nominal 0,120 → 0,052) e os
  números viram lixo (invalid_rate 33%, gap +15%).
- Estado já restaurado para o default limpo (v_nominal 0,120). O backup poluído está em
  `~/cerise_log_POLUIDO_gazebo_20260626.csv`. **As figuras dos slides estão corretas.**

**Imagens por slide:**
- Slide 2: A\_yolo\_detection\_grid.jpg (grid de frames com detecções YOLO)
- Slide 3: A\_yolo\_detection\_grid.jpg (câmera overhead)
- Slide 4: B\_digital\_twin\_4cm7\_error.png (erro de localização)
- Slide 5: **H\_mdp\_diagram.png** (MDP: Estado→Agente→Ação→Ambiente→Recompensa)
- Slide 6: **C2\_rl\_learning\_curve.png** (curva de aprendizado PPO(YOLO) vs PPO(odom))
- Slide 7: **C\_rl\_ablation\_atual.png** (4 barras) + **G\_rl\_latency\_boxplot.png** (boxplot 7 políticas)
- Slide 8: F\_load\_response\_curve.png (carga vs response_time)
- *(E\_allocation\_animated.gif disponível como B-roll, fora dos slides principais)*

**Tempo estimado:** 12–15 min + perguntas
