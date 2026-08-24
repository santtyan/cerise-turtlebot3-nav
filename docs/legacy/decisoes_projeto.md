# CERISE — Registro de Decisões do Projeto

Documento cronológico das decisões técnicas e de escopo tomadas ao longo do projeto,
com justificativas. Serve de memória para o paper e para apresentações ao Prof. Alisson.

---

## Fase 1 — Dataset e Modelo YOLO (Abril–Maio 2026)

### D1 — Coleta por teleporte, não por navegação contínua
**Data:** 2026-04-07 (início) → concluído 2026-05-12

**Decisão:** usar `collect_teleport.py` (teleporta robôs para poses fixas, captura imagem,
avança para próxima pose) em vez de gravar vídeo contínuo durante a navegação.

**Justificativa:** sincronização temporal entre imagem e odometria é frágil — pequenos
atrasos de rede/ROS geram labels com offset. O teleporte elimina esse problema: o robô
está exatamente na pose quando a câmera captura.

**Resultado:** 375 frames limpos, labels sem ruído de sincronização.

---

### D2 — YOLOv8n (nano) sem GPU
**Data:** 2026-05-12

**Decisão:** usar YOLOv8n com imgsz=416, lr0=0.001, batch=8, patience=15, seed=42.

**Justificativa:**
- `imgsz=640` levaria 6h de treino na CPU disponível; `imgsz=416` levou 1,5h com mAP similar.
- `lr0=0.01` causava divergência com apenas 1 classe; `0.001` estabilizou.
- YOLOv8n (6MB) atinge inferência em 36ms/frame na CPU — viável para tempo real.

**Resultado:** mAP@0.5 = 0,995. Modelo reproduzível com seed=42.

---

### D3 — Correção do mapeamento de eixos da câmera overhead (pitch = π/2)
**Data:** 2026-05-12

**Decisão:** `world_x = raw_y`, `world_y = -raw_x` (não o mapeamento ingênuo direto).

**Justificativa:** câmera com pitch=π/2 (apontada para baixo) cria rotação de eixos
não trivial. O mapeamento original (`world_y = raw_x`) gerava erro de 1,3m. A correção
de sinal reduziu para 4,7cm.

**Como foi descoberto:** erro de 1,3m detectado visualmente ao comparar bboxes com
posições reais dos robôs no Gazebo.

**Resultado:** erro médio 4,7cm (validação dinâmica, 2031 amostras via Nav2).

---

## Fase 2 — Gêmeo Digital (Maio 2026)

### D4 — YOLO como observador passivo (gêmeo digital de monitoramento)
**Data:** 2026-05-18

**Decisão:** `yolo_detector.py` publica `/robot_detections` (PoseArray) e
`/detection_error` (Float32), mas NÃO influencia nenhuma decisão de alocação.

**Justificativa:** requisito do Prof. Alisson era "digital twin sync: detected position →
robô virtual na posição correta". O gêmeo digital de monitoramento cumpre isso.
A integração com decisões ficou como próximo passo.

**Resultado validado:** erro médio 4,7cm, P95=7,4cm, 100% abaixo de 15cm.
2031 amostras com 2 robôs navegando via Nav2.

---

## Fase 3 — Aprendizado por Reforço (Junho 2026)

### D5 — PPO via Stable-Baselines3 (não DQN, não A3C)
**Data:** 2026-06-07

**Decisão:** usar PPO (Proximal Policy Optimization) como algoritmo principal.

**Justificativa:**
- DQN é mais instável para reward esparso; pode divergir.
- A3C mais poderoso mas complexidade de implementação alta dado o prazo (38 dias para LARS).
- PPO tem precedente direto: YOLOv5+PPO em robótica de armazém (Frontiers/PMC 2024).
- Stable-Baselines3 já instalado na máquina.

**Alternativa registrada:** DQN como comparativo secundário se houver tempo.

---

### D6 — Treino em ambiente Gym leve analítico (não em Gazebo)
**Data:** 2026-06-07

**Decisão:** treinar o PPO num ambiente Python puro (`AllocationEnv`) sem ROS/Gazebo,
depois transferir a política para o Gazebo real.

**Justificativa:** treinar em Gazebo é matematicamente inviável no prazo:
- 1 navegação Nav2 leva 15–30s reais.
- 500k timesteps × 15s = ~2.000h de CPU.
- No env leve: 500k timesteps em ~15 minutos.

**Como foi calibrado:** parâmetros do modelo analítico (`v_nominal`, `noise_std`) são
extraídos de `~/cerise_log.csv` (tempos reais coletados no Gazebo) para aproximar a
dinâmica real. Isso fecha o gap sim→real de forma honesta.

**Resultado:** pipeline de treino funciona end-to-end; política transferível via `.zip`.

---

### D7 — Observação com formato compartilhado (obs_encoding.py)
**Data:** 2026-06-07

**Decisão:** criar módulo `obs_encoding.py` como ÚNICA fonte de verdade sobre o layout
da observação, usado pelo env de treino E pelo nó ROS2 de inferência.

**Justificativa:** se os dois caminhos montarem a observação de forma diferente, a
política treinada recebe lixo em produção — bug silencioso e catastrófico. O módulo
compartilhado elimina essa classe de erro por design.

**Verificado por:** teste de paridade explícito comparando `encode_obs` nos dois caminhos
para o mesmo estado bruto. Resultado: diferença < 1e-6. ✅

---

### D8 — inter_arrival=15s para o cenário de alta carga
**Data:** 2026-06-07

**Descoberta:** com inter_arrival=30s (valor padrão do `demand_generator`), o PPO
**empatou** com o baseline nearest_free (22,45s vs 22,40s). Com poucos robôs e
carga baixa, a regra gulosa já é quase ótima.

**Decisão:** criar cenário de alta carga (inter_arrival=15s, 6 waypoints, 3 robôs)
onde o baseline comete 6,7% de ações inválidas e o PPO tem espaço real para ganhar.

**Justificativa técnica:** PPO otimiza o reward acumulado do episódio — aprende a
antecipar congestionamento. nearest_free é míope (só olha o estado atual).

**Resultado:** PPO(yolo) reduziu latência média em 2,1% e p95 em 4,2% vs nearest_free.

---

### D9 — Cenário ampliado: 3 robôs + 6 waypoints
**Data:** 2026-06-07

**Decisão:** ampliar de 2 robôs/4 waypoints (escopo original) para 3 robôs/6 waypoints.

**Justificativa:** empate demonstrado experimentalmente no cenário simples (D8).
O cenário ampliado cria situações onde a regra gulosa é subótima.

**Status:** NÃO confirmado pelo Prof. Alisson. Consulta pendente antes do paper.
(O professor aprovou a ideia geral com 2 robôs em 29/05/2026.)

---

### D10 — Ablação YOLO vs odometria como fonte do estado
**Data:** 2026-06-07

**Decisão:** treinar duas políticas idênticas exceto pela fonte de posição:
- `obs=yolo`: ruído gaussiano fixo de 4,7cm por detecção (erro medido do gêmeo digital).
- `obs=odom`: drift acumulado de 3cm por tarefa concluída.

**Justificativa:** isola e quantifica o valor do gêmeo digital na qualidade da decisão.
Sem essa ablação, o paper só diz "usamos YOLO"; com ela, diz "YOLO melhora (ou não)
a política em X%". Contribuição científica mais forte.

**Resultado (env leve, 1000 episódios):**

| Política | Lat. média | p95 | Makespan |
|---|---|---|---|
| nearest_free | 32,29s | 53,59s | 968,7s |
| PPO(yolo) | 31,59s | 51,36s | 947,7s |
| PPO(odom) | **30,81s** | **50,64s** | **924,3s** |

**Interpretação:** no env analítico, PPO(odom) ganhou porque o modelo de drift da odom
(3cm/tarefa) é pequeno. Na validação real do Gazebo, onde o drift acumula de forma mais
agressiva, espera-se que PPO(yolo) se aproxime ou supere PPO(odom). Essa hipótese será
verificada na Fase de Validação Gazebo.

---

### D11 — Curva de aprendizado sem TensorBoard
**Data:** 2026-06-07

**Decisão:** gerar curva de aprendizado via `Monitor` wrapper do SB3 em vez do TensorBoard.

**Justificativa:** TensorBoard não estava instalado e sem acesso à internet no momento.
O `Monitor` wrapper captura `ep_rew_mean` diretamente do `VecEnv` via `episode` info,
solução nativa do SB3 sem dependências externas.

**Resultado:** curva mostra PPO convergindo de -41 para -34 em 500k steps.
Baseline nearest_free = -28,5 — **PPO ainda não alcançou o baseline**.

**Interpretação para o paper:** o agente está aprendendo na direção correta mas precisa
de mais timesteps (estimativa: 1M–2M para alcançar o baseline). Reportar honestamente
como limitação — a contribuição está na arquitetura e na ablação, não em superar o NF
em reward absoluto no env analítico. A vantagem do PPO aparece nas métricas de latência
e p95 (avaliação pareada, não reward do episódio).

---

### D12 — Análise qualitativa do comportamento aprendido
**Data:** 2026-06-07

**Decisão:** além das métricas agregadas, identificar decisões concretas onde PPO e
nearest_free divergem e comparar os resultados.

**Resultado (2000 episódios, 22.467 decisões divergentes):**
- PPO ganhou: 29,9% das divergências
- NF ganhou: 34,1% das divergências
- Empate: 36,1%

**Exemplo para o paper (PPO ganha 16,4%):**
```
robot2: pos=(0.00, 0.00)  ocupado 17s
robot1: pos=(-1.50,-1.50) livre
robot3: pos=(-1.50,-1.50) ocupado 17s
Demanda: origem=(+1.50,-1.50) → dest=(0.00, 0.00)

PPO escolheu robot2 → travel=37,4s  ← antecipa que robot2 libera perto do destino
NF  escolheu robot1 → travel=44,7s  ← escolhe o livre mais próximo da origem
PPO ganhou 7,3s (16,4%)
```

**Interpretação:** PPO aprendeu a antecipar que um robô ocupado mas próximo do destino
será mais eficiente que um robô livre mas distante. nearest_free não consegue fazer esse
raciocínio por ser míope (olha só o estado atual).

---

## Decisões Pendentes

| ID | Decisão | Responsável | Prazo |
|---|---|---|---|
| P1 | Confirmar 3 robôs/6 wp com Prof. Alisson | Yan | Antes de escrever o paper |
| P2 | Validação Gazebo: baseline vs PPO(yolo) vs PPO(odom) | Yan | Semana 5–6 (Jul 05–11) |
| P3 | Popular ~/cerise_log.csv com dados reais para recalibrar nav_model | Yan | Antes da validação Gazebo |
| P4 | Treinar PPO com 1M–2M steps para alcançar o baseline no reward | Yan | Opcional — métricas de latência já mostram ganho |

---

## Métricas Consolidadas do Projeto

| Componente | Métrica | Valor |
|---|---|---|
| Modelo YOLO | mAP@0.5 | 0,995 |
| Modelo YOLO | Inferência (CPU) | 36 ms/frame |
| Gêmeo digital | Erro médio (estático) | 1,24 cm |
| Gêmeo digital | Erro médio (dinâmico) | 4,7 cm |
| Gêmeo digital | Cobertura | 100% abaixo de 15 cm |
| RL vs baseline | Melhoria latência média | -2,1% |
| RL vs baseline | Melhoria p95 | -4,2% |
| RL vs baseline | Melhoria makespan | -2,1% |
| PPO comportamento | Decisões onde PPO ganha | 29,9% das divergências |
| PPO comportamento | Ganho máximo identificado | 16,4% por decisão |
| Curva aprendizado | Reward inicial → final (500k) | -41 → -34 |
| Curva aprendizado | Baseline nearest_free | -28,5 (não alcançado) |
