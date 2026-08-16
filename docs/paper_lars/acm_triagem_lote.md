# Triagem em lote do ACM Digital Library (356 resultados)

## Passo 1 — Rejeitar em massa via "Search within results"

Na aba Study Selection > ACM Digital Library, use o campo "Search within results" com cada termo abaixo, marque "Select All on Page" (ou selecione manualmente os resultados) e use Action > Reject. Repita para cada termo:

1. `Proceedings of` — pega quase todos os "XXXX '24: Proceedings of..." (containers de conferência sem paper real)
2. `Survey` — remove surveys (excluídos pelo critério de seleção)
3. `Crowdsensing` — fora de escopo (não é robótica física)
4. `Crowdsourcing` — fora de escopo
5. `Edge Computing` — fora de escopo (a menos que mencione robôs explicitamente — checar antes de rejeitar se aparecer robot/UAV junto)
6. `Cloud Computing` — fora de escopo
7. `Job Scheduling` sem `robot` — geralmente cloud/HPC, não MRTA físico
8. `UAV` isolado (sem `task allocation` ou `multi-robot`) — muitos são networking/communication, não RL de alocação

**Atenção**: alguns termos acima (UAV, edge computing) podem capturar falsos positivos que são relevantes (ex.: "Scalable Multi-Agent RL for UAV Scheduling in Multi-Hop Emergency Networks" é mais sobre rede de comunicação que MRTA — excluir; mas confira título por título antes de rejeitar em lote se aparecer "task allocation" no mesmo título).

## Passo 2 — Candidatos a ACEITAR (revisar manualmente, não rejeitar em lote)

Da lista já vista, estes têm potencial real de entrar como estudo primário:

- Multi-Robot Motion and Task Planning in Automotive Production Using Controller-based Safe Reinforcement Learning (Wete et al., 2024)
- Task Allocation Optimization for Warehouse Autonomous Mobile Robot (Hu, Fujimura, Feng, 2024)
- Heterogeneous Multi-Robot Reinforcement Learning (Bettini, Shankar, Prorok, 2023)
- Distributed Task Allocation in Network of Agents Based on Ant Colony Foraging Behavior (Minarolli, 2023)
- Task Allocation in Multi-Agent Systems with Grammar-Based Evolution (Samarasinghe et al., 2021)
- The Holy Grail of Multi-Robot Planning: Learning to Generate Online-Scalable Solutions from Offline-Optimal Experts (Prorok et al., 2022)
- Negotiated Path Planning for Non-Cooperative Multi-Robot Systems (Gautier et al., 2022)
- Multiagent Task Allocation and Planning with Multi-Objective Requirements (Robinson, Su, Zhang, 2021)
- Extending Behavior Trees with Market-Based Task Allocation in Dynamic Environments (Wang, Shi, Yi, 2021)
- Using evolutionary game theory to understand scalability in task allocation (Rizk et al., 2022) — checar se usa RL ou só teoria dos jogos evolutiva (pode não passar no critério de RL)
- Reinforcement learning in autonomous multi-vehicle systems: A structured review (Merkle et al., 2024) — é review, mas de veículos autônomos, avaliar se conta como survey excluído ou como fonte de snowballing
- Attention for the Allocation of Tasks in Multi-Agent Pickup and Delivery (Fenoy et al., 2024) — já marcado "Duplicated" no Parsifal, então já tratado

## Passo 3 — Excluir explicitamente (survey, não estudo primário, mas útil para citar no texto)

- A Systematic Literature Review on Multi-Robot Task Allocation (Athira, Divya Udayan, Subramaniam, 2024) — SLR publicada, citar no Related Work mas rejeitar como estudo primário

## Nota sobre a causa do ruído

A busca no ACM Digital Library retornou muitos "container records" (proceedings inteiros) e resultados de domínios adjacentes (crowdsourcing, edge/cloud computing, UAV networking sem RL de alocação) porque a busca por "Abstract" no ACM indexa também resumos de coleções completas de conferência, não só papers individuais. Isso é uma limitação conhecida da interface do ACM DL para buscas amplas — não é erro da query em si.
