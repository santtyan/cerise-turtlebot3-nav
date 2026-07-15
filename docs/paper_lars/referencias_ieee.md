# Referências bibliográficas (formato IEEE) — paper LARS 2026

Já usadas na bibliografia do `.tex` (PT-BR e inglês), formatadas em estilo IEEE. Copie diretamente para onde precisar (ex: EasyChair, outro documento).

```
[1] B. P. Gerkey and M. J. Matarić, "A formal analysis and taxonomy of task allocation in multi-robot systems," Int. J. Robot. Res., vol. 23, no. 9, pp. 939–954, 2004.

[2] M. B. Dias, R. Zlot, N. Kalra, and A. Stentz, "Market-based multirobot coordination: A survey and analysis," Proc. IEEE, vol. 94, no. 7, pp. 1257–1270, 2006.

[3] R. Lowe, Y. Wu, A. Tamar, J. Harb, P. Abbeel, and I. Mordatch, "Multi-agent actor-critic for mixed cooperative-competitive environments," in Proc. NeurIPS, 2017.

[4] G. Jocher, A. Chaurasia, and J. Qiu, "Ultralytics YOLO," 2023. [Online]. Available: https://github.com/ultralytics/ultralytics. Accessed: Jul. 15, 2026.

[5] P. Henderson, R. Islam, P. Bachman, J. Pineau, D. Precup, and D. Meger, "Deep reinforcement learning that matters," in Proc. AAAI, 2018.

[6] J. Schulman, F. Wolski, P. Dhariwal, A. Radford, and O. Klimov, "Proximal policy optimization algorithms," 2017, arXiv:1707.06347.

[7] A. Raffin, A. Hill, A. Gleave, A. Kanervisto, M. Ernestus, and N. Dormann, "Stable-Baselines3: Reliable reinforcement learning implementations," J. Mach. Learn. Res., vol. 22, no. 268, pp. 1–8, 2021.

[9] S. Huang and S. Ontañón, "A closer look at invalid action masking in policy gradient algorithms," in Proc. Int. FLAIRS Conf., vol. 35, 2022.

[10] J. Orr and A. Dutta, "Multi-agent deep reinforcement learning for multi-robot applications: A survey," Sensors, vol. 23, no. 7, p. 3625, 2023.

[11] A. Agrawal, S. Hariharan, A. S. Bedi, and D. Manocha, "DC-MRTA: Decentralized multi-robot task allocation and navigation in complex environments," in Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS), 2022, pp. 11711–11718.

[12] A. Pal, A. Chauhan, and M. Baranwal, "Together we rise: Optimizing real-time multi-robot task allocation using coordinated heterogeneous plays," in Proc. Int. Conf. Auton. Agents Multiagent Syst. (AAMAS), 2025.

[13] I. Ahmed, S. Din, G. Jeon, F. Piccialli, and G. Fortino, "Towards collaborative robotics in top view surveillance: A framework for multiple object tracking by detection using deep learning," IEEE/CAA J. Autom. Sinica, vol. 8, no. 7, pp. 1253–1270, 2021.
```

Todas verificadas uma a uma via busca web nesta sessão (autor/venue/ano/DOI conferidos), exceto [1], [2], [3], [5], [6], [7], [8] que já estavam no paper antes desta sessão e não foram reverificadas (são papers muito conhecidos/canônicos na área).

---

# Notas úteis extraídas do Gerkey & Matarić (2004) para o Related Work

O usuário colou o texto completo deste paper (ref. [1] acima) nesta sessão. Pontos aproveitáveis para reforçar a Seção "Trabalhos Relacionados" do artigo do LARS:

## Taxonomia MRTA (útil para contextualizar onde nosso problema se encaixa)
Gerkey & Matarić propõem 3 eixos para classificar problemas de MRTA:
- **ST vs. MT** (single-task vs. multi-task robots): um robô executa uma tarefa por vez, ou várias simultaneamente
- **SR vs. MR** (single-robot vs. multi-robot tasks): uma tarefa precisa de um robô, ou de vários
- **IA vs. TA** (instantaneous vs. time-extended assignment): decisão instantânea sem planejar o futuro, vs. com informação sobre demandas futuras/chegando

**Nosso problema (CERISE) se classifica como ST-SR-IA** na formulação padrão (cada robô faz uma tarefa por vez, cada tarefa precisa de um robô, decisão tomada a cada chegada de demanda) — mas com uma extensão: temos *lookahead* de 2 demandas futuras na observação, o que empurra nosso problema para mais perto do espectro TA (time-extended) sem ser puramente isso. Isso é um ponto de discussão interessante: nosso paper mostra que mesmo com esse lookahead parcial, o RL não supera a heurística gulosa — o que é um resultado mais forte do que se estivéssemos comparando RL num regime puramente IA.

## Optimal Assignment Problem (OAP)
- ST-SR-IA é uma instância do OAP (problema clássico de teoria dos jogos/pesquisa operacional)
- Solução centralizada ótima: método Húngaro (Kuhn 1955), O(mn²)
- Solução distribuída: algoritmos de leilão (Bertsekas 1990)
- **Relevante para nós:** o `nearest-free` que usamos como baseline NÃO é a solução ótima do OAP — é um algoritmo guloso (greedy) mais simples. Gerkey & Matarić mostram que o algoritmo guloso é *2-competitive* para o OAP (na pior das hipóteses, entrega metade da utilidade ótima). Isso dá um enquadramento teórico pro porquê nosso oráculo clarividente (agora removido do paper, mas ainda usado internamente pros cálculos) existia: para saber o quão longe do ótimo greedy/PPO estavam.

## Ponto potencialmente citável no texto
"Gerkey and Matarić~\cite{gerkey2004formal} show that greedy assignment is 2-competitive for the Optimal Assignment Problem in the worst case, providing theoretical grounding for why a greedy heuristic such as nearest-free can be a strong baseline despite its myopia." — pode reforçar a seção de Related Work ou a discussão do porquê o nearest-free é difícil de bater.

## Para a revisão de literatura futura (não-SLR)
Esse paper é a referência fundacional de taxonomia em MRTA — praticamente todo paper de alocação de tarefas multi-robô cita ele. Vale usá-lo como ponto de partida para "snowballing" (seguir quem cita e quem é citado por ele) na revisão de literatura que o usuário vai começar.

## Correção/refinamento (2026-07-15, após leitura do texto completo)

O texto completo do paper (colado pelo usuário) traz uma distinção que a nota original não tinha e que é mais precisa para o nosso caso:

- **2-competitive** é o bound do Greedy algorithm para o **OAP offline geral** (Section 5.1.1, "Variant: iterated assignment", citando Avis 1983) — quando todas as tarefas já estão disponíveis de uma vez.
- **3-competitive** é o bound correto para o problema de **online assignment** (Section 5.1.2, citando Kalyanasundaram & Pruhs 1993) — quando as tarefas chegam uma de cada vez e, uma vez atribuído, o robô não pode ser reatribuído. O texto afirma explicitamente: *"the Greedy algorithm is known to be 3-competitive with respect to the optimal post hoc offline solution. Furthermore, this performance bound is the best possible for any online assignment algorithm (Kalyanasundaram & Pruhs 1993)."*
- O sistema MURDOCH dos próprios autores (Gerkey & Matarić 2002b) é dado como exemplo de algoritmo de online assignment: atribui cada nova tarefa ao robô mais apto disponível no momento — **estruturalmente idêntico ao que nosso `nearest-free` faz** (decisão greedy no momento da chegada da demanda, sem reatribuição).

**Implicação para o nosso paper:** o `nearest-free` do CERISE se encaixa melhor na categoria *online assignment* (tarefas chegam uma a uma, sem replanejamento) do que no OAP offline geral. Portanto, o bound teórico mais correto e mais forte para citar é **3-competitive** (Kalyanasundaram & Pruhs 1993, citado em Gerkey & Matarić 2004, Section 5.1.2), não o 2-competitive do Greedy/OAP offline mencionado na nota anterior. Isso ainda sustenta a mesma conclusão (nearest-free tem garantia teórica de proximidade ao ótimo mesmo sendo míope), mas com o enquadramento tecnicamente correto — vale corrigir se essa afirmação já estiver no texto do paper LARS como "2-competitive".

**Atenção:** Kalyanasundaram & Pruhs 1993 é citado apenas *dentro* do texto de Gerkey & Matarić — não temos o paper original deles, então essa referência deve ser citada como citação indireta (via Gerkey & Matarić), não como leitura direta, a menos que o usuário cole o paper original também.

---

# Notas úteis extraídas do Dias, Zlot, Kalra & Stentz (2006) — ref. [2]

"Market-Based Multirobot Coordination: A Survey and Analysis", Proc. IEEE, vol. 94, no. 7.

## Espectro centralizado vs. distribuído vs. market-based
- Centralizado: ótimo em teoria, mas não escala, ponto único de falha, alta demanda de comunicação
- Distribuído: rápido, robusto, flexível, mas pode ser bem subótimo
- Market-based (leilões): híbrido, tenta pegar o melhor dos dois

## OTPR (One-Task-Per-Robot) — relevante para nós
- Robôs "behave[s] myopically and only consider[s] handling one task at any given time" (texto original: "each robot behaves myopically and only considers handling one task at any given time") — **essa é literalmente a definição formal de "míope"/greedy que usamos para caracterizar o nearest-free** no nosso paper. Bom para citar como fundamentação teórica da nossa caracterização. Se for citar como bloco de citação direta no `.tex`, usar a forma verbatim do original ("behaves... considers", singular, concordando com "each robot").
- Contraponto: "sequencing approaches" (robôs enfileiram/agendam múltiplas tarefas, raciocinando sobre dependências) — mais próximo do que um agente RL com lookahead tenta fazer.

## Garantias teóricas citáveis
- Lagoudakis et al.: bidding por custo marginal dá 2-aproximação para objetivo de soma de custos (mesmo resultado de competitividade 2x que aparece em Gerkey & Matarić 2004 pro greedy) — reforça duplamente que heurísticas gulosas simples têm garantia teórica conhecida de "no máximo 2x pior que o ótimo", o que ajuda a explicar por que nearest-free é um baseline tão difícil de bater na prática.
- **Refinamento (2026-07-15, após leitura do texto completo):** essa 2-aproximação vale especificamente para o objetivo de **soma de custos** (sum-of-costs). O texto original diz explicitamente: *"bidding marginal costs results in a 2-approximation when minimizing a sum-of-costs team objective, while for a makespan objective the approximation ratio scales with the number of robots on the team."* Ou seja, para objetivo de **makespan** (tempo máximo de conclusão — mais parecido com nosso tempo de resposta agregado do que soma de custos), a razão de aproximação **não é constante, escala com N**. Isso é relevante para o nosso paper: se formos citar esse bound como garantia teórica para o nearest-free, precisamos ser precisos sobre qual métrica objetivo estamos comparando (nosso `response_time` está mais próximo de makespan do que de soma de custos) — não dá pra simplesmente dizer "2-aproximação" sem essa ressalva.

## Aprendizado em coordenação market-based (2006) — gap histórico
- O survey de 2006 já apontava que "the application of learning techniques in market-based coordination is currently at a very early stage" e que os autores não tinham conhecimento de nenhuma técnica de aprendizado implementada em time físico de robôs coordenado via mercado.
- **Ponto de discussão possível**: quase 20 anos depois, nosso paper testa diretamente RL (PPO) como substituto de alocação baseada em regras/mercado, e mostra que essa promessa antiga ainda não se realiza trivialmente — é consistente com o resultado negativo do nosso paper. Pode ser uma frase interessante pra Introdução ou Trabalhos Relacionados: "o gap entre aprendizado e coordenação multi-robô, identificado há quase duas décadas, permanece parcialmente aberto".

---

# Notas úteis extraídas do Lowe, Wu, Tamar, Harb, Abbeel & Mordatch (2017) — ref. [3]

"Multi-Agent Actor-Critic for Mixed Cooperative-Competitive Environments" (MADDPG), NeurIPS 2017.

## Por que é citado no nosso paper
Já citamos esse paper duas vezes: (1) Related Work, pra mencionar CTDE (Centralized Training with Decentralized Execution) como paradigma de formulações multiagente; (2) Trabalhos Futuros, como base teórica pra uma eventual extensão Dec-POMDP do nosso alocador. Vale entender melhor o que o paper realmente mostra pra citar com mais precisão.

## O que o MADDPG resolve
O paper trata Q-learning e policy gradient como falhando por **dois mecanismos distintos**, não a mesma causa (correção de precisão feita em 2026-07-15, revisão do texto completo):
- **Q-learning independente**: falha por **não-estacionariedade** — cada agente vê o ambiente como não-estacionário porque as políticas dos outros mudam durante o treino, violando a suposição de Markov e impedindo o uso direto de experience replay ("each agent's policy is changing as training progresses, and the environment becomes non-stationary from the perspective of any individual agent").
- **Policy gradient independente**: falha por **alta variância**, que piora com o número de agentes ("policy gradient... usually exhibit very high variance when coordination of multiple agents is required"). A **Proposition 1** do paper formaliza especificamente esse segundo mecanismo (variância), não o de não-estacionariedade: num cenário simples com N agentes de ação binária, a probabilidade do gradiente estimado apontar na direção correta cai proporcionalmente a (0.5)^N.
- Solução proposta: **crítico centralizado, ator descentralizado** (CTDE). Durante o treino, o crítico de cada agente vê as ações de todos; na execução, cada agente só usa sua observação local. Isso resolve a não-estacionariedade do ponto de vista do Q-learning subjacente ao crítico, sem exigir comunicação em tempo de execução.

## Relevância direta para o nosso trabalho
- **Nosso agente é single-agent centralizado** (um único PPO decide pra todos os N=3 robôs) — não sofre do problema de não-estacionariedade que o MADDPG resolve, porque não há múltiplas políticas mudando simultaneamente. Isso é uma diferença importante a explicitar: nosso resultado negativo NÃO é causado pelo problema clássico de MARL não-estacionário — é um problema de restrição suave vs. rígida, dentro de um único agente centralizado.
- Se a extensão Dec-POMDP dos Trabalhos Futuros for adiante (cada robô como agente independente), o MADDPG vira diretamente aplicável: dá pra usar o mesmo esquema de crítico centralizado/ator descentralizado, com o oráculo clarividente (ainda usado internamente, mesmo removido da narrativa do paper atual) servindo de teto de otimalidade também pro caso descentralizado — como já mencionamos na conclusão.
- O MADDPG também usa **ensemble de políticas** para robustez contra adversários que mudam de estratégia — não é diretamente aplicável ao nosso caso cooperativo/único-agente, mas pode ser nota de rodapé se a extensão futura incluir robôs adversariais ou heterogêneos.

## Trecho potencialmente citável (corrigido 2026-07-15 para não conflar variância com não-estacionariedade)
"Lowe et al.~\cite{lowe2017multi} show that independent multi-agent policy gradient suffers from variance that grows with the number of agents, with the probability of a correct gradient step decreasing as $(0.5)^N$ (Proposition 1); our single-agent centralized formulation avoids both this failure mode and the related non-stationarity issue affecting independent Q-learning, isolating the soft-constraint violation as the object of study rather than confounding it with multi-agent training instability."

---

# Notas extraídas do Henderson, Islam, Bachman, Pineau, Precup & Meger (2018) — ref. [5]

"Deep Reinforcement Learning that Matters", AAAI 2018 (arXiv:1709.06560, submetido em 2017-09-19, versão final v3 em 2019-01-30). Texto completo colado pelo usuário em 2026-07-15 (substitui a nota anterior baseada só no abstract).

## O que o paper faz
Estudo experimental (não teórico) sobre reprodutibilidade em deep RL, focado em métodos de policy gradient para controle contínuo (TRPO, DDPG, PPO, ACKTR) em ambientes MuJoCo do OpenAI Gym (Hopper-v1, HalfCheetah-v1, Walker2d-v1, Swimmer-v1). Investigam 5 fontes de variância: hiperparâmetros/arquitetura de rede, escala de recompensa, seeds aleatórias e número de trials, propriedades do ambiente, e diferenças de codebase.

## Achado mais relevante para o CERISE: efeito de seed única (Random Seeds and Trials)
- Os autores rodam **10 trials** do TRPO no HalfCheetah-v1 com a **mesma configuração de hiperparâmetros**, variando só a seed aleatória, depois dividem em dois grupos de 5 e comparam as médias.
- Resultado citável direto: **"The average 2-sample t-test across entire training distribution resulted in t = −9.0916, p = 0.0016."** — ou seja, dois grupos de 5 seeds do *mesmo* algoritmo com os *mesmos* hiperparâmetros produzem distribuições de treino estatisticamente diferentes.
- Conclusão dos autores: "even averaging several learning results together across totally different random seeds can lead to the reporting of misleading results" — e citam que é comum na literatura usar apenas top-N trials selecionados ou médias de poucos trials (N<5), o que pode ser enganoso.
- **Relevância direta:** nosso treino PPO usa seed=42 única (limitação já registrada em memória/trabalhos futuros). Esse resultado do Henderson et al. é evidência empírica direta e citável de que o efeito de seed sozinho pode ser responsável por uma fração não-trivial da diferença observada entre execuções — reforça (e credibiliza com uma fonte concreta) a limitação de "treino com seed única" que já está no nosso paper como trabalho futuro. Se formos expandir a seção de limitações/trabalhos futuros, vale citar esse t/p diretamente como motivação para treino multi-seed.

## Outros achados úteis (mais periféricos para nós)
- Arquitetura de rede e função de ativação têm efeito significativo e **inconsistente entre algoritmos/ambientes** — não há configuração universalmente melhor.
- Escala de recompensa (reward rescaling) pode ter efeito grande, especialmente em DDPG.
- Diferenças de codebase (mesmo implementando "o mesmo" algoritmo) produzem resultados bem diferentes — reforça a importância de publicar código, o que já fazemos.
- Recomendação central dos autores para a comunidade: reportar todos os hiperparâmetros, detalhes de implementação, e usar testes de significância (t-test, Kolmogorov-Smirnov, bootstrap com intervalo de confiança de 95%) em vez de só comparar retornos médios — é exatamente o que já fazemos com Wilcoxon + delta de Cliff (mesmo espírito metodológico, teste não-paramétrico ao invés do t-test paramétrico que eles usam, mais apropriado para os nossos dados).

## Relevância direta para o nosso trabalho (mantido/reforçado da nota anterior)
Continua sendo o paper mais relevante para a **metodologia estatística** do nosso artigo, não para o problema de MRTA. Já citado (`\cite{henderson2018deep}`). O achado do efeito de seed (t=-9.09, p=0.0016) é o ponto novo mais forte para citar se expandirmos a discussão de limitações.

---

# Notas extraídas do Schulman, Wolski, Dhariwal, Radford & Klimov (2017) — ref. [6]

"Proximal Policy Optimization Algorithms" (arXiv:1707.06347v2, publicado 2017-07-20, revisado 2017-08-28). Texto completo colado pelo usuário em 2026-07-15. Este é o paper original do **algoritmo central usado no CERISE** (PPO, via Stable-Baselines3).

## O que o paper propõe
- Objetivo substituto (surrogate) com **clipping da razão de probabilidade** $r_t(\theta) = \pi_\theta(a_t|s_t)/\pi_{\theta_{old}}(a_t|s_t)$: $L^{CLIP}(\theta) = \hat{E}_t[\min(r_t(\theta)\hat{A}_t, \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon)\hat{A}_t)]$, com $\epsilon=0.2$ como valor padrão usado na maioria dos experimentos.
- Alternativa: penalidade adaptativa de KL-divergence (Seção 4) — os próprios autores relatam que essa variante teve desempenho pior que o clipping na prática, mas incluíram como baseline.
- Permite múltiplas épocas de atualização minibatch sobre os mesmos dados coletados (diferente do policy gradient "vanilla", que faz um update por amostra) — ganho de eficiência de amostra sem a complexidade do TRPO (que precisa de gradiente conjugado).

## Resultados relatados
- Nos 7 ambientes de controle contínuo MuJoCo (HalfCheetah, Hopper, InvertedDoublePendulum, InvertedPendulum, Reacher, Swimmer, Walker2d), a versão com clipping $\epsilon=0.2$ obteve o melhor score médio normalizado (0.82) entre as variantes testadas (sem clipping/penalidade: -0.39; KL adaptativo: 0.68–0.74; KL fixo: 0.62–0.72).
- PPO superou TRPO, CEM, vanilla PG e A2C na maioria dos ambientes de controle contínuo, e teve desempenho comparável ou melhor que A2C/ACER em jogos Atari (venceu em 30 de 49 jogos por uma métrica, 19 de 49 por outra).

## Ponto metodológico relevante (conexão com Henderson et al. 2018 — ref. [5])
No experimento de comparação de objetivos substitutos (Seção 6.1), os próprios autores usam **apenas 3 seeds aleatórias por ambiente** ("Each algorithm was run on all 7 environments, with 3 random seeds on each"). Isso é interessante para contextualizar: mesmo o paper que introduziu o PPO — algoritmo que usamos — não usa um número grande de seeds em sua validação original, o que é exatamente o tipo de prática que Henderson et al. (2018) criticam como potencialmente insuficiente para conclusões estatisticamente robustas (ver nota da ref. [5] acima sobre o efeito de seed única, t=-9.09, p=0.0016). Não é uma crítica ao PPO em si (o algoritmo é amplamente validado depois por outros trabalhos), mas é um ponto honesto a reconhecer se formos discutir nossa própria limitação de seed única: mesmo a literatura fundacional de PPO tem validação com poucos seeds.

## Relevância direta para o nosso trabalho
Já citado (`\cite{schulman2017proximal}`) como a referência do algoritmo PPO em si — não precisa de mudança na citação. Não há necessidade de expandir o Related Work com esse paper além do que já está, já que ele é citado principalmente como fundamentação do método, não como trabalho comparável em MRTA.

---

# Notas extraídas do Raffin, Hill, Gleave, Kanervisto, Ernestus & Dormann (2021) — ref. [7]

"Stable-Baselines3: Reliable Reinforcement Learning Implementations", JMLR 22 (2021), pp. 1–8. Texto completo colado pelo usuário em 2026-07-15 (ResearchGate + PDF). Esta é a **biblioteca de software usada para treinar o PPO no CERISE**.

**Nota sobre a referência IEEE já existente [7]:** o texto colado não mostra o número de artigo JMLR (só "22 (2021) 1-8" no cabeçalho da página), então não consigo re-verificar o "no. 268" a partir deste documento — mantenho a referência como estava (não é uma alucinação nova, só não pude confirmar esse dado específico com este texto).

## O que o paper descreve
- SB3 é uma reescrita completa em PyTorch da Stable-Baselines2 (que era um fork do OpenAI Baselines), com 95% de cobertura de testes automatizados.
- Implementa 7 algoritmos on/off-policy: A2C, **PPO**, DDPG, SAC, TD3, HER, DQN.
- Foco declarado: simplicidade e confiabilidade (não escalabilidade/paralelismo distribuído, ao contrário de bibliotecas como RLlib).

## Conexão direta com nossas outras referências (cadeia de citações coerente)
O próprio paper do SB3 cita **Henderson et al. 2018** (nossa ref [5]) duas vezes, de forma quase idêntica ao motivo pelo qual nós o citamos:
1. Na introdução: "results are often difficult to reproduce (Henderson et al., 2018)."
2. Na seção de features: "We follow best practices for training and evaluation (Henderson et al., 2018), such as evaluating in a separate environment, using deterministic evaluation where required (SAC) and storing all hyperparameters necessary to replicate the experiment."

Isso fecha uma cadeia de citação coerente já presente (mesmo que implicitamente) no nosso paper: **PPO (Schulman et al., ref [6]) → implementado via SB3 (ref [7]) → que segue as boas práticas de reprodutibilidade de Henderson et al. (ref [5])**. Vale mencionar essa cadeia explicitamente se expandirmos a seção de metodologia — reforça que a escolha de ferramentas (SB3) não foi arbitrária, mas alinhada com práticas de reprodutibilidade reconhecidas na área.

## Referência potencialmente nova (ainda não em nossa lista) mencionada nas referências do SB3
O SB3 cita: **L. Engstrom, A. Ilyas, S. Santurkar, D. Tsipras, F. Janoos, L. Rudolph, and A. Madry, "Implementation matters in deep RL: A case study on PPO and TRPO," in Proc. Int. Conf. Learning Representations (ICLR), 2020.** — especificamente sobre como detalhes de implementação (não o algoritmo em si) afetam desempenho de PPO/TRPO, o que é diretamente relevante para uma discussão de "por que os resultados do PPO dependem tanto da implementação e não só da formulação teórica" no nosso paper. **Atenção:** só tenho a entrada bibliográfica (autores, título, venue, ano) retirada da lista de referências do SB3 — não li o conteúdo desse paper, então não posso extrair notas sobre o que ele afirma até que o texto completo seja colado. Não usar essa referência no `.tex` além da citação bibliográfica básica sem antes ler o paper.

## Relevância direta para o nosso trabalho
Já citado (`\cite{stable-baselines3}`) corretamente como a ferramenta de treino do PPO. Não precisa de mudança. O achado da cadeia de citações com Henderson et al. é o ponto mais útil aqui, caso a metodologia seja expandida.

---

# Notas extraídas do Huang & Ontañón (2022) — ref. [9]

"A Closer Look at Invalid Action Masking in Policy Gradient Algorithms", FLAIRS Vol. 35, 2022 (arXiv:2006.14171, publicado 2020-06-25, revisado até v3 em 2022-05-31). Texto completo colado pelo usuário em 2026-07-15. **Este é o paper que fundamenta teoricamente o nosso próprio experimento de action masking** — o mais diretamente relevante da lista até agora.

## O que o paper prova e mostra
1. **Proposition 1 (formal)**: action masking (substituir logits de ações inválidas por um número muito negativo antes do softmax) produz um gradiente de política **válido** — é uma função diferenciável dependente do estado aplicada à política, satisfazendo as premissas do teorema do gradiente de política (Sutton et al. 2000). Ou seja, masking não é um "hack" heurístico, tem justificativa formal.
2. **Empiricamente, em μRTS (jogo de estratégia em tempo real)**, comparando 4 estratégias — invalid action penalty (recompensa negativa), invalid action masking, naive masking (masking só na amostragem, gradiente sem zerar), e masking removido na avaliação:
   - **Action masking escala bem**: tempo até convergência (`tsolve`) fica em torno de 12% do treino, **consistente** entre mapas de 4×4 até 24×24 (espaço de ações inválidas crescendo drasticamente).
   - **Invalid action penalty NÃO escala**: conforme o espaço de ações inválidas cresce, o agente treinado com penalidade demora muito mais (ou falha) para encontrar até a primeira recompensa; o hiperparâmetro de penalidade é difícil de ajustar (penalidade muito negativa desencoraja exploração).
   - **Naive masking tem melhor retorno médio, mas diverge em KL**: a divergência KL entre a política antes/depois da atualização "explode" comparado às outras estratégias — instável, mais sensível ao tamanho do mapa.
   - **Masking removido na avaliação ainda funciona razoavelmente**: o agente treinado com masking, quando avaliado sem masking, ainda produz comportamento útil (embora degradando com o tamanho do mapa) — evidência de que o masking ensina comportamento genuinamente válido, não é só uma muleta.
3. Usam PPO (Schulman et al. 2017, nossa ref [6]) via a implementação do OpenAI Baselines, e citam **Engstrom et al. 2020** ("Implementation matters in deep RL: A case study on PPO and TRPO") para justificar detalhes de otimização em nível de código — **segunda vez que esse paper aparece** (já tinha sido mencionado na lista de referências do SB3, ref [7]). Reforça que vale a pena buscar esse paper especificamente se formos aprofundar a metodologia.

## Diferença importante com o nosso resultado (ponto central para o Related Work/Discussão)
O achado de Huang & Ontañón é que, **no domínio deles** (μRTS, RTS game), action masking **escala bem e resolve o problema de ações inválidas de forma eficaz conforme o espaço de ações inválidas cresce** — ou seja, nesse domínio, eliminar ações inválidas estruturalmente parece ser suficiente para manter bom desempenho.

**O nosso resultado é diferente e mais matizado**: aplicamos exatamente essa mesma técnica (masking estrutural, elimina invalid_rate de 4,0% para 0,0%), e ela funciona exatamente como a teoria prevê — remove as ações inválidas por construção. Mas, diferente de Huang & Ontañón, isso **não fechou a lacuna de desempenho** entre PPO e nearest-free (só ~1 ponto percentual dos ~12pp de gap). Isso sugere que, no nosso domínio de alocação multi-robô, a violação de restrição suave (que o masking elimina) **não era a causa dominante do gap de desempenho** — diferente do domínio de RTS onde parece ser. Essa é uma diferenciação real e citável: o masking funciona (no sentido teórico do Huang & Ontañón — é um gradiente de política válido, resolve o problema estrutural), mas o "problema estrutural" não era o gargalo principal no nosso caso. Vale explicitar essa nuance se expandirmos a discussão — reforça que nosso "resultado negativo" é sobre a causa raiz, não sobre a validade teórica do masking em si.

## Referência potencialmente nova, ainda não confirmada em nosso texto
Confirma novamente a existência de **Engstrom, Ilyas, Santurkar, Tsipras, Janoos, Rudolph & Madry, "Implementation matters in deep RL: A case study on PPO and TRPO," ICLR 2020** — agora citado por duas fontes independentes (SB3 e Huang & Ontañón) como referência séria sobre como detalhes de implementação de PPO afetam desempenho. Ainda não li o conteúdo desse paper — só tenho a entrada bibliográfica repetida em duas fontes diferentes, o que aumenta a confiança de que a citação em si (autores/título/venue/ano) está correta, mas não substitui ler o texto antes de extrair alegações de conteúdo.

## Relevância direta para o nosso trabalho
Já citado (`\cite{huang2022masking}`, linha 179 do `.tex` de submissão) como fundamentação do action masking — a frase do Related Work já é precisa: "who show that action masking produces a valid policy gradient and outperforms soft penalization of invalid actions" bate exatamente com Proposition 1 e os resultados empíricos do paper.

---

# Notas extraídas do Orr & Dutta (2023) — ref. [10]

"Multi-Agent Deep Reinforcement Learning for Multi-Robot Applications: A Survey", Sensors 23(7), 3625, 2023. Texto colado pelo usuário em 2026-07-15 (MDPI, via ResearchGate) — **truncado**: o texto disponível vai até o meio da Seção 3.2 (Path Planning and Navigation) e não inclui a Seção 4 ("Challenges and Discussion") nem a Conclusão.

## 🚨 CONFIRMADO: alegação usada no nosso paper é INCORRETA (2026-07-15, Seção 4 e Conclusão agora lidas)
Nosso `.tex` (linha 49) contém: *"Orr and Dutta~\cite{orr2023survey} survey multi-agent RL in robotics and note that most works evaluate only against weak baselines without a rigorous statistical comparison, a gap we address with paired significance testing."*

**Essa alegação é factualmente incorreta.** Com a Seção 4 ("Challenges and Discussion") e a Conclusão agora completas, os desafios que o paper realmente identifica são:
1. **Escalabilidade**: "Most of the papers reviewed in this article do not scale beyond tens of robots."
2. **Reprodutibilidade — mas por falta de simuladores/benchmarks padronizados**, não por falta de rigor estatístico: *"most papers employing MADRL use their own (simulation) environments for their robots, which makes it extremely difficult for others to reproduce the results. As a community, we need to come up with an accepted set of benchmarks and/or simulators..."*
3. **Gap sim-to-real**: a maioria dos experimentos é só em simulação, corroborando achado de Liang et al. [257].
4. Discussão de ferramentas (VMAS, MultiRoboLearn, MARLlib), escassez de dados de treino, e dois gaps de aplicação (robótica modular auto-reconfigurável; aprendizado simultâneo de manipulação+movimento).

**Não há, em nenhum lugar do texto (abstract, introdução, Seção 4, conclusão), qualquer menção a "baselines fracos" ou "falta de comparação estatística rigorosa".** Essa frase específica não existe no paper — é uma citação incorreta que precisa ser corrigida ou removida do nosso `.tex` antes de qualquer revisão/camera-ready. A citação mais próxima e correta seria sobre **falta de simuladores/benchmarks padronizados** (ponto 2 acima), que é um argumento diferente do que está escrito atualmente.

**Ação recomendada:** reescrever a frase da linha 49 para refletir o argumento real (reprodutibilidade via ambientes de simulação não padronizados), ou trocar a citação por outra fonte que de fato discuta falta de rigor estatístico em baselines de RL — por exemplo, Henderson et al. 2018 (nossa própria ref [5]) já cobre esse ponto especificamente e com precisão.

## O que o texto disponível confirma
- É a primeira survey de MADRL (multi-agent deep RL) aplicado a sistemas multi-robô — a survey anterior mais próxima (Yang & Gu) é de 2004 e cobre só RL clássico (DRL não existia ainda).
- Classifica aplicações de MRS em 8 categorias: (1) coverage, (2) path planning, (3) swarm behavior, (4) **task allocation**, (5) information collection, (6) pursuit-evasion, (7) object transportation, (8) construction.
- Cobre fundamentos técnicos (MDP, Q-learning, Multi-Agent Q-learning/Nash equilibrium, DQN/DDQN, políticas via policy gradient — DDPG, PPO, TRPO, A3C — e extensões multiagente: MADDPG, MAPPO, VDN/QMIX/factorização de função de valor).
- Na parte de Path Planning (a que temos texto completo), cita dezenas de trabalhos usando PPO/DDPG/DQN para navegação e evasão de colisão multi-robô — nenhum deles trata especificamente do problema de **alocação de tarefas** (nosso foco); a seção 3.4 (Task Allocation), que seria a mais diretamente comparável ao nosso trabalho, não está no texto colado.

## Para revisão futura
✅ Resolvido em 2026-07-15 — Seção 4 e Conclusão foram coladas e a citação foi corrigida no `.tex` (ver acima). Ainda não temos a Seção 3.4 (Task Allocation) especificamente, mas não é mais bloqueante.

---

# Notas extraídas do Agrawal, Hariharan, Bedi & Manocha (2022) — ref. [11]

"DC-MRTA: Decentralized Multi-Robot Task Allocation and Navigation in Complex Environments", IROS 2022 (arXiv:2209.02865). Texto completo colado pelo usuário em 2026-07-15. **É o trabalho mais diretamente comparável ao CERISE de toda a lista** — RL para alocação de tarefas em ambiente de armazém, comparado contra baseline guloso.

## Confirmação da citação já usada
Nosso `.tex` (linha 49): "Recent RL-based warehouse MRTA includes decentralized MDP approaches~\cite{agrawal2022dcmrta}..." — **correto**. O paper formula alocação de tarefas como MDP (estado = posições/tarefas, ação = escolher tarefa da fila, recompensa = -TTD/tempo de viagem), resolvido com RL, e é decentralizado (a navegação de baixo nível usa ORCA, decentralizada).

## O que o paper faz e encontra
- Abordagem em duas camadas: **RL para alocação de tarefas** (nível alto) + **ORCA para navegação decentralizada sem colisão** (nível baixo), com recompensa da camada de alocação definida como o feedback (TTD — Total Travel Delay) da camada de navegação — ou seja, as duas camadas são acopladas via recompensa.
- Compara contra dois baselines gulosos: **MPDM** (minimum pickup distance minimization — estruturalmente idêntico ao nosso `nearest-free`) e **RBTS** (regret-based task selection — auction/regret, mais sofisticado que MPDM).
- **Resultado: RL supera ambos os baselines gulosos**, com melhoria de até 14% em tempo de conclusão de tarefas (TTD) e até 40% de redução em colisões inter-agente (Tabelas I–IV, testado em 5 layouts de armazém, 10 a 1000 robôs).
- Importante: a melhoria do RL sobre MPDM/RBTS **diminui conforme o número de robôs cresce** (ex: 10 robôs → melhorias de até 7%, 100 robôs → melhorias menores, ~1-6%) — atribuído pelos autores a mais congestionamento com mais agentes, fazendo o esquema de navegação decentralizado desviar do comportamento ensinado no treino.

## 🎯 Achado mais relevante para o CERISE — possível explicação arquitetural para o nosso resultado negativo
Os autores relatam explicitamente uma limitação de arquitetura que tentaram e abandonaram: *"One could try a simple NN architecture where we pass all the available robot information (locations, availability) to the network and train it. But unfortunately, such simple designs doesn't result in convergence and also not scalable with respect to number of robots and tasks. Hence, we carefully designed the architecture using attention based mechanisms to make sure that policy network supports variable number of robots and tasks."*

Ou seja: uma rede neural simples (MLP padrão) **não convergiu nem escalou** para o problema de alocação de tarefas multi-robô no domínio deles — eles precisaram de uma arquitetura baseada em **atenção** (Vaswani et al., "Attention is all you need") para conseguir bons resultados.

**Isso é potencialmente muito relevante para explicar o nosso resultado negativo.** Se o nosso PPO usa uma política MLP padrão (via `MlpPolicy` do Stable-Baselines3, presumivelmente — checar no código), isso é exatamente o tipo de arquitetura simples que o DC-MRTA relata como insuficiente para esse problema. Isso sugere uma **hipótese alternativa/complementar** à do espaço de ação pequeno já registrada como trabalho futuro: talvez não seja só o espaço de ação pequeno, mas a **arquitetura de política (MLP simples vs. atenção)** que limita o desempenho do PPO no nosso caso. Vale conferir no código do CERISE (`allocation_env.py`) qual arquitetura de rede está sendo usada, e considerar mencionar isso como uma limitação/hipótese adicional nos Trabalhos Futuros, com essa citação como apoio direto.

## Relevância direta para o nosso trabalho
Já citado corretamente. **Recomendação forte**: se houver qualquer expansão futura (camera-ready, workshop), esse é o paper mais importante para uma frase de contraste direto — "diferente do DC-MRTA, que demonstra que RL supera baselines gulosos em MRTA de armazém mas requer arquitetura de atenção para escalar, nosso PPO usa uma política MLP padrão, o que pode explicar parte do gap observado" (verificar arquitetura real usada antes de escrever isso).

---

# Notas extraídas do Pal, Chauhan & Baranwal (2025) — ref. [12]

"Together We Rise: Optimizing Real-Time Multi-Robot Task Allocation using Coordinated Heterogeneous Plays" (MRTAgent), AAMAS 2025. Texto completo colado pelo usuário em 2026-07-15.

## Confirmação da citação já usada
Nosso `.tex` (linha 49): "...dual-agent frameworks~\cite{pal2025together}; unlike these, our focus is diagnosing why a learned policy fails to beat a greedy heuristic..." — **correto**. MRTAgent é literalmente dual-agent: um Planner (seleciona tarefa da fila) e um Executor (aloca robô à tarefa selecionada), treinados via self-play (alternando 40 episódios cada), ambos com PPO.

## O que o paper faz e encontra
- **Usa PPO** (igual ao CERISE) para os dois agentes, mas com arquitetura especializada: camadas de embedding separadas por robô/tarefa, seguidas de "gates" sigmoid multiplicativos (`a_i`) e concatenação — não é um MLP simples aplicado direto ao vetor de estado bruto.
- Compara contra dois baselines: **FIFO** (fila simples) e **BFO — Brute-Force Optimal** (avalia exaustivamente todos os pares tarefa-robô na janela de lookahead e escolhe o de menor custo; os próprios autores afirmam que BFO "is an optimal task allocation and assignment approach given the current state of the LA" — ou seja, é um baseline *ótimo* dado o lookahead disponível, mais forte que qualquer heurística gulosa simples).
- **Resultado: MRTAgent (PPO) supera consistentemente até o BFO** (baseline ótimo local) em quase todos os cenários testados (Tabelas 1-5), inclusive sob mudança de distribuição de tarefas, variação no número de robôs (10→30) e no número de tarefas (505→2005), sem retreino.
- Explicação dos autores para superar até um baseline ótimo local: *"MRTAgent exploits the underlying distribution defining task generation to plan for tasks to appear in future despite it having access to the same causal information as the BFO"* — ou seja, o RL aprende padrões estatísticos de chegada de tarefas que o BFO (que só otimiza miopicamente dentro da janela de lookahead) não consegue explorar.

## 🎯 Reforço da hipótese arquitetural (conecta com a nota do DC-MRTA, ref [11])
O related work deste próprio paper cita **RTAW** (Agrawal, Bedi & Manocha, 2023, ICRA — *"An attention inspired reinforcement learning method for multi-robot task allocation in warehouse environments"*), do mesmo grupo do DC-MRTA — o título já diz explicitamente "attention inspired". Combinado com a arquitetura de embeddings/gates do próprio MRTAgent (não um MLP simples), temos agora **dois exemplos independentes na literatura de MRTA com RL** (DC-MRTA/RTAW e MRTAgent) que usam arquiteturas especializadas (atenção ou embeddings por entidade) e batem baselines fortes — nenhum deles usa uma política MLP padrão aplicada ao vetor de estado bruto, que é presumivelmente o que o CERISE usa via SB3.

**Isso fortalece a hipótese, já registrada na nota do DC-MRTA, de que a arquitetura de rede (não só o espaço de ação) pode ser um fator explicativo do nosso resultado negativo.** Vale verificar a arquitetura de política usada no `allocation_env.py`/config do PPO do CERISE antes de escrever qualquer frase sobre isso — mas se for de fato MLP padrão, essa é uma citação forte para uma hipótese adicional em Trabalhos Futuros.

## Referência nova candidata (não processada ainda)
**A. Agrawal, A. S. Bedi, and D. Manocha, "RTAW: An attention inspired reinforcement learning method for multi-robot task allocation in warehouse environments," in Proc. IEEE Int. Conf. Robotics and Automation (ICRA), 2023, pp. 1393–1399.** — só tenho a entrada bibliográfica (extraída da lista de referências do MRTAgent), não o conteúdo. Seria o paper mais direto para confirmar a hipótese arquitetural acima, já que é do mesmo grupo do DC-MRTA e é explicitamente sobre atenção aplicada a esse exato problema. Não usar além da citação básica até o texto ser colado.

## Relevância direta para o nosso trabalho
Já citado corretamente (`\cite{pal2025together}`). Ponto mais forte: junto com DC-MRTA, sugere que a arquitetura de política pode ser tão ou mais importante que o espaço de ação para o desempenho do RL em MRTA — vale investigar antes de finalizar a narrativa de "espaço de ação pequeno" nos Trabalhos Futuros.

---

# Notas extraídas do Ahmed, Din, Jeon, Piccialli & Fortino (2021) — ref. [13]

"Towards Collaborative Robotics in Top View Surveillance: A Framework for Multiple Object Tracking by Detection Using Deep Learning", IEEE/CAA J. Autom. Sinica, vol. 8, no. 7, pp. 1253–1270, 2021. Texto completo colado pelo usuário em 2026-07-15.

## Confirmação da citação já usada
Nosso `.tex` (linha 49): "...YOLO-family detectors~\cite{yolov8} are common for vision-based tracking~\cite{ahmed2021topview}, but prior work evaluates detection quality in isolation, without ablating its effect on a learned allocation policy while holding all else fixed" — **correto**.

**Nota (2026-07-15):** o segundo `\cite` nesse trecho (Wang & Liu, "YOLOv5-PPO") foi removido a pedido do usuário por sinais de baixo rigor — autores fora da área de robótica/CS, linguagem promocional sem sustentação numérica visível ("cleverly combines", "excellent robustness and adaptability"), e ano de publicação incorreto na citação original (2023 no `.tex`, mas a publicação real é de 2024). Referência e `\cite` removidos de `referencias_ieee.md` e dos dois `.tex` (inglês e PT-BR). Este paper avalia exclusivamente qualidade de detecção/rastreamento (True Detection Rate, False Detection Rate, tracking accuracy) — não há nenhuma política aprendida (RL) a jusante; é puramente sobre percepção.

## O que o paper faz e encontra
- Framework de vigilância colaborativa com câmera robótica vista de cima (top view), usando **YOLO** e **SSD** (ambos pré-treinados só em dataset de vista frontal, COCO/PASCAL VOC — sem fine-tuning específico para vista de cima) para detecção, combinados com 6 algoritmos de tracking clássicos (GOTURN, MEDIANFLOW, TLD, KCF, MIL, BOOSTING).
- Resultado: mesmo sem fine-tuning para a perspectiva de cima, os modelos alcançam True Detection Rate de 90–93%, False Detection Rate até 0,6%, e acurácia de tracking de 90–94%.
- Motivação central do paper (Introdução): vista de cima reduz problemas de oclusão em relação à vista frontal — objetos se sobrepõem menos quando vistos de cima.

## Relevância metodológica para o CERISE
Esse é um paralelo direto e favorável ao nosso próprio pipeline: eles usam YOLO **genérico, sem fine-tuning** para vista de cima e ainda assim conseguem ~90%+ de detecção. O CERISE vai além — treina/faz fine-tuning do YOLO especificamente no dataset de vista de cima do ambiente TurtleBot3 (ver [[project_yolo_dataset]], mAP@0.5=0.995), o que é uma abordagem mais robusta que a deles. Vale notar essa diferença se quisermos reforçar a seção de percepção: nosso YOLO fine-tuned supera a robustez que um YOLO genérico não fine-tuned alcança neste paper, mesmo em tarefa mais restrita (detectar robôs, não humanos/veículos genéricos).

## Relevância direta para o nosso trabalho
Já citado corretamente e a alegação no Related Work sobre ele é factualmente precisa — não precisa de nenhuma correção. **Recomendação:** se houver espaço para expandir a Discussão/Related Work na versão futura (camera-ready), esse é o paper mais forte para uma frase explícita de diferenciação: "ao contrário de Huang & Ontañón (2022), que mostram que masking resolve efetivamente o problema de ações inválidas em RTS conforme o espaço cresce, nosso resultado mostra que a mesma técnica, aplicada corretamente (invalid_rate→0%), não fecha a lacuna de desempenho no domínio de alocação multi-robô — evidência de que a causa do nosso resultado negativo é distinta."
