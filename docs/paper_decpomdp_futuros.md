# Trabalhos Futuros — Extensão para Dec-POMDP (MARL)

## Parágrafo pronto para o paper (seção 6 ou 7)

A arquitetura centralizada adotada neste trabalho — um único agente PPO que decide qual robô atende cada demanda — representa uma formulação single-agent do problema MRTA. Uma extensão natural é reformulá-lo como um **Processo de Decisão de Markov Parcialmente Observável Descentralizado (Dec-POMDP)** [Oliehoek & Amato, 2016], no qual cada robô é um agente independente com observação local, tomando decisões de forma autônoma e coordenada. Nessa formulação, a alocação emergeria de um protocolo de leilão distribuído ou de políticas treinadas com a paradigma *Centralized Training with Decentralized Execution* (CTDE) [Lowe et al., 2017], eliminando o gargalo do alocador central. O presente trabalho preparou terreno para essa extensão: a observação já é fatorada por robô em `obs_encoding`, a política é uma função pura parametrizada por N e o oráculo clarividente fornece o mesmo teto de optimalidade para avaliar o regret de políticas descentralizadas. A validação experimental com o gêmeo digital YOLO permanece aplicável sem alterações, pois as posições observadas continuam sendo a entrada de cada agente local.


## Referências sugeridas

- Oliehoek, F. A., & Amato, C. (2016). *A Concise Introduction to Decentralized POMDPs*. Springer.
- Lowe, R. et al. (2017). Multi-agent actor-critic for mixed cooperative-competitive environments. *NeurIPS*.
- Dorri, A. et al. (2018). Multi-agent systems: A survey. *IEEE Access*.
