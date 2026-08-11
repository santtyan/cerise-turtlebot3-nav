# Submissão LARS 2026 — Arquivos Finais (05/08/2026)

## Qual arquivo enviar

**`Leite_Cardoso_LARS2026_CausalDiagnosis_MRTA.pdf`** ← este vai ao EasyChair.

O arquivo `..._PTBR.pdf` é apenas espelho de referência pessoal. O LARS exige submissão
em inglês.

## Fontes para o Overleaf

Em `../OVERLEAF_2026-08-05/`:
- `Leite_Cardoso_LARS2026_source_EN.zip` (ou a pasta `en/` com os arquivos soltos)
- O arquivo principal chama-se `main.tex`, detectado automaticamente pelo Overleaf.

## Link de submissão

https://easychair.org/my/conference?conf=lars2026

Usar a opção de **atualizar a submissão existente** (o paper já foi submetido antes como
`LARS_2026_paper_51.pdf`), não criar submissão nova.

## Dados para o formulário

**Título:**
Causal Diagnosis of Reinforcement Learning Performance in Vision-Based Multi-Robot Task
Allocation with a Digital Twin

**Autores:**
- Yan Santos Leite — santosleiteyan@gmail.com — Universidade Federal de Goiás (UFG)
- Alisson Assis Cardoso — alsnac@ufg.br — Universidade Federal de Goiás (UFG)

**Keywords:**
multi-robot task allocation, deep reinforcement learning, proximal policy optimization,
action masking, digital twin, YOLOv8, ROS 2, reproducibility, negative results

**Tópicos do LARS aplicáveis (marcar no formulário):**
- Multi-Robot and Multi-Agents, Cooperation and Collaboration
- Multi-robot systems
- Robot planning, reasoning, communication, adaptation and learning
- Vision in robotics and automation

## Conformidade verificada

- Formato: template IEEE de conferência (`\documentclass[conference]{IEEEtran}`)
- Páginas: 5 (limite LARS: 4 a 6)
- Fontes: todas Type 1 e embutidas (requisito IEEE Xplore)
- Papel: Letter (612x792 pt)
- Referências: 18, nenhuma órfã
- Compilação: sem erros nem referências indefinidas

## O que mudou nesta versão (vs. a submissão anterior de 15/07)

1. **Tabela II corrigida** — os valores anteriores não vinham de nenhuma execução real dos
   modelos atuais. Substituídos pelos valores reproduzíveis de
   `scripts/eval_policy.py --ablation --inter-arrival 12 --episodes 1000`.
2. **Descoberta relevante:** com os dados reais, o PPO tem *melhor* balanceamento de carga
   (0.135) que o nearest-free (0.149). O texto foi ajustado — antes afirmava que o baseline
   vencia em todas as quatro métricas.
3. Razão p95/média corrigida de "1.56–1.59" para o valor real "1.47–1.56".
4. Taxa de ação inválida corrigida de 4.0% para 4.2% (valor real).
5. Related Work expandido com 5 novas referências verificadas contra os PDFs originais e
   reorganizado em 3 parágrafos temáticos.
6. Contribuições em lista, hipótese enunciada explicitamente, seção de Limitações adicionada.
7. Removido "our advisor corroborated this" (apelo à autoridade; o orientador é coautor).
8. Conclusão reescrita com síntese em vez de repetir o resumo.

## Prazo

05/08/2026, meia-noite no fuso de Bogotá (sede do evento).
