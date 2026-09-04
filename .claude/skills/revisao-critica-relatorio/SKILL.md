---
name: revisao-critica-relatorio
description: Revisão crítica linha-por-linha de relatórios científicos e artigos (rigor de lógico, gramático sênior e revisor de conferência de computação): decisões arbitrárias sem justificativa, afirmações sem fonte, lacunas de reprodutibilidade, perguntas que uma banca faria e o texto não responde, inconsistências internas, falácias lógicas e validade argumentativa, coesão e fluidez narrativa em português, idiossincrasias de texto gerado por IA (travessão, antítese "X, não Y", tríades compulsivas, adjetivação vazia, conectivos clichê, densidade de parágrafo uniforme), e força da tese como um todo (narrativa contexto→lacuna→problema→contribuição na Introdução, diferenciação explícita contra trabalhos relacionados, benchmarks contra o estado da arte real, cobertura das figuras, cenário de uso concreto, palavras-chave para indexação internacional, pontos cegos do autor). Use sempre que o usuário pedir para revisar criticamente, auditar, "achar furos", checar a lógica/raciocínio do texto, deixar o texto mais fluente/fluido/natural, preparar para banca/prêmio, ou perguntar se a tese/argumentação/narrativa está forte, bem justificada ou pronta para submissão. Use também antes de considerar um relatório final ou artigo pronto para entrega.
---

# Revisão crítica de relatório científico

Esta skill audita um documento já escrito (não escreve do zero) com o padrão de uma banca de
prêmio: adversarial, exaustiva, sem elogiar. O objetivo é achar toda pergunta que alguém pode
fazer e o autor não sabe responder, antes que essa pergunta apareça numa arguição de verdade.

## Passo 1 — rodar o detector determinístico primeiro

```bash
python3 .claude/skills/revisao-critica-relatorio/scripts/check_ai_tics.py <arquivo.tex>
```

Reporta em segundos, sem gastar contexto de LLM: contagem de travessões, padrão antitético
"X, não Y" (incluindo a variante "não é apenas X, é Y"), adjetivos sem número ao lado, advérbios
em "-mente" por 1000 palavras, conectivos de abertura clichê ("além disso", "vale ressaltar
que"...), transições vazias que reformulam o que já foi dito, pares de adjetivos coordenados
redundantes ("robusto e eficiente"), uniformidade suspeita de tamanho de parágrafo (coeficiente
de variação), negrito abrindo parágrafo, meta-comentário sobre o próprio processo de correção,
referências bibliográficas órfãs nos dois sentidos (citada no corpo mas sem `\bibitem`, ou
`\bibitem` nunca citado), e se hardware está declarado quando o texto menciona benchmark de
tempo/CPU. É um detector, não veredito: cada ocorrência pode ser legítima, mas toda ocorrência
merece uma segunda olhada.

## Passo 2 — releitura integral linha por linha

O script não substitui leitura. Ler o documento inteiro (não amostrar seções) categorizando
achados em:

1. **Decisões arbitrárias sem justificativa.** Todo número, limiar ou parâmetro precisa de uma
   frase "por quê" a menos de 2-3 linhas de distância. `occ ≥ 65` sem dizer que é o default do
   Nav2, ou `N=1500 trials` sem dizer por que 1500 e não 500 ou 3000, são exatamente o tipo de
   coisa que um avaliador pergunta e o autor esquece de ter respondido no texto.
2. **Afirmações sem fonte.** Qualquer frase sobre "a literatura" ou "o estado da arte" precisa
   apontar citação específica. "A literatura trata, em sua maioria, X" sustentado por 3
   referências é o tipo de generalização que convida à pergunta "revisão sistemática ou
   impressão?".
3. **Lacunas no "como".** Hardware (CPU/RAM: sem isso, todo benchmark de tempo é inválido),
   versões de software, seeds, critério de parada, definição operacional de métricas ("sucesso"
   significa o quê exatamente?). Sem esses detalhes o trabalho não é reproduzível.
4. **Perguntas adversariais.** Para cada seção, perguntar: que pergunta um professor cético faria
   aqui, e o texto responde? Ser especificamente adversarial com o achado mais forte do trabalho
   — é ali que mora o ataque mais provável.
5. **Inconsistências internas.** Números que mudam entre seções, referências cruzadas para
   conteúdo que não existe ou diz o oposto, resumo que afirma algo que o corpo depois retrata.
6. **Coesão e fluidez em português.** Anglicismos sem tradução nem itálico consistente (trials,
   baseline, goal, stack, mock, timeout — escolher tradução OU itálico sistemático, não misturar),
   repetição excessiva da mesma expressão, períodos longos demais para leitura corrida,
   personificação vaga ("os cenários revelaram").

## Passo 2b — padrão de gramático sênior da língua portuguesa

Além de coesão/fluidez (item 6 do Passo 2), aplicar rigor de revisor profissional de português,
frase por frase:

- **Crase**: toda ocorrência de "a" antes de palavra feminina checada por substituição (trocar
  por masculino: "ao"/"a"?) e por regência do verbo/preposição anterior ("referente à",
  "em relação à", "devido à" pedem crase; "a partir de" nunca leva).
- **Regência verbal e nominal**: verbos como "implica" (transitivo direto, sem "em"), "assistir"
  (sentido de ver = "assistir a"), "visar" (objetivo = "visar a"), preposição exigida por
  adjetivos ("propenso a", "favorável a") checada uma a uma.
- **Concordância verbal e nominal em frases longas**: sujeito composto ou intercalado por
  aposto/parentético é o ponto onde concordância mais quebra ("as taxas de sucesso, medidas em
  três densidades, revelam" — verbo concorda com "taxas", não com o substantivo mais próximo).
- **Paralelismo sintático em enumerações**: itens de uma lista ou oração com "e"/"ou" devem ter
  a mesma estrutura gramatical (todos substantivos, todos infinitivos, todos adjetivos) — não
  misturar "implementar X, avaliação de Y e testar Z".
- **Colocação pronominal**: próclise após palavra atrativa (não, que, quando, quem), ênclise em
  início de oração; evitar mesóclise (não cabe em prosa científica) e próclise incorreta após
  pausa forte (ponto, ponto e vírgula).
- **Pontuação de orações intercaladas e explicativas**: aposto e oração explicativa entre
  vírgulas (nunca apenas uma vírgula de abertura sem a de fechamento); ponto e vírgula para
  separar orações longas já pontuadas internamente com vírgula, não vírgula simples.
- **Ambiguidade referencial**: pronome ou elipse cujo antecedente pode ser mais de um substantivo
  na frase anterior ("o critério supera o método fixo, que não generaliza" — "que" refere-se a
  qual dos dois?).
- **Registro formal consistente**: nada de coloquialismo, gíria técnica não glossada na primeira
  ocorrência, ou mistura de registro entre seções (uma seção em tom de manual, outra em tom de
  ensaio).

O padrão de aceitação é o de um gramático profissional revisando um texto para publicação, não
"compreensível" ou "sem erro grosseiro": toda frase deveria sobreviver à leitura de um revisor de
prova de editora acadêmica sem marcação.

## Passo 2c — rigor lógico-argumentativo

Além de gramática (2b) e fluidez (Passo 5), auditar a estrutura de raciocínio do texto,
frase a frase e parágrafo a parágrafo. Isto é distinto de checar números (Passo 2, itens 1-3):
aqui o número pode estar certo e a inferência que se tira dele, errada.

- **Cadeia premissa→evidência→conclusão.** Toda conclusão declarada tem uma premissa e uma
  evidência a até 2-3 linhas de distância? Reconstruir o silogismo implícito de cada claim forte
  ("logo", "portanto", "isso mostra que", "confirma que") e perguntar: as premissas realmente
  sustentam esta conclusão, ou apenas soam relacionadas a ela?
- **Falácias comuns em texto científico:**
  - *Generalização apressada*: conclusão ampla tirada de amostra pequena ou de um único caso.
  - *Post hoc ergo propter hoc*: correlação temporal ou de coocorrência apresentada como causal
    sem mecanismo ou controle que isole a causa.
  - *Non sequitur*: a conclusão não decorre logicamente das premissas apresentadas, mesmo que
    cada premissa isolada seja verdadeira.
  - *Petição de princípio*: a conclusão já está pressuposta numa das premissas, disfarçada de
    dedução independente.
  - *Falso dilema*: apresentar duas opções como exaustivas ("ou X ou Y") quando existe uma
    terceira alternativa não considerada.
- **Validade vs. solidez.** Um argumento pode ser formalmente válido (a conclusão decorre das
  premissas) mas não sólido (uma premissa é falsa). Toda vez que o texto usa um conectivo
  conclusivo ("logo", "portanto", "consequentemente", "isso implica"), checar se a premissa
  anterior é factualmente estabelecida (número real, citação real, já verificada) ou apenas
  assumida sem verificação — reconectar com o protocolo anti-fabricação de citações já em uso
  neste projeto.
- **Consistência lógica entre seções.** Uma conclusão do Resumo ou da Conclusão contradiz,
  enfraquece silenciosamente, ou generaliza além do que a evidência da seção de Resultados
  sustenta? Isto vai além de "inconsistência de número" (Passo 2, item 5): é sobre a força da
  afirmação bater com a força da evidência, não apenas os números baterem entre si.
- **Contrafactual do revisor cético.** Para cada conclusão forte do trabalho, formular
  explicitamente: "que evidência, se existisse, refutaria isto?" Se a resposta for "nenhuma
  evidência plausível refutaria", a claim está infalsificável demais para ciência — reformular
  para algo que um experimento poderia de fato derrubar.

## Passo 3 — idiossincrasias de texto gerado por IA (além do que o script pega)

- **Antítese "X, não Y" em excesso** (mais de 5-6 no documento): é a assinatura mais delatora de
  texto passado por LLM sem reescrita humana. Cada ocorrência isolada é aceitável; a repetição é
  o problema.
- **Tríades compulsivas**: enumerar sempre em grupos de três ("três fases", "três hipóteses",
  "três grandezas") mesmo quando o número real de itens é outro, ou quando a lista poderia ser
  maior/menor sem perda.
- **Negrito abrindo parágrafo como bullet disfarçado**: `\textbf{Frase de efeito.} Resto do
  parágrafo...` repetido 5+ vezes é estrutura de slide, não de prosa científica corrida.
- **Meta-comentário sobre o próprio processo de edição**: "nesta correção", "na versão revisada",
  "foi retirado", "ao reconferir" — um relatório final apresenta a versão final, não narra a
  história de suas próprias reescritas. Isso sinaliza ao leitor que o documento foi remendado às
  pressas.
- **Hedging excessivo / auto-desculpa**: "resultado mais modesto", "tratado aqui como preliminar
  e não confirmado... permanece válida independentemente desta ressalva" — defender-se antes de
  ser atacado, repetidamente, é tão ruim quanto não se defender.

## Passo 4 — checar se o corte de página não cortou substância

Armadilha específica deste tipo de projeto: comprimir um documento para caber num limite de
página é fácil de fazer cortando prosa redundante, mas fácil de errar cortando substância junto
(estudo de ablação, seção metodológica inteira, nota que contextualiza uma tabela). Antes de
aceitar uma versão comprimida como final:

```bash
wc -w <arquivo_fonte_completo.md> <arquivo_comprimido.tex>
```

Se o comprimido tem menos da metade das palavras do fonte, investigar especificamente: existe
ablação/baseline que sumiu? Existe seção metodológica que só existe na versão longa? Existe nota
de honestidade/limitação que contextualizava um resultado e desapareceu, deixando a claim nua e
mais forte do que os dados sustentam?

## Passo 5 — fluidez narrativa extrema

Diferente dos passos anteriores (auditoria/detecção), este é um passo de reescrita ativa.
Aplicar por último, depois de toda correção de conteúdo/lógica/gramática estar fechada — reescrever
fluência antes disso desperdiça trabalho em texto que ainda vai mudar de número ou estrutura.

- **Variação de abertura de frase.** Nenhuma frase deveria abrir com a mesma palavra ou estrutura
  sintática da frase imediatamente anterior dentro do mesmo parágrafo ("O critério mede... O
  critério também..." → variar o sujeito ou a construção da segunda frase).
- **Ritmo de leitura.** Alternar deliberadamente frases curtas (fecho de ideia, ênfase) com
  frases longas (desenvolvimento, encadeamento de causas). Um parágrafo inteiro de frases de
  comprimento semelhante, lido em voz alta, soa mecânico mesmo estando gramaticalmente perfeito.
- **Transição implícita antes de explícita.** Preferir que a relação lógica entre duas frases
  fique clara pela própria ordem das ideias e escolha de palavras; recorrer a conectivo explícito
  ("portanto", "logo", "dessa forma") só quando a relação não é óbvia sem ele. Conectivo em toda
  frase é sintoma de insegurança do texto, não de rigor — e é um dos traços mais delatores de
  prosa gerada por IA.
- **Uma ideia por parágrafo.** Se um parágrafo muda de assunto no meio, quebrar em dois. Se dois
  parágrafos adjacentes tratam do mesmo ponto sem avançar o argumento, fundir — mesma lógica já
  aplicada nas fusões de seções redundantes deste tipo de relatório.
- **Leitura em voz alta mental.** Para cada parágrafo revisado, simular a leitura em voz alta:
  qualquer trecho que exigiria pausar e reler para entender não está fluente, mesmo que
  gramaticalmente correto e factualmente preciso.
- **Critério de aceitação.** O texto deveria soar como um artigo publicado, escrito por um autor
  humano experiente na área, não como uma lista de fatos verdadeiros concatenados por conectivos.
  Se ao ler em voz alta o texto soa "correto mas robótico", ainda não passou neste passo.

## Passo 2d — força e narrativa da tese como um todo

Distinto dos passos anteriores (que auditam frase/parágrafo/seção isolados): aqui a unidade de
análise é o artigo inteiro como argumento único. Fazer esta passada depois do Passo 2c (rigor
lógico-argumentativo local), antes do Passo 5 (fluidez).

- **A tese central resiste a "e daí?".** Existe uma frase única, localizável no texto, que
  resume a contribuição central? Se essa frase fosse removida, o restante do artigo ainda faria
  sentido como conjunto coerente, ou viraria uma lista de resultados desconexos?
- **Contexto → lacuna → problema → contribuição, na Introdução, nesta ordem e de forma
  explícita.** Marcar linha a linha onde cada um desses quatro elementos aparece:
  - *Contexto/importância*: por que esse tema importa, para quem, com que urgência — não
    assumido, dito.
  - *Lacuna*: o que a literatura/prática atual não resolve, nomeada com precisão (não "há
    poucos estudos sobre X", mas "nenhum trabalho revisado reporta Y sob a condição Z").
  - *Problema*: a pergunta de pesquisa específica que decorre da lacuna, não uma reformulação
    genérica do tema.
  - *Contribuição*: o que este trabalho especificamente entrega, em uma lista ou frase
    enumerável, diferenciada do que já existe (não "avançamos o entendimento de X").
  Se qualquer um dos quatro só aparece implícito ou espalhado por várias seções sem uma
  âncora clara na Introdução, isso é um achado a reportar, não uma suposição a fazer por conta
  própria — citar a lacuna real linha a linha.
- **A lacuna é convincente, não apenas presente.** Não basta a lacuna existir no texto: ela
  precisa convencer o leitor de que vale a pena resolver — evidência de custo/impacto de não
  resolvê-la, ou de que tentativas anteriores esbarraram nela. Uma lacuna meramente descrita
  ("nenhum trabalho fez X") é mais fraca que uma lacuna motivada ("a ausência de X causa Y,
  como mostrado por Z").
- **Diferenciação explícita da contribuição.** Para cada trabalho relacionado citado como
  próximo/comparável, o texto precisa dizer clara e localizadamente o que este trabalho faz
  diferente — não deixar para o leitor inferir por justaposição. "Diferente de X, nós fazemos Y"
  é mais forte que citar X e Y lado a lado sem essa frase de ponte.
- **Cenário de uso concreto.** Para leitores não familiarizados com o domínio da aplicação,
  existe um exemplo tangível (não abstrato) de quando/onde/por quem essa contribuição seria
  usada na prática? A ausência de um cenário concreto é uma fraqueza real para acessibilidade
  do artigo fora do subcampo imediato — reportar como achado, não assumir que "fica implícito".
- **Melhor benchmark/estado da arte disponível, não apenas um benchmark plausível.** Toda
  comparação numérica contra trabalho relacionado precisa responder: esse é o resultado mais
  forte/recente publicamente disponível para essa tarefa, ou existe um número melhor que o
  texto não está enfrentando? Se a resposta não é verificável a partir do texto sozinho, isso é
  uma lacuna de honestidade a reportar, não a assumir como resolvida.
- **As figuras cobrem o que o texto alega, e são as mais fortes possíveis para o argumento.**
  Para cada alegação central do texto que poderia ser mostrada visualmente, existe uma figura
  correspondente? E, ao contrário, toda figura existente sustenta uma alegação central (nenhuma
  é decorativa)? Uma figura ausente para o achado mais importante do trabalho é um ponto cego
  sério — mais grave que uma figura existente com legenda fraca.
- **Revisão sistemática vs. seleção de conveniência.** O Related Work reflete uma revisão
  sistemática real (protocolo de busca declarado, critérios de inclusão/exclusão) ou é uma
  seleção de referências convenientes ao argumento? Se não houve SLR formal, isso deve ser
  nomeado como limitação honesta, nunca disfarçado de cobertura completa da literatura.
- **Palavras-chave para indexação internacional.** As keywords declaradas (`\keywords{}` ou
  equivalente) usam a terminologia que um pesquisador da área buscaria em bases indexadas
  (IEEE Xplore, Scopus, ACM DL) — termos consolidados do subcampo, não sinônimos idiossincráticos
  cunhados pelo próprio trabalho? Um termo técnico inventado no lugar do termo-padrão da área
  reduz achabilidade internacional, mesmo sendo tecnicamente correto.
- **Passo-a-passo do "como" está registrado, não só o "o quê".** Além de hardware/versões/seeds
  (Passo 2, item 3): o processo de decisão em si está documentado — por que essa abordagem e
  não outra considerada, em que ordem os experimentos/decisões aconteceram, o que foi tentado e
  descartado? Um artigo pode ser numericamente reprodutível e ainda assim omitir a lógica do
  processo que levou às escolhas finais.
- **Limitações reais, nomeadas com honestidade, não genéricas.** "Mais estudos são necessários"
  não é uma limitação real. Uma limitação real nomeia uma condição específica sob a qual o
  resultado pode não se sustentar, e por quê — o mesmo padrão de honestidade epistêmica já usado
  para achados que revertem o resultado principal do trabalho.
- **Pontos cegos do autor: a pergunta que a autora não sabe responder.** Para cada seção,
  formular explicitamente uma pergunta adversarial que um avaliador familiarizado com a área
  faria e que não tem resposta localizável no texto atual — não é suficiente que a resposta
  "exista na cabeça do autor"; se não está no texto, é um ponto cego a reportar nominalmente.

## Padrão-ouro de referência (não reinventar métrica que já existe)

Antes de aceitar uma métrica ou vocabulário criado pelo próprio trabalho, verificar se a área já
tem um equivalente consolidado na literatura. Inventar terminologia própria para algo que já tem
nome — uma métrica de distância a um "oráculo" que já se chama VBS/SBS em *algorithm selection*
(Rice, 1976), uma medida de erro que já é RMSE ou MAE em outro nome, um "índice de qualidade"
que reinventa uma métrica-padrão do subcampo — é o tipo de escolha que um revisor de área
reconhece na hora e cobra explicação. Reportar a métrica com o nome e a referência que a área já
usa é mais reconhecido internacionalmente do que uma métrica nova sem ancoragem, e geralmente é
computável dos mesmos dados que já existem, sem rodar experimento novo.

Da mesma forma, checar se existe um benchmark ou dataset canônico da subárea que um revisor
esperaria ver citado ou comparado — cada campo tem os seus (BARN e SPL/SCT em navegação
robótica, GLUE/SuperGLUE em NLP, ImageNet/COCO em visão computacional, MNIST/CIFAR como
baseline mínimo em ML tabular, e assim por diante). A ausência desse comparativo, quando ele
existe e é acessível, costuma ser a primeira pergunta de um revisor familiarizado com a área.

## Checklist final antes de dizer que está pronto

- [ ] Todo parâmetro numérico tem justificativa a até 2-3 linhas de distância
- [ ] Toda claim sobre literatura tem citação específica, não "a literatura, em sua maioria"
- [ ] Hardware declarado se há qualquer benchmark de tempo
- [ ] Toda referência bibliográfica é citada no corpo; toda citação existe na bibliografia
- [ ] Resumo e Conclusão não afirmam nada que o corpo do texto contradiz ou já retratou
- [ ] Objetivos declarados na Introdução são os mesmos do Plano de Trabalho oficial, não uma
      reformulação que promete algo (ex. "prova teórica") nunca entregue no corpo
- [ ] Zero travessão em prosa (regra deste projeto); antítese "X, não Y" usada com moderação
- [ ] O achado mais forte do trabalho resiste à pergunta adversarial mais óbvia sobre ele
- [ ] Toda conclusão forte tem premissa e evidência a até 2-3 linhas de distância; nenhuma
      falácia (generalização apressada, post hoc, non sequitur, petição de princípio, falso
      dilema) sobrevive à reconstrução do silogismo implícito
- [ ] Nenhuma claim é infalsificável ao ponto de nenhuma evidência plausível poder refutá-la
- [ ] Lido em voz alta (mentalmente), nenhum parágrafo soa mecânico ou exige pausa para reler
- [ ] Existe uma frase única e localizável que resume a tese central do trabalho
- [ ] Introdução tem contexto/importância, lacuna, problema e contribuição explícitos, nesta
      ordem, cada um localizável em linha específica (não implícito ou espalhado)
- [ ] A lacuna convence, não apenas descreve ("nenhum trabalho fez X" é mais fraco que "a
      ausência de X causa Y, como Z mostra")
- [ ] Para cada trabalho relacionado próximo, há uma frase explícita de diferenciação
      ("diferente de X, nós fazemos Y"), não apenas justaposição
- [ ] Existe cenário de uso concreto para leitor não familiarizado com o domínio da aplicação
- [ ] Toda comparação numérica contra a literatura usa o melhor/mais recente resultado
      publicamente disponível, não apenas um número plausível
- [ ] Toda alegação central tem figura correspondente; toda figura existente sustenta uma
      alegação central (nenhuma decorativa)
- [ ] Cobertura de literatura é revisão sistemática real ou está nomeada como seleção
      não-sistemática (nunca disfarçada de completude)
- [ ] Palavras-chave usam terminologia-padrão da área indexada internacionalmente (IEEE
      Xplore/Scopus/ACM DL), não sinônimo idiossincrático cunhado pelo próprio trabalho
- [ ] O "como" documenta o processo de decisão (por que essa abordagem, o que foi descartado),
      não só hardware/versões/seeds
- [ ] Toda limitação nomeada é específica e honesta ("mais estudos são necessários" não conta)
- [ ] Para cada seção, existe ao menos uma pergunta adversarial cuja resposta não está
      localizável no texto atual — reportada nominalmente como ponto cego
