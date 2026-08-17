# CERISE — TurtleBot3 Multi-Robot Digital Twin

## Comandos que você não vai adivinhar

- **Nunca usar Docker** — decisão de escopo já tomada para os papers LARS/LAFusion.
- **Sempre `python3`**, nunca `python`.
- Compilar papers LaTeX: `pdflatex -interaction=nonstopmode main.tex` — rodar **2 vezes** (referências cruzadas só resolvem na 2ª passada). Contar páginas: `pdfinfo main.pdf | grep Pages`.
- Subir o pipeline multi-robô: `./launch_3robots_with_camera.sh`, aguardar **70s+** antes de rodar qualquer outro nó (Nav2 demora a estabilizar).
- Nós zumbis do Gazebo/Nav2 travados: `pkill -9 component_container` antes de tentar relançar — reiniciar sem isso deixa portas DDS presas.
- Se um nó ROS2 continua logando mas nenhum subscriber CLI recebe nada (mesmo com QoS compatível), é estado DDS travado do processo específico — mate e relance, não precisa reiniciar o ambiente inteiro.

## Git

- **Nunca incluir `Co-Authored-By: Claude` em commits** — preferência explícita do usuário.
- Sempre revisar `git status` antes de `git add` amplo — o repo acumula arquivos soltos de sessões antigas (ex. `docs/RSL/`, `docs/paper_lars/`) que não devem ser commitados sem confirmar do que se trata.

## Ambiente (Wayland/GNOME + VSCode snap)

- Ferramentas de captura X11 legadas (`scrot`, `import`, `gnome-screenshot`) retornam tela preta sob Wayland — não há captura de tela 100% automatizada neste ambiente.
- Variáveis injetadas pelo VSCode snap (`GTK_PATH`, `GDK_PIXBUF_MODULEDIR`, etc.) quebram binários nativos (RViz2 e afins) com erro de símbolo GLIBC — `unset` essas variáveis antes de rodar binários gráficos do sistema.

## Papers científicos deste projeto

Dois papers em desenvolvimento no repo: LARS 2026 (`docs/paper_lars/`, já submetido) e LAFusion 2026 (`docs/lafusion_paper/`, EKF de fusão sensorial). Antes de escrever/editar qualquer seção de paper, ou gerar figura científica, ou adicionar citação — ver as skills abaixo, que carregam os procedimentos detalhados sob demanda:

- `verify-citation` — protocolo de verificação de referência bibliográfica antes de incluir no `.tex`.
- `verify-claim-against-code` — protocolo de checar toda alegação técnica/numérica contra código ou execução real.
- `paper-figure-style` — padrão-ouro de figura científica (matplotlib) para papers de robótica/fusão sensorial deste projeto.
- `anonymize-paper` — checklist de anonimização double-blind (`.tex` + metadados de PDF).
- `critical-presentation-review` — revisão crítica sem pontos cegos para slides/apresentações (CERISE, LARS, LAFusion): toda decisão justificada, arquitetura e benchmarks explícitos, perguntas difíceis antecipadas.

## RViz2 não tem screenshot programático

Não existe serviço ROS2 nem binding Python para captura de tela do RViz2 no Humble — não tentar essa rota para gerar figuras de paper. Ver `paper-figure-style` para a alternativa (matplotlib sobre dados de bag/tópico).
