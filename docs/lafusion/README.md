# LAFusion 2026 — Material Consolidado

Pasta única com todo o material do paper "Extended Kalman Filter Fusion of
Visual Detection and Odometry for Multi-Robot Position Estimation: A Case
Study on the CERISE Digital Twin" (título de trabalho), organizado para
consulta e submissão.

**Git SHA de referência**: ver `git log -1` no momento da submissão (branch `feature/rl-task-allocator`)

> Os arquivos aqui são **cópias** dos originais no repositório — mantidos
> para consulta/empacotamento do paper. Os arquivos-fonte "vivos" (editáveis,
> com os imports relativos corretos para rodar) continuam em `scripts/`,
> `src/cerise_nav/cerise_nav/`, e `bags/` na raiz do repositório. Para
> executar qualquer script, use os originais, não as cópias desta pasta.

## Estrutura

```
docs/lafusion/
├── figures/          — 3 figuras do paper (trajetória, erro, arquitetura)
├── scripts/          — cópia dos 5 scripts de validação/avaliação/plot
├── code/             — cópia do código de produção (EKF + associação)
├── bags/             — 3 cenários gravados (MCAP) + pacote de reprodutibilidade
└── README.md         — este arquivo
```

### figures/

**Resultados / dados (matplotlib, estilo IEEE — fonte serifada, paleta
Okabe-Ito colorblind-safe, DPI 400, ver `STYLE_NOTES` abaixo):**
- `lafusion_trajectory.png` — trajetória EKF vs. odometria (com drift) vs.
  ground truth, cenário `cenario1_parado` (ganho de +46.3%, o mais
  representativo do resultado agregado). Scatter de dispersão, não linha.
- `lafusion_error_over_time.png` — erro de posição ao longo do tempo
  (suavizado), 3 cenários lado a lado, ylim compartilhado.
- `lafusion_nees_nis.png` — séries de NEES/NIS da validação sintética
  (etapa 1.5) com banda de confiança de 95% e valor esperado marcados.
- `lafusion_architecture.png` — diagrama do pipeline (câmera→calibração→
  YOLO→associação→EKF→pose fundida).

**Capturas reais do sistema rodando (não esquemas — screenshots do
Gazebo/ROS2 em produção, 13/08/2026):**
- `lafusion_pipeline_camera_raw.jpg` — frame bruto da câmera overhead
  (3 robôs visíveis, `/camera/image_raw`).
- `lafusion_pipeline_yolo_detection.jpg` — mesmo frame com detecção YOLO
  desenhada (bounding boxes + confiança real: 0.90/0.92/0.89,
  `/detection_image`).
- `lafusion_pipeline_calibration_board.jpg` — tabuleiro de calibração real
  spawnado no Gazebo durante o passo 2.5, visto pela câmera overhead.
- `lafusion_validation_terminal.png` — captura estilizada (fundo terminal)
  da execução real de `validate_ekf_synthetic.py`, com o output completo
  (NEES/NIS, "filtro PRATICAMENTE consistente").

*(Tentativa de capturar `gzclient`, a GUI 3D do Gazebo, não teve sucesso —
ambiente sem renderização gráfica real/headless, tela preta. A câmera
overhead acima é a fonte de imagem real usada pelo pipeline, então cobre
o que importa cientificamente.)*

### scripts/
- `validate_ekf_synthetic.py` — etapa 1.5: validação sintética NEES/NIS.
- `calibrate_camera.py` — passo 2.5: calibração de câmera. Corrigida nesta
  revisão: fx/fy/cx/cy agora fixados no nominal geométrico (a calibração
  completa dos 4 intrínsecos era mal-condicionada, ver
  `bags/reproducibility_package/README.md` item 4).
- `eval_ekf_vs_baseline.py` — passo 4: EKF vs. odom-only vs. ground truth
  nos 3 bags reais, erro medido nos instantes de correção YOLO (Tabela 1
  do paper), com RMSE.
- `eval_ekf_continuous_error.py` — erro medido a cada leitura de odometria,
  não só nos instantes de correção (Tabela 2 do paper, o achado central),
  com RMSE.
- `plot_ekf_results.py` / `plot_architecture_diagram.py` / `plot_nees_nis.py`
  — geração das figuras de resultado/diagrama.
- `render_terminal_screenshot.py` — gera a captura estilizada da validação.

### code/
- `ekf_fusion_node.py` — nó ROS2 de produção (EKF por robô, gating por
  Mahalanobis).
- `association.py` — módulo de associação compartilhado (greedy +
  Mahalanobis).
- `projection.py` — projeção world↔pixel (heurística por FOV, em produção;
  e via intrínsecos calibrados, `pixel_to_world_with_camera`/
  `world_to_pixel_with_camera`, ambas confirmadas geometricamente
  equivalentes após a correção da calibração).
- `yolo_detector.py` — nó ROS2 que publica `/robot_detections`; parâmetro
  `use_calibrated_projection` (default `False`) permite A/B entre as duas
  projeções acima sem mudar o comportamento padrão.

### bags/
- `cenario{1,2,3}_*` — dados brutos gravados no Gazebo (MCAP).
- `reproducibility_package/` — README detalhado, `params.yaml` com todos os
  hiperparâmetros e resultados numéricos, URDF, world file, calibração de
  câmera. **Ver `reproducibility_package/README.md` para o passo-a-passo
  completo de reprodução e os achados metodológicos que dão contexto aos
  números.**

## Resultado principal (resumo)

**Tabela 1 — erro nos instantes de correção YOLO** (`eval_ekf_vs_baseline.py`):

| Condição | Erro EKF | Erro odom-only | Ganho (mean / RMSE) |
|---|---|---|---|
| Sem drift artificial | 3.6cm | 0.0cm | 0% (odom = ground truth por definição) |
| Drift leve | 5.3cm | 5.0cm | -5.3% / -2.1% |
| **Drift agressivo** | **12.8cm** | **16.8cm** | **+23.7% / +16.7%** (Wilcoxon p<0.001, n=1323) |

**Tabela 2 — erro contínuo, a cada leitura de odometria** (`eval_ekf_continuous_error.py`,
achado central do paper): sob a mesma condição de drift agressivo, mas
medindo continuamente em vez de só nos instantes de correção, o EKF é
**12.7% pior** que odometria pura (RMSE: -13.4%, Wilcoxon p<0.001, n=1607) —
sinal invertido em relação à Tabela 1, explicado no paper como consequência
teórica esperada de operar abaixo da taxa crítica de observação (Sinopoli
et al. 2004), não uma falha de implementação.

Ver `bags/reproducibility_package/README.md` para a interpretação completa
e as ressalvas metodológicas (por que o ganho depende do regime de drift,
os bugs encontrados e corrigidos durante a avaliação, a correção da
calibração de câmera, etc.).
