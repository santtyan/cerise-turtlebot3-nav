# LAFusion 2026 — Material Consolidado

Pasta única com todo o material do paper "Extended Kalman Filter Fusion of
Visual Detection and Odometry for Multi-Robot Position Estimation: A Case
Study on the CERISE Digital Twin" (título de trabalho), organizado para
consulta e submissão.

**Git SHA de referência**: `6e71e56` (branch `feature/rl-task-allocator`)

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
- `lafusion_trajectory.png` — trajetória EKF vs. odometria (com drift) vs.
  ground truth, cenário `cenario1_parado` (ganho de +46.3%, o mais
  representativo do resultado agregado).
- `lafusion_error_over_time.png` — erro de posição ao longo do tempo, 3
  cenários lado a lado.
- `lafusion_architecture.png` — diagrama do pipeline (câmera→calibração→
  YOLO→associação→EKF→pose fundida).

### scripts/
- `validate_ekf_synthetic.py` — etapa 1.5: validação sintética NEES/NIS.
- `calibrate_camera.py` — passo 2.5: calibração de câmera com checagem de
  sanidade (fx vs. valor teórico).
- `eval_ekf_vs_baseline.py` — passo 4: EKF vs. odom-only vs. ground truth
  nos 3 bags reais (resultado principal do paper).
- `plot_ekf_results.py` / `plot_architecture_diagram.py` — geração das
  figuras acima.

### code/
- `ekf_fusion_node.py` — nó ROS2 de produção (EKF por robô, gating por
  Mahalanobis).
- `association.py` — módulo de associação compartilhado (greedy +
  Mahalanobis).

### bags/
- `cenario{1,2,3}_*` — dados brutos gravados no Gazebo (MCAP).
- `reproducibility_package/` — README detalhado, `params.yaml` com todos os
  hiperparâmetros e resultados numéricos, URDF, world file, calibração de
  câmera. **Ver `reproducibility_package/README.md` para o passo-a-passo
  completo de reprodução e os achados metodológicos que dão contexto aos
  números.**

## Resultado principal (resumo)

| Condição | Erro EKF | Erro odom-only | Ganho |
|---|---|---|---|
| Sem drift artificial | 3.6cm | 0.0cm | 0% (odom = ground truth por definição) |
| Drift leve | 5.3cm | 5.0cm | -5.3% |
| **Drift agressivo** | **12.8cm** | **16.8cm** | **+23.7%** (Wilcoxon p<0.001, n=1323) |

Ver `bags/reproducibility_package/README.md` para a interpretação completa
e as ressalvas metodológicas (por que o ganho depende do regime de drift,
os 2 bugs encontrados e corrigidos durante a avaliação, etc.).
