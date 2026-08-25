# Pacote de Reprodutibilidade: EKF de Fusão YOLO+Odometria (CERISE)

Material suplementar para o paper LAFusion 2026 — "Extended Kalman Filter
Fusion of Visual Detection and Odometry for Multi-Robot Position Estimation:
A Case Study on the CERISE Digital Twin" (título de trabalho).

## Git SHA de referência

```
fd62b08c08f5b2f90255c55abf3bf2141c8f80ab
```

Todo o código citado abaixo (com caminhos relativos à raiz do repositório
`cerise-turtlebot3-nav`) corresponde exatamente a este commit. Para
reproduzir os resultados, faça checkout deste SHA antes de rodar os scripts.

**Nota (atualizado nesta revisão)**: o SHA anterior (`3bf2521b7...`) usava
uma calibração de câmera com os 4 intrínsecos livres, que se mostrou
mal-condicionada (ver item 4 dos achados metodológicos abaixo, revisado).
`camera_calibration.npz` neste pacote já reflete a versão corrigida.

## Conteúdo deste pacote

- `world_with_camera.world` — mundo Gazebo usado (3 robôs TurtleBot3 Waffle +
  câmera overhead fixa em `(0,0,3)`, `pitch=π/2`, FOV horizontal 60°).
- `turtlebot3_waffle.urdf` — descrição do robô (inclui LiDAR 2D de fábrica,
  não utilizado neste trabalho — ver Future Work no paper).
- `camera_calibration.npz` — resultado da calibração de câmera real
  (`cv2.calibrateCamera`, método de Zhang 2000): matriz intrínseca `mtx`,
  coeficientes de distorção `dist`, erro de reprojeção 0.162px.
- `params.yaml` — todos os hiperparâmetros do EKF, câmera, e resultados
  numéricos da avaliação, em formato estruturado.

Os 3 bags de dados brutos (formato MCAP) estão no diretório pai
(`bags/cenario{1,2,3}_*/`), não duplicados aqui por tamanho (~2MB cada).

## Dependências específicas (não óbvias)

- **ROS2 Humble** — `ros-humble-rosbag2-storage-mcap` **não vem instalado por
  padrão** nesta distro (ao contrário de Iron+, onde é mais comum já estar
  presente). Instalar manualmente:
  ```
  sudo apt-get install -y ros-humble-rosbag2-storage-mcap
  ```
- Python: `numpy`, `opencv-python`, `scipy`, `rclpy`, `rosbag2_py` (parte do
  ROS2 Humble), `cv_bridge`, `ultralytics` (YOLO).
- Modelo YOLO treinado: `model_robot_detector.pt` (6MB, na raiz do repo,
  mAP@0.5=0.995 — ver memória do projeto para detalhes de treino).

## Como reproduzir, passo a passo

### 1. Subir o ambiente Gazebo

```bash
cd cerise-turtlebot3-nav
./launch_3robots_with_camera.sh
# Aguardar ~70s+ até "[OK] Nav2 pronto!" no terminal
```

### 2. Subir o detector YOLO

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 run cerise_nav yolo_detector
```

Confirma funcionamento correto se o log mostrar `detections=3 matched=3
mean_error≈0.03m` continuamente.

### 3. (Opcional) Recalibrar a câmera do zero

```bash
python3 scripts/lafusion/0.setup/calibrate_camera.py
```

Gera `camera_calibration.npz` na raiz do repo. **Nota de honestidade
epistêmica**: o script inclui uma checagem de sanidade que compara o `fx`
calibrado contra o valor teórico esperado pelo FOV conhecido da câmera —
calibrações com poucas poses concentradas no mesmo local do frame produzem
erro de reprojeção artificialmente baixo mas parâmetros intrínsecos
fisicamente incorretos (ver seção "Achados metodológicos" abaixo).

### 4. Gravar dataset (já feito, bags inclusos em `bags/`)

```bash
ros2 bag record -o <nome> --storage mcap --use-sim-time \
  /robot_detections /robot1/odom /robot2/odom /robot3/odom \
  /robot1/scan /robot2/scan /robot3/scan
```

### 5. Validar o EKF sinteticamente (sem depender do Gazebo)

```bash
python3 scripts/lafusion/1.validation/validate_ekf_synthetic.py
```

Deve reportar NEES≈3.0 e NIS≈2.0 (dentro de ~1-2% de erro relativo, ver nota
metodológica no próprio script sobre por que o teste chi-quadrado formal é
excessivamente rigoroso com muitas amostras agregadas).

### 6. Rodar a avaliação completa (EKF vs. baseline vs. ground truth)

```bash
python3 scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py       # Tabela 1 (correction-instant), com RMSE
python3 scripts/lafusion/2.evaluation/eval_ekf_continuous_error.py  # Tabela 2 (continuous error), com RMSE
```

O primeiro reproduz a Tabela 1 do paper (3 condições: sem drift, drift
leve, drift agressivo). O segundo reproduz a Tabela 2 (erro medido a cada
leitura de odometria, não só nos instantes de correção — o achado central
do paper) — ver `params.yaml` para os números esperados de ambos.

## Achados metodológicos relevantes para reprodução (honestidade epistêmica)

Esta seção documenta descobertas feitas durante o desenvolvimento que afetam
diretamente a interpretação dos resultados — omiti-las tornaria os números
enganosos fora de contexto.

1. **Odometria do Gazebo é ruído-zero.** Não é uma medição realista de
   sensor — é a posição verdadeira do modelo físico simulado. Por isso o
   `yolo_detector.py` já a trata como "ground truth" (`_compute_error`), e a
   condição "sem drift artificial" do `eval_ekf_vs_baseline.py` é
   trivialmente perfeita para o baseline odom-only. Para avaliar o EKF de
   forma significativa, é necessário injetar drift artificial (parâmetro
   `POS_DRIFT_ODOM`, mesmo modelo de `allocation_env.py`).

2. **O ganho do EKF depende do regime de drift.** Com pouco drift, a
   detecção YOLO real (erro ~3-6cm) é *menos* precisa que a odometria ainda
   pouco corrompida — fundir as duas nesse regime não ajuda e pode até
   piorar levemente a estimativa (comportamento matematicamente correto de
   um filtro bem calibrado, não um bug). O ganho de +23.7% só aparece
   quando o drift acumulado ultrapassa o erro de detecção do YOLO.

3. **Dois bugs de implementação foram encontrados e corrigidos durante a
   avaliação** (não estavam presentes no design original, mas na primeira
   implementação do passo 3): (a) a predição substituía o estado pelo valor
   absoluto da odometria em vez de integrar o delta, descartando correções
   anteriores; (b) o `Q` calibrado sinteticamente era ordens de magnitude
   pequeno demais para o drift real, e ao corrigi-lo, a covariância do
   filtro podia explodir sem um teto quando o YOLO perdia detecções por
   oclusão — quebrando o gating de associação (Mahalanobis) e causando
   atribuição incorreta de detecções entre robôs. Ambos corrigidos; ver
   histórico de commits para o diagnóstico completo.

4. **Calibração de câmera: erro de reprojeção baixo não garante parâmetros
   corretos, mesmo com poses espalhadas.** Duas tentativas mostraram isso:
   a primeira (5 poses concentradas perto do centro do frame) deu erro de
   reprojeção de 0.018px mas `fx` 8.9x maior que o esperado — overfitting
   clássico por poses mal distribuídas. A segunda tentativa (poses
   espalhadas pelo frame, `fx_sanity_ratio=1.33`, então dentro da faixa
   "aceitável" 0.5x-2.0x usada originalmente) ainda estava mal-condicionada:
   ao aplicar essa calibração ao pipeline, o erro de posição saltou para
   ~0.75m (vs. ~0.03m da heurística por FOV). Causa raiz identificada: com a
   câmera fixa olhando reto para baixo, o tabuleiro de calibração fica
   sempre fronto-paralelo ao sensor em todas as poses (só variação de
   yaw/XY no próprio plano do chão, nunca de tilt relativo à câmera) — o
   caso degenerado clássico de Zhang (2000)/Sturm & Maybank (1999): sem
   variação de inclinação relativa, `fx` e a distância percebida ficam
   quase linearmente dependentes, permitindo erro de reprojeção baixo com
   intrínsecos fisicamente incorretos. **Correção aplicada**: como a
   geometria da câmera simulada já é conhecida com precisão (altura e FOV
   nominais exatos), `calibrate_camera.py` agora fixa `fx,fy,cx,cy` no
   valor nominal geométrico (`cv2.CALIB_FIX_FOCAL_LENGTH` +
   `cv2.CALIB_FIX_PRINCIPAL_POINT`) e calibra só a distorção — a prática
   padrão-ouro quando os intrínsecos nominais já são confiáveis. A
   calibração corrigida reproduz a heurística por FOV já usada em produção
   a menos de 0.001m de diferença, confirmando que ambas são
   geometricamente equivalentes (`fx_sanity_ratio` na checagem de sanidade
   antiga não detecta este tipo de mal-condicionamento — ver nota no
   próprio `calibrate_camera.py`).

5. **A lógica do EKF (predict/correct, `Q`, teto de covariância) está
   duplicada em 3 arquivos**: `src/cerise_nav/cerise_nav/ekf_fusion_node.py`
   (nó ROS2 de produção), `scripts/lafusion/2.evaluation/eval_ekf_vs_baseline.py` e
   `scripts/lafusion/2.evaluation/eval_ekf_continuous_error.py` (avaliação offline sobre os
   bags), e `scripts/lafusion/1.validation/validate_ekf_synthetic.py` (validação sintética) —
   cada um reimplementa a mesma matemática com nomes de método diferentes,
   em vez de importar de um módulo único compartilhado. Limitação de
   engenharia conhecida, não corrigida por risco de invalidar resultados já
   validados perto do prazo de submissão: uma correção futura no filtro
   precisa ser replicada manualmente nos 3 arquivos.

## Referências bibliográficas citadas nos comentários do código

Ver seção "Related Work" do paper e o histórico completo de busca
bibliográfica (15 referências verificadas por texto completo) no plano de
desenvolvimento desta sessão — não reproduzido aqui por brevidade.
