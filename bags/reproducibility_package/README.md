# Pacote de Reprodutibilidade: EKF de Fusão YOLO+Odometria (CERISE)

Material suplementar para o paper LAFusion 2026 — "Extended Kalman Filter
Fusion of Visual Detection and Odometry for Multi-Robot Position Estimation:
A Case Study on the CERISE Digital Twin" (título de trabalho).

## Git SHA de referência

```
3bf2521b719dc2195dbd43c6f9f0492bbe74e91a
```

Todo o código citado abaixo (com caminhos relativos à raiz do repositório
`cerise-turtlebot3-nav`) corresponde exatamente a este commit. Para
reproduzir os resultados, faça checkout deste SHA antes de rodar os scripts.

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
python3 scripts/calibrate_camera.py
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
python3 scripts/validate_ekf_synthetic.py
```

Deve reportar NEES≈3.0 e NIS≈2.0 (dentro de ~1-2% de erro relativo, ver nota
metodológica no próprio script sobre por que o teste chi-quadrado formal é
excessivamente rigoroso com muitas amostras agregadas).

### 6. Rodar a avaliação completa (EKF vs. baseline vs. ground truth)

```bash
python3 scripts/eval_ekf_vs_baseline.py
```

Reproduz a Tabela de Resultados do paper (3 condições: sem drift, drift
leve, drift agressivo — ver `params.yaml` para os números esperados).

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
   corretos.** A primeira tentativa de calibração (5 poses concentradas
   perto do centro do frame) deu erro de reprojeção de 0.018px — parecendo
   excelente — mas o `fx` calibrado estava 8.9x maior que o valor
   geometricamente esperado pelo FOV conhecido da câmera. É overfitting
   clássico por poses mal distribuídas. `calibrate_camera.py` inclui uma
   checagem de sanidade (`fx_sanity_ratio`) para detectar esse problema
   automaticamente — a calibração final aceita (4 poses espalhadas) tem
   `fx_sanity_ratio=1.33` (dentro da faixa aceitável 0.5x-2.0x).

## Referências bibliográficas citadas nos comentários do código

Ver seção "Related Work" do paper e o histórico completo de busca
bibliográfica (15 referências verificadas por texto completo) no plano de
desenvolvimento desta sessão — não reproduzido aqui por brevidade.
