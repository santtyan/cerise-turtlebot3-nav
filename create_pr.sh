#!/bin/bash
# Script para criar PR via GitHub API

set -e

REPO="santtyan/cerise-turtlebot3-nav"
BASE_BRANCH="main"
HEAD_BRANCH="feature/yolo-dataset"
TOKEN="${GITHUB_TOKEN:-}"

if [ -z "$TOKEN" ]; then
    echo "❌ Erro: GITHUB_TOKEN não definido"
    echo ""
    echo "Para usar este script:"
    echo "1. Crie um Personal Access Token em: https://github.com/settings/tokens"
    echo "   - Scope: 'repo' (Full control of private repositories)"
    echo ""
    echo "2. Execute:"
    echo "   export GITHUB_TOKEN='seu_token_aqui'"
    echo "   bash create_pr.sh"
    echo ""
    exit 1
fi

TITLE="feat: Interface gráfica Gazebo + 2-robots simulação"
BODY=$(cat <<'EOF'
## 📋 Resumo Executivo

Interface gráfica Gazebo agora **100% funcional** com simulação multi-robô pronta para coleta de dataset YOLO.

## ✅ Implementado

- [x] GUI Gazebo ativada (DISPLAY=:0, QT_QPA_PLATFORM=xcb)
- [x] 2 TurtleBot3 Waffle spawned automaticamente
- [x] Câmera overhead ativa (640×480 RGB8 @ 10Hz)
- [x] Odometria dos robots publicando
- [x] Script run_gui.sh para inicialização simples
- [x] Launch file ROS2 alternativo (gazebo_2robots.launch.py)
- [x] Documentação completa (QUICK_START + CHECKLIST)

## 🔧 Principais Correções

| Problema | Solução |
|----------|---------|
| Gazebo não abria GUI | Ativar DISPLAY=:0, QT_QPA_PLATFORM=xcb |
| XML malformado | Mover camera_overhead para dentro </world> |
| Caminhos WSL inválidos | Usar caminhos relativos $SCRIPT_DIR |
| Spawn vazio | Corrigir -file "$WAFFLE" |
| Namespace câmera | Ajustar para /camera/image_raw |

## 🚀 Como Testar

```bash
cd ~/cerise-turtlebot3-nav
./run_gui.sh
```

**Esperado:**
- ✅ Janela Gazebo aparece em ~3s
- ✅ 2 robots visíveis (azul + laranja)
- ✅ Câmera overhead ativa
- ✅ Topics de odometria publicando

## 📂 Arquivos Modificados

- `launch_2robots_with_camera.sh` — Corrigido (caminhos + GUI)
- `world_with_camera.model` — XML fix (camera inside </world>)
- `run_gui.sh` — Novo (inicialização 3-passos)
- `launch/gazebo_2robots.launch.py` — Novo (ROS2 launch proper)
- `QUICK_START.md` — Novo (guia usuário)
- `CHECKLIST_VALIDATION.md` — Novo (validação step-by-step)
- `PR_TEMPLATE.md` — Novo (documentação PR)

## 🎯 Status

✅ **Prof. Alisson - Item 1 Completo**: Simular cenário multi-robô (2x TurtleBot3 Waffle)

## 📋 Próximas Fases

1. **Dataset Collection** → `ros2 run cerise_nav dataset_collector`
2. **YOLO Training** → `yolo detect train data=dataset.yaml`
3. **Validation** → Testar inferência camera → posição

---

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
EOF
)

echo "📍 Criando Pull Request..."
echo "   Repo: $REPO"
echo "   Base: $BASE_BRANCH"
echo "   Head: $HEAD_BRANCH"
echo ""

# Fazer chamada à API
RESPONSE=$(curl -s -X POST \
  -H "Authorization: token $TOKEN" \
  -H "Accept: application/vnd.github.v3+json" \
  https://api.github.com/repos/$REPO/pulls \
  -d @- << PAYLOAD
{
  "title": "$TITLE",
  "body": $(echo "$BODY" | jq -R -s .),
  "base": "$BASE_BRANCH",
  "head": "$HEAD_BRANCH"
}
PAYLOAD
)

# Verificar se houve erro
if echo "$RESPONSE" | grep -q '"id"'; then
    PR_URL=$(echo "$RESPONSE" | grep -o '"html_url": "[^"]*' | cut -d'"' -f4)
    PR_NUM=$(echo "$RESPONSE" | grep -o '"number": [0-9]*' | grep -o '[0-9]*')

    echo "✅ Pull Request criada com sucesso!"
    echo ""
    echo "🔗 PR #$PR_NUM"
    echo "   URL: $PR_URL"
    echo ""
    echo "📋 Title: $TITLE"
    echo ""
else
    echo "❌ Erro ao criar PR:"
    echo "$RESPONSE" | jq .
    exit 1
fi
