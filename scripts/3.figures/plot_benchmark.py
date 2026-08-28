"""Gera gráfico bonito do benchmark para apresentação."""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import os

# Dados do benchmark
poses = list(range(1, 9))
erros = [0.013, 0.012, 0.014, 0.014, 0.014, 0.010, 0.011, 0.011]
erros_cm = [e * 100 for e in erros]
media = np.mean(erros_cm)
maximo = np.max(erros_cm)

# Setup
fig, ax = plt.subplots(figsize=(12, 6), dpi=150)
fig.patch.set_facecolor('#f8f9fa')
ax.set_facecolor('#ffffff')

# Cores: barras em azul, média em vermelho
colors = ['#3498db' if e <= media else '#e74c3c' for e in erros_cm]
bars = ax.bar(poses, erros_cm, color=colors, alpha=0.8, edgecolor='#2c3e50', linewidth=1.5, width=0.6)

# Linha de média
ax.axhline(y=media, color='#2ecc71', linestyle='--', linewidth=2.5, label=f'Erro médio: {media:.2f} cm')

# Eixos e labels
ax.set_xlabel('Pose (posição dos robôs)', fontsize=12, fontweight='bold')
ax.set_ylabel('Erro de Detecção (cm)', fontsize=12, fontweight='bold')
ax.set_title('Validação do Gêmeo Digital: Erro de Detecção por Pose',
             fontsize=14, fontweight='bold', pad=20)

# Grid
ax.grid(axis='y', alpha=0.3, linestyle=':', linewidth=1)
ax.set_axisbelow(True)

# Valores nas barras
for i, (pose, erro) in enumerate(zip(poses, erros_cm)):
    ax.text(pose, erro + 0.15, f'{erro:.1f}cm', ha='center', va='bottom',
            fontweight='bold', fontsize=10)

# Estatísticas no canto
stats_text = (
    f'📊 ESTATÍSTICAS\n'
    f'━━━━━━━━━━━━━━━\n'
    f'Poses testadas: 8/8 (100%)\n'
    f'Erro médio: {media:.2f} cm\n'
    f'Erro máx: {maximo:.2f} cm\n'
    f'Erro mín: {min(erros_cm):.2f} cm'
)
ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
        fontsize=10, verticalalignment='top', family='monospace',
        bbox=dict(boxstyle='round', facecolor='#ecf0f1', alpha=0.9, pad=1))

# Legenda
ax.legend(loc='upper right', fontsize=11, framealpha=0.95)

# Eixo Y
ax.set_ylim(0, max(erros_cm) * 1.3)
ax.set_xticks(poses)
ax.set_xticklabels([f'P{i}' for i in poses])

# Remove spines (bordas)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['left'].set_linewidth(0.5)
ax.spines['bottom'].set_linewidth(0.5)

plt.tight_layout()

# Salva
out = os.path.expanduser('~/Documentos/Projetos/cerise-turtlebot3-nav/docs/benchmark_plot.png')
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
print(f'✓ Gráfico salvo em: {out}')
plt.close()

# Também salva em formato maior para impressão
fig, ax = plt.subplots(figsize=(14, 8), dpi=200)
fig.patch.set_facecolor('#f8f9fa')
ax.set_facecolor('#ffffff')

colors = ['#3498db' if e <= media else '#e74c3c' for e in erros_cm]
bars = ax.bar(poses, erros_cm, color=colors, alpha=0.8, edgecolor='#2c3e50', linewidth=2, width=0.5)

ax.axhline(y=media, color='#2ecc71', linestyle='--', linewidth=3, label=f'Erro médio: {media:.2f} cm')

ax.set_xlabel('Pose', fontsize=16, fontweight='bold')
ax.set_ylabel('Erro de Detecção (cm)', fontsize=16, fontweight='bold')
ax.set_title('Validação do Gêmeo Digital: Erro de Detecção por Pose',
             fontsize=18, fontweight='bold', pad=30)

ax.grid(axis='y', alpha=0.3, linestyle=':', linewidth=1.5)
ax.set_axisbelow(True)

for i, (pose, erro) in enumerate(zip(poses, erros_cm)):
    ax.text(pose, erro + 0.2, f'{erro:.1f}', ha='center', va='bottom',
            fontweight='bold', fontsize=13)

ax.set_ylim(0, max(erros_cm) * 1.4)
ax.set_xticks(poses)
ax.set_xticklabels([f'Pose {i}' for i in poses], fontsize=12)
ax.tick_params(axis='y', labelsize=12)

ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.legend(loc='upper right', fontsize=13, framealpha=0.95)

plt.tight_layout()
out_large = os.path.expanduser('~/Documentos/Projetos/cerise-turtlebot3-nav/docs/benchmark_plot_large.png')
plt.savefig(out_large, dpi=200, bbox_inches='tight', facecolor='white')
print(f'✓ Gráfico grande salvo em: {out_large}')
