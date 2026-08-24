#!/usr/bin/env python3
"""Gera diagrama MDP limpo para o Slide 5."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os

fig, ax = plt.subplots(figsize=(10, 6), facecolor='white')
ax.set_xlim(0, 10)
ax.set_ylim(0, 6)
ax.axis('off')

# Título
ax.text(5, 5.6, 'MRTA — AllocationEnv MDP Formulation',
        ha='center', fontsize=14, fontweight='bold', color='#0F172A')

# Caixa Estado
state_box = FancyBboxPatch((0.2, 3.5), 1.8, 1.2,
                           boxstyle="round,pad=0.1",
                           edgecolor='#3B82F6', facecolor='#DBEAFE',
                           linewidth=2)
ax.add_patch(state_box)
ax.text(1.1, 4.4, 'State (s)', ha='center', fontweight='bold', fontsize=10, color='#0369A1')
ax.text(1.1, 4.05, 'dim = 21', ha='center', fontsize=9, color='#0369A1')
ax.text(1.1, 3.75, '• YOLO positions', ha='center', fontsize=7.5, color='#0369A1')

# Anotação Estado
ax.text(1.1, 3.35, '6 coords (x,y)\n3 occupancy\n4 demand\n8 lookahead',
        ha='center', fontsize=7, color='#1E40AF', style='italic',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7, pad=0.3))

# Seta Estado → Agente
arrow1 = FancyArrowPatch((2.1, 4.1), (3.3, 4.1),
                        arrowstyle='->', mutation_scale=25,
                        linewidth=2.5, color='#475569')
ax.add_patch(arrow1)

# Caixa Agente (PPO)
agent_box = FancyBboxPatch((3.3, 3.5), 1.8, 1.2,
                          boxstyle="round,pad=0.1",
                          edgecolor='#A855F7', facecolor='#F3E8FF',
                          linewidth=2)
ax.add_patch(agent_box)
ax.text(4.2, 4.4, 'Agent', ha='center', fontweight='bold', fontsize=10, color='#6D28D9')
ax.text(4.2, 4.05, 'PPO', ha='center', fontsize=9, color='#6D28D9')
ax.text(4.2, 3.75, 'Stable-Baselines3', ha='center', fontsize=7.5, color='#6D28D9')

# Seta Agente → Ação
arrow2 = FancyArrowPatch((5.1, 4.1), (6.3, 4.1),
                        arrowstyle='->', mutation_scale=25,
                        linewidth=2.5, color='#475569')
ax.add_patch(arrow2)

# Caixa Ação
action_box = FancyBboxPatch((6.3, 3.5), 1.8, 1.2,
                           boxstyle="round,pad=0.1",
                           edgecolor='#EF4444', facecolor='#FEE2E2',
                           linewidth=2)
ax.add_patch(action_box)
ax.text(7.2, 4.4, 'Action (a)', ha='center', fontweight='bold', fontsize=10, color='#991B1B')
ax.text(7.2, 4.05, 'Choose robot', ha='center', fontsize=9, color='#991B1B')
ax.text(7.2, 3.75, '{0, 1, 2}', ha='center', fontsize=8, color='#991B1B', family='monospace')

# Seta Ação → Ambiente (curva)
arrow3 = FancyArrowPatch((7.2, 3.4), (7.2, 2.4),
                        arrowstyle='->', mutation_scale=25,
                        linewidth=2.5, color='#475569', connectionstyle="arc3,rad=0.3")
ax.add_patch(arrow3)

# Caixa Ambiente
env_box = FancyBboxPatch((5.8, 1.2), 2.8, 1,
                        boxstyle="round,pad=0.1",
                        edgecolor='#10B981', facecolor='#D1FAE5',
                        linewidth=2)
ax.add_patch(env_box)
ax.text(7.2, 1.85, 'Environment (AllocationEnv)', ha='center', fontweight='bold', fontsize=10, color='#065F46')
ax.text(7.2, 1.5, 'Nav2 → real latency', ha='center', fontsize=8, color='#065F46')

# Seta Ambiente → Recompensa (curva)
arrow4 = FancyArrowPatch((5.7, 1.7), (4.0, 2.0),
                        arrowstyle='->', mutation_scale=25,
                        linewidth=2.5, color='#475569', connectionstyle="arc3,rad=0.3")
ax.add_patch(arrow4)

# Caixa Recompensa
reward_box = FancyBboxPatch((1.5, 1.2), 2.5, 1,
                           boxstyle="round,pad=0.1",
                           edgecolor='#F59E0B', facecolor='#FEF3C7',
                           linewidth=2)
ax.add_patch(reward_box)
ax.text(2.75, 1.85, 'Reward (r)', ha='center', fontweight='bold', fontsize=10, color='#92400E')
ax.text(2.75, 1.5, '−(wait + travel)', ha='center', fontsize=8, color='#92400E', family='monospace')

# Seta Recompensa → Estado (feedback)
arrow5 = FancyArrowPatch((2.75, 2.2), (1.6, 3.5),
                        arrowstyle='->', mutation_scale=25,
                        linewidth=2.5, color='#94A3B8', linestyle='dashed', connectionstyle="arc3,rad=0.3")
ax.add_patch(arrow5)
ax.text(1.0, 2.8, 'new state', fontsize=7, color='#475569', style='italic')

# Caixa resumo objective
summary_box = FancyBboxPatch((0.2, 0.05), 9.6, 0.9,
                            boxstyle="round,pad=0.08",
                            edgecolor='#64748B', facecolor='#F1F5F9',
                            linewidth=1.5, linestyle='--')
ax.add_patch(summary_box)
ax.text(5, 0.72, 'Objective: Policy π(a | s) that minimizes E[Σ response_time]  =  maximizes E[Σ reward]',
        ha='center', fontsize=9, color='#1E293B', fontweight='bold')
ax.text(5, 0.35, 'Training: 500k steps | Ablation: PPO(YOLO) vs PPO(odom) | Evaluation: 1000 paired episodes vs nearest_free + oracle',
        ha='center', fontsize=8, color='#475569', style='italic')

plt.tight_layout()
out = 'docs/legacy/slides_amanha/H_mdp_diagram.png'
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
print(f'Diagrama MDP salvo: {out}')
