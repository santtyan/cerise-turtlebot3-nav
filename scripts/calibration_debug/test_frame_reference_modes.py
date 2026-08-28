#!/usr/bin/env python3
"""
Teste educacional: simula os 2 modos de publicação de odometria em Gazebo.

Este script demonstra a ambiguidade que causou o bug no commit 560558b.
Pode ser usado para entender como o problema aconteceu e por que testes
unitários não o detectaram.

Execução:
    python3 test_frame_reference_modes.py
"""

import numpy as np

# ============================================================================
# Configuração do Mundo
# ============================================================================

ROBOT_SPAWN = {
    'robot1': (0.0, 0.5),
    'robot2': (0.0, -0.5),
}

CAMERA_HEIGHT = 3.0
IMG_W, IMG_H = 640, 480

# Parâmetros da câmera (típicos para FOV 60°)
fx = 554.254
fy = 554.254
cx = 320.0
cy = 240.0


def project_to_pixels(world_x: float, world_y: float) -> tuple:
    """Projeta coordenada mundo para pixel."""
    cam_x = world_x
    cam_y = -world_y
    cam_z = CAMERA_HEIGHT

    u = fx * (cam_x / cam_z) + cx
    v = fy * (cam_y / cam_z) + cy

    norm_x = u / IMG_W
    norm_y = v / IMG_H

    return u, v, norm_x, norm_y


# ============================================================================
# Simulação dos 2 Modos
# ============================================================================

print("=" * 80)
print("TESTE: Ambiguidade de Frames de Referência em Gazebo")
print("=" * 80)

print("\n### CENÁRIO: Robot1 em spawn, SEM movimento ###\n")

print("Posição esperada no MUNDO: (0.0, 0.5)")
print("Posição esperada em PIXELS: (~320, ~147) [projeção correta]\n")

# --------- MODO A: Odom publica coordenadas MUNDO ---------
print("-" * 80)
print("MODO A: /robot1/odom publica em coordenadas MUNDO (esperado)")
print("-" * 80)

# Gazebo publica: posição absoluta no mundo
odom_x_modoA = 0.0
odom_y_modoA = 0.5

print(f"Gazebo world pose: ({0.0}, {0.5})")
print(f"/robot1/odom publicado: ({odom_x_modoA}, {odom_y_modoA})")
print(f"  → header.frame_id: 'odom'")
print(f"  → child_frame_id: 'base_footprint'")

print("\nCódigo dataset_collector.py (ATUAL):")
print("  p = msg.pose.pose.position")
print("  self.poses[name].x = p.x")
print("  self.poses[name].y = p.y")

pose_x_modoA = odom_x_modoA  # Direto
pose_y_modoA = odom_y_modoA

print(f"\nPose calculada em dataset_collector: ({pose_x_modoA}, {pose_y_modoA})")
px_a, py_a, norm_x_a, norm_y_a = project_to_pixels(pose_x_modoA, pose_y_modoA)
print(f"Projeção em pixels: ({px_a:.1f}, {py_a:.1f})")
print(f"Normalizado (YOLO): ({norm_x_a:.3f}, {norm_y_a:.3f})")
print(f"\nResultado: ✓ CORRETO - Bbox em cima do robô\n")

# --------- MODO B: Odom publica coordenadas LOCAIS ---------
print("-" * 80)
print("MODO B: /robot1/odom publica em coordenadas LOCAIS (possível bug)")
print("-" * 80)

# Gazebo publica: posição RELATIVA ao spawn (frame local)
odom_x_modoB = 0.0
odom_y_modoB = 0.0  # ← Relativo ao spawn em (0, 0.5)

print(f"Gazebo world pose: ({0.0}, {0.5})")
print(f"/robot1/odom publicado: ({odom_x_modoB}, {odom_y_modoB})")
print(f"  → header.frame_id: 'robot1/odom' (namespace local!)")
print(f"  → child_frame_id: 'robot1/base_footprint'")

print("\nCódigo dataset_collector.py (ATUAL):")
print("  p = msg.pose.pose.position")
print("  self.poses[name].x = p.x    # ← Usa direto, ERRADO!")
print("  self.poses[name].y = p.y")

pose_x_modoB_wrong = odom_x_modoB
pose_y_modoB_wrong = odom_y_modoB

print(f"\nPose calculada (ERRADO): ({pose_x_modoB_wrong}, {pose_y_modoB_wrong})")
px_b_wrong, py_b_wrong, norm_x_b_wrong, norm_y_b_wrong = \
    project_to_pixels(pose_x_modoB_wrong, pose_y_modoB_wrong)
print(f"Projeção em pixels: ({px_b_wrong:.1f}, {py_b_wrong:.1f})")
print(f"Normalizado (YOLO): ({norm_x_b_wrong:.3f}, {norm_y_b_wrong:.3f})")
print(f"\nResultado: ✗ ERRADO - Bbox na origem, fora do robô!")

# --------- MODO B COM FIX (commit 560558b) ---------
print("\n" + "-" * 80)
print("MODO B com FIX (commit 560558b): Soma offset ROBOT_SPAWN")
print("-" * 80)

print("\nCódigo dataset_collector.py (COMMIT 560558b):")
print("  spawn_x, spawn_y = ROBOT_SPAWN[name]")
print("  self.poses[name].x = p.x + spawn_x")
print("  self.poses[name].y = p.y + spawn_y")

spawn_x, spawn_y = ROBOT_SPAWN['robot1']
pose_x_modoB_fixed = odom_x_modoB + spawn_x
pose_y_modoB_fixed = odom_y_modoB + spawn_y

print(f"\nOffset ROBOT_SPAWN: ({spawn_x}, {spawn_y})")
print(f"Pose calculada (com fix): ({pose_x_modoB_fixed}, {pose_y_modoB_fixed})")
px_b_fixed, py_b_fixed, norm_x_b_fixed, norm_y_b_fixed = \
    project_to_pixels(pose_x_modoB_fixed, pose_y_modoB_fixed)
print(f"Projeção em pixels: ({px_b_fixed:.1f}, {py_b_fixed:.1f})")
print(f"Normalizado (YOLO): ({norm_x_b_fixed:.3f}, {norm_y_b_fixed:.3f})")
print(f"\nResultado: ✓ CORRETO - Fix funciona para Modo B")

# --------- PROBLEMA: Sistema está em Modo A, não B ---------
print("\n" + "-" * 80)
print("O PROBLEMA DE 560558b: Sistema estava em MODO A, NÃO B!")
print("-" * 80)

print("\nModo realmente sendo usado: A (odom em mundo)")
print(f"Gazebo publicava: ({odom_x_modoA}, {odom_y_modoA})")

print("\nMas código (560558b) assumiu: B (odom relativo)")
pose_x_erro = odom_x_modoA + spawn_x  # Soma offset desnecessário!
pose_y_erro = odom_y_modoA + spawn_y

print(f"Aplicou fix incorretamente: ({odom_x_modoA}, {odom_y_modoA}) + ({spawn_x}, {spawn_y})")
print(f"Resultado: ({pose_x_erro}, {pose_y_erro})")
px_erro, py_erro, norm_x_erro, norm_y_erro = \
    project_to_pixels(pose_x_erro, pose_y_erro)
print(f"Projeção em pixels: ({px_erro:.1f}, {py_erro:.1f})")
print(f"Normalizado (YOLO): ({norm_x_erro:.3f}, {norm_y_erro:.3f})")
print(f"\nResultado: ✗ ERRADO - Bbox deslocado para ({pose_x_erro}, {pose_y_erro})")

# ============================================================================
# Resumo da Análise
# ============================================================================

print("\n" + "=" * 80)
print("RESUMO: Por que 560558b causou erro (e foi revertido)")
print("=" * 80)

print(f"""
1. Sistema era MODO A: /robot1/odom = (0.0, 0.5)

2. Código antigo (pré-560558b) assumia MODO A:
   pose = odom_x = 0.0, odom_y = 0.5 → pixel (320, 147.6) ✓

3. Desenvolvedor pensou: "odom é relativo ao spawn, preciso adicionar offset"
   Adicionou: pose = odom_x + spawn_x = 0.0 + 0.0 = 0.0
             pose = odom_y + spawn_y = 0.5 + 0.5 = 1.0 → pixel (320, ~55) ✗

4. Bbox aparecia errado (mais ao topo, fora do robô)

5. Depois de 2 semanas, identificou erro e reverteu o commit

6. Código voltou ao original (MODO A):
   pose = odom_x = 0.0, odom_y = 0.5 → pixel (320, 147.6) ✓
""")

# ============================================================================
# Lições Aprendidas
# ============================================================================

print("\n" + "=" * 80)
print("LIÇÕES: Como evitar este tipo de erro no futuro")
print("=" * 80)

print(f"""
1. NÃO ASSUMA qual modo está sendo usado
   → Valide experimentalmente com RViz

2. NÃO USE valores hardcoded em testes
   → Use dados reais de Gazebo/ROS

3. IMPLEMENTE validação automática
   → Verificar se poses fazem sentido
   → Avisar se mudou para frame local

4. USE TF para transformação robusta
   → TF converte automaticamente entre frames
   → Funciona em qualquer modo

5. DOCUMENTE qual frame está sendo usado
   → Em docstrings, comentários, logs

Exemplo de código robusto (recomendado):

    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_point

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, self)

    # Em vez de assumir:
    p = msg.pose.pose.position
    pose_x = p.x

    # Sempre transformar para 'world':
    transform = tf_buffer.lookup_transform('world', f'{{name}}/base_footprint')
    pose_x = transform.transform.translation.x
    pose_y = transform.transform.translation.y

    # Isto funciona em qualquer configuração!
""")

# ============================================================================
# Tabela Comparativa
# ============================================================================

print("\n" + "=" * 80)
print("TABELA: Comparação dos 3 cenários")
print("=" * 80)

print(f"""
{"|": <5} {"Cenário": <30} {"Odom": <20} {"Pixel": <15} {"Status": <15} |")
{"|": <5} {"-"*30} {"-"*20} {"-"*15} {"-"*15} |
{"|": <5} {"MODO A (esperado)": <30} {"(0.0, 0.5)": <20} {f"({px_a:.0f}, {py_a:.0f})": <15} {"✓ Correto": <15} |
{"|": <5} {"MODO B sem fix": <30} {"(0.0, 0.0)": <20} {f"({px_b_wrong:.0f}, {py_b_wrong:.0f})": <15} {"✗ Errado": <15} |
{"|": <5} {"MODO B com fix": <30} {"(0.0, 0.0)": <20} {f"({px_b_fixed:.0f}, {py_b_fixed:.0f})": <15} {"✓ Correto": <15} |
{"|": <5} {"MODO A + fix (560558b)": <30} {"(0.0, 0.5)": <20} {f"({px_erro:.0f}, {py_erro:.0f})": <15} {"✗ Errado": <15} |
""")

print("=" * 80)
print("\nConclusão: O bug 560558b ocorreu porque o fix era correto para MODO B,")
print("           mas o sistema estava em MODO A.")
print("\nSolução:   Use TF para transformação automática (funciona em qualquer modo).")
print("=" * 80)
