#!/usr/bin/env python3
"""
Unit test: Simula o cálculo de pose (odom + offset spawn).
Verifica se o resultado bate com o esperado.
"""

ROBOT_SPAWN = {
    'robot1': (0.0,  0.5),
    'robot2': (0.0, -0.5),
}

# Simulando diferentes leituras de odômetro
test_cases = [
    # (odom.x, odom.y, robot_name, expected_world_x, expected_world_y, description)
    (0.0,   0.0,  'robot1',  0.0,   0.5,  "Robot1 na origem do odom"),
    (0.0,   0.0,  'robot2',  0.0,  -0.5,  "Robot2 na origem do odom"),
    (0.5,   0.0,  'robot1',  0.5,   0.5,  "Robot1 moveu +0.5 em X"),
    (0.0,   0.3,  'robot1',  0.0,   0.8,  "Robot1 moveu +0.3 em Y"),
    (-1.0, -1.0,  'robot1', -1.0,  -0.5,  "Robot1 moveu -1 em XY"),
]

print("=== Testando cálculo de pose: odom + spawn_offset ===\n")

all_pass = True
for odom_x, odom_y, robot_name, expected_x, expected_y, desc in test_cases:
    spawn_x, spawn_y = ROBOT_SPAWN[robot_name]

    # Este é o cálculo em dataset_collector.py linhas 85-86
    pose_x = odom_x + spawn_x
    pose_y = odom_y + spawn_y

    match_x = abs(pose_x - expected_x) < 0.001
    match_y = abs(pose_y - expected_y) < 0.001

    status = "✅" if (match_x and match_y) else "❌"
    print(f"{status} {desc}")
    print(f"   Odom: ({odom_x:.1f}, {odom_y:.1f})")
    print(f"   Offset spawn: {ROBOT_SPAWN[robot_name]}")
    print(f"   Resultado: ({pose_x:.1f}, {pose_y:.1f})")
    print(f"   Esperado: ({expected_x:.1f}, {expected_y:.1f})")
    print()

    if not (match_x and match_y):
        all_pass = False

if all_pass:
    print("✅ Todos os testes passaram! Lógica de odom+offset está correta.")
else:
    print("❌ Alguns testes falharam!")

print("\n" + "="*60)
print("CONCLUSÃO:")
print("="*60)
print("""
Se a lógica de odom+offset está correta (testes passam),
então o problema é em UMA DESSAS FASES:

1. O odômetro não está sendo lido corretamente
   - _odom_cb() está recebendo pose.pose.position errada?

2. A câmera está capturando pose antes do offset ser aplicado
   - Timing issue: pose lida ANTES do sync?

3. A projeção pixel→mundo está invertida
   - Os valores de poses invertidos (-0.6 vs +0.6, -1.3 vs +1.3)?

PRÓXIMO PASSO: Recolha dataset com prints de debug vendo:
- Valor raw de odom.pose.position
- Valor calculado de pose_x, pose_y
- Valor de cx_norm, cy_norm calculado
""")
