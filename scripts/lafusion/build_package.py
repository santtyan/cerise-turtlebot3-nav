#!/usr/bin/env python3
"""Resincroniza docs/lafusion/{code,scripts,bags/reproducibility_package/README.md}
a partir dos arquivos-fonte reais em src/cerise_nav/cerise_nav/, scripts/lafusion/
e bags/reproducibility_package/.

docs/lafusion/ é uma CÓPIA para consulta/empacotamento do paper (ver seu
README) — os arquivos-fonte editáveis continuam em scripts/, src/cerise_nav/,
bags/. Antes deste script, o resync era feito manualmente (cp) e foi fonte de
atrito documentada (esquecido em sessões anteriores). Rodar este script
sempre que qualquer arquivo-fonte listado abaixo for editado.

Uso: python3 scripts/lafusion/build_package.py
"""

import filecmp
import os
import shutil

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

CODE_FILES = [
    'association.py',
    'ekf_core.py',
    'ekf_fusion_node.py',
    'projection.py',
    'yolo_detector.py',
]

SCRIPT_STAGE_DIRS = [
    '0.setup',
    '1.validation',
    '2.evaluation',
    '3.figures',
]


def sync_file(src, dst):
    changed = not (os.path.exists(dst) and filecmp.cmp(src, dst, shallow=False))
    if changed:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
    return changed


def main():
    updated = []

    code_src_dir = os.path.join(_REPO, 'src', 'cerise_nav', 'cerise_nav')
    code_dst_dir = os.path.join(_REPO, 'docs', 'lafusion', 'code')
    for name in CODE_FILES:
        src = os.path.join(code_src_dir, name)
        dst = os.path.join(code_dst_dir, name)
        if sync_file(src, dst):
            updated.append(os.path.relpath(dst, _REPO))

    scripts_dst_dir = os.path.join(_REPO, 'docs', 'lafusion', 'scripts')
    for stage in SCRIPT_STAGE_DIRS:
        stage_dir = os.path.join(_REPO, 'scripts', 'lafusion', stage)
        for name in sorted(os.listdir(stage_dir)):
            if not name.endswith('.py'):
                continue
            src = os.path.join(stage_dir, name)
            dst = os.path.join(scripts_dst_dir, name)
            if sync_file(src, dst):
                updated.append(os.path.relpath(dst, _REPO))

    repro_src = os.path.join(_REPO, 'bags', 'reproducibility_package', 'README.md')
    repro_dst = os.path.join(_REPO, 'docs', 'lafusion', 'bags', 'reproducibility_package', 'README.md')
    if sync_file(repro_src, repro_dst):
        updated.append(os.path.relpath(repro_dst, _REPO))

    if updated:
        print(f'Atualizados ({len(updated)}):')
        for path in updated:
            print(f'  {path}')
    else:
        print('docs/lafusion/ já está sincronizada — nada a atualizar.')


if __name__ == '__main__':
    main()
