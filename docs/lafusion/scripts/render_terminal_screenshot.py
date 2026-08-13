#!/usr/bin/env python3
"""Renderiza a saída real de validate_ekf_synthetic.py como imagem estilo
terminal, para uso como figura no paper (evidência de que a validação foi
de fato executada, complementar ao gráfico de plot_nees_nis.py).

Uso: python3 scripts/render_terminal_screenshot.py
Saída: docs/lafusion_validation_terminal.png
"""

import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(_REPO, 'docs', 'lafusion_validation_terminal.png')

# Paleta estilo terminal escuro (macOS Terminal.app / iTerm2 "Pro" theme)
BG = (30, 30, 30)
TITLEBAR_BG = (50, 50, 50)
FG = (220, 220, 220)
FG_DIM = (150, 150, 150)
FG_GREEN = (100, 220, 130)
FG_RED = (230, 100, 100)
FG_YELLOW = (230, 200, 100)
DOT_RED, DOT_YELLOW, DOT_GREEN = (255, 95, 86), (255, 189, 44), (39, 201, 63)

FONT_REGULAR = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf',
    '/usr/share/fonts/truetype/liberation2/LiberationMono-Regular.ttf',
]
FONT_BOLD = [
    '/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf',
    '/usr/share/fonts/truetype/liberation2/LiberationMono-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf',
]


def find_font(size, bold=False):
    candidates = FONT_BOLD + FONT_REGULAR if bold else FONT_REGULAR
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def colorize_line(line):
    """Aplica cor por palavra-chave, imitando o que o usuário veria com
    print colorido/grep --color, mesmo que o script não use cores reais."""
    if 'NÃO' in line or 'INCONSISTENTE' in line:
        return FG_RED
    if 'SIM' in line or 'consistente' in line.lower() and 'IN' not in line:
        return FG_GREEN
    if line.startswith('===') or line.startswith('---'):
        return FG_DIM
    if 'RESULTADO' in line:
        return FG_YELLOW
    return FG


def main():
    result = subprocess.run(
        [sys.executable, os.path.join(_REPO, 'scripts', 'validate_ekf_synthetic.py')],
        capture_output=True, text=True, cwd=_REPO)
    output = result.stdout

    font_size = 15
    font = find_font(font_size)
    font_bold = find_font(font_size, bold=True)
    line_height = int(font_size * 1.55)

    lines = output.rstrip('\n').split('\n')
    padding_x, padding_top = 20, 46
    titlebar_h = 34

    # Largura: medir a linha mais longa
    tmp_img = Image.new('RGB', (10, 10))
    tmp_draw = ImageDraw.Draw(tmp_img)
    max_w = max(tmp_draw.textlength(line, font=font) for line in lines if line)
    width = int(max_w) + 2 * padding_x
    height = titlebar_h + padding_top + len(lines) * line_height + 20

    img = Image.new('RGB', (width, height), BG)
    draw = ImageDraw.Draw(img)

    # Barra de título estilo macOS
    draw.rectangle([0, 0, width, titlebar_h], fill=TITLEBAR_BG)
    for i, color in enumerate([DOT_RED, DOT_YELLOW, DOT_GREEN]):
        cx = 18 + i * 22
        draw.ellipse([cx - 6, titlebar_h // 2 - 6, cx + 6, titlebar_h // 2 + 6], fill=color)
    title_font = find_font(13)
    title_text = 'python3 scripts/validate_ekf_synthetic.py'
    tw = draw.textlength(title_text, font=title_font)
    draw.text(((width - tw) / 2, titlebar_h // 2 - 8), title_text, font=title_font, fill=FG_DIM)

    # Prompt + comando
    y = titlebar_h + 16
    prompt = '$ python3 scripts/validate_ekf_synthetic.py'
    draw.text((padding_x, y), prompt, font=font_bold, fill=FG_GREEN)
    y += line_height

    for line in lines:
        color = colorize_line(line)
        draw.text((padding_x, y), line, font=font, fill=color)
        y += line_height

    img.save(OUT_PATH)
    print(f'Salvo: {OUT_PATH}')


if __name__ == '__main__':
    main()
