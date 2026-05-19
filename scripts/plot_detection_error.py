#!/usr/bin/env python3
"""Gera histograma + CDF do detection_error a partir do log coletado."""

import sys
import re
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def load_errors(path: str) -> np.ndarray:
    values = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        try:
            values.append(float(line))
        except ValueError:
            pass
    return np.array(values)


def main():
    log_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/detection_error.txt'
    errors = load_errors(log_path)

    if len(errors) == 0:
        print("Nenhum dado encontrado no log.")
        sys.exit(1)

    print(f"Amostras: {len(errors)}")
    print(f"Média:    {errors.mean()*100:.1f} cm")
    print(f"Mediana:  {np.median(errors)*100:.1f} cm")
    print(f"P95:      {np.percentile(errors, 95)*100:.1f} cm")
    print(f"Max:      {errors.max()*100:.1f} cm")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    fig.suptitle('CERISE Digital Twin — Erro de Localização (YOLO vs Odometria)', fontsize=13)

    # Histograma
    ax1.hist(errors * 100, bins=30, color='steelblue', edgecolor='white', alpha=0.85)
    ax1.axvline(errors.mean() * 100, color='red', linestyle='--', label=f'Média = {errors.mean()*100:.1f} cm')
    ax1.axvline(np.median(errors) * 100, color='orange', linestyle='--', label=f'Mediana = {np.median(errors)*100:.1f} cm')
    ax1.set_xlabel('Erro (cm)')
    ax1.set_ylabel('Frequência')
    ax1.set_title('Histograma do Erro')
    ax1.legend()

    # CDF
    sorted_err = np.sort(errors * 100)
    cdf = np.arange(1, len(sorted_err) + 1) / len(sorted_err)
    ax2.plot(sorted_err, cdf, color='steelblue', linewidth=2)
    ax2.axvline(15, color='red', linestyle='--', alpha=0.7, label='Alvo: 15 cm')
    pct_below_15 = (errors < 0.15).mean() * 100
    ax2.axhline(pct_below_15 / 100, color='orange', linestyle=':', alpha=0.7,
                label=f'{pct_below_15:.0f}% < 15 cm')
    ax2.set_xlabel('Erro (cm)')
    ax2.set_ylabel('Prob. acumulada')
    ax2.set_title('CDF do Erro')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out = Path('docs') / 'digital_twin_error.png'
    out.parent.mkdir(exist_ok=True)
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f"\nGráfico salvo em: {out}")


if __name__ == '__main__':
    main()
