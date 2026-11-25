#!/usr/bin/env python3
"""Gráfico del punto 3.3: error real del EKF vs incertidumbre que informa (σ) para cada configuración de Q/R.
Lee resumen.csv. Uso: plot_qr.py [carpeta_experimentos]"""
import csv, sys, pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).parent.parent)
rows = {r['corrida']: r for r in csv.DictReader(open(base / 'resumen.csv'))}
CONFIGS = ['qdiv10', 'base', 'q10', 'q100', 'rdiv10', 'r10', 'r100']
MOVES = [('circulo', 'Círculo'), ('orbita', 'Órbita lateral'), ('rotrapida', 'Rotación rápida')]

fig, axs = plt.subplots(1, len(MOVES), figsize=(14, 4.6), sharey=True)
for ax, (m, title) in zip(axs, MOVES):
    err = [float(rows[f'{m}_{c}']['ekf_pos_mean']) * 100 for c in CONFIGS]
    sig = [float(rows[f'{m}_{c}']['sigma_xy_mean']) * 100 for c in CONFIGS]
    x = range(len(CONFIGS))
    ax.bar([i - 0.2 for i in x], err, width=0.4, label='Error real del EKF', color='tab:blue')
    ax.bar([i + 0.2 for i in x], sig, width=0.4, label='σ que informa el EKF', color='tab:orange')
    ax.set_xticks(list(x)); ax.set_xticklabels(CONFIGS, rotation=30)
    ax.set_yscale('log'); ax.set_title(title); ax.grid(True, axis='y', which='both', alpha=.4)
    ax.axvspan(-0.5, 0.5, color='tab:green', alpha=.08)
axs[0].set_ylabel('cm (escala logarítmica)'); axs[0].legend(loc='upper left')
fig.suptitle('Precisión vs incertidumbre informada según Q y R (sombreado: configuración elegida)')
fig.tight_layout(); fig.savefig(base / 'plots' / 'qr_error_vs_sigma.png', dpi=200)
print('ok')
