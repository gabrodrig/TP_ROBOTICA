#!/usr/bin/env python3
"""Analiza las corridas de runs/*/poses.csv.
Genera:
  - resumen.csv y resumen.md con las métricas de cada corrida
  - plots/<corrida>_*.png con trayectoria, componentes, error y covarianza
Uso: analyze.py [carpeta_experimentos]"""
import csv, math, sys, pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).parent.parent)
runs, plots = base / 'runs', base / 'plots'
plots.mkdir(exist_ok=True)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load(path):
    with open(path) as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    return [r for r in rows if not math.isnan(r['P_xx'])]


def metrics(rows):
    out = {}
    for src in ('odo', 'ekf'):
        ep = [math.hypot(r[f'{src}_x'] - r['gt_x'], r[f'{src}_y'] - r['gt_y']) for r in rows]
        et = [abs(wrap(r[f'{src}_th'] - r['gt_th'])) for r in rows]
        out[src] = dict(pos_mean=sum(ep) / len(ep), pos_max=max(ep), pos_final=ep[-1],
                        th_mean=sum(et) / len(et), th_max=max(et), th_final=et[-1])
    out['sigma_xy_mean'] = sum(math.sqrt(max(r['P_xx'], 0) + max(r['P_yy'], 0)) for r in rows) / len(rows)
    out['vis2'] = sum(r['n_postes'] >= 2 for r in rows) / len(rows)
    return out


def plot(name, rows):
    t = [r['t'] for r in rows]
    series = [('gt', 'Ground truth', 'tab:orange'), ('odo', 'Odometría', 'tab:green'), ('ekf', 'EKF', 'tab:blue')]

    # trayectoria
    fig, ax = plt.subplots(figsize=(6, 6))
    for k, lab, col in series:
        ax.plot([r[f'{k}_x'] for r in rows], [r[f'{k}_y'] for r in rows], label=lab, color=col)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title(f'Trayectoria ({name})')
    ax.axis('equal'); ax.grid(True); ax.legend()
    fig.tight_layout(); fig.savefig(plots / f'{name}_trayectoria.png', dpi=200); plt.close(fig)

    # componentes x, y, theta
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for ax, comp, lab in zip(axs, ('x', 'y', 'th'), ('x [m]', 'y [m]', 'θ [rad]')):
        for k, l, col in series:
            ax.plot(t, [r[f'{k}_{comp}'] for r in rows], label=l, color=col)
        ax.set_ylabel(lab); ax.grid(True)
    axs[0].legend(); axs[-1].set_xlabel('t [s]'); axs[0].set_title(f'Pose en el tiempo ({name})')
    fig.tight_layout(); fig.savefig(plots / f'{name}_componentes.png', dpi=200); plt.close(fig)

    # error contra ground truth + covarianza + postes visibles
    fig, axs = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for k, l, col in series[1:]:
        axs[0].plot(t, [math.hypot(r[f'{k}_x'] - r['gt_x'], r[f'{k}_y'] - r['gt_y']) for r in rows], label=l, color=col)
        axs[1].plot(t, [abs(wrap(r[f'{k}_th'] - r['gt_th'])) for r in rows], label=l, color=col)
    axs[0].set_ylabel('error posición [m]'); axs[1].set_ylabel('error θ [rad]')
    axs[2].plot(t, [math.sqrt(max(r['P_xx'], 0)) for r in rows], label='σx EKF [m]', color='tab:blue')
    axs[2].plot(t, [math.sqrt(max(r['P_thth'], 0)) for r in rows], label='σθ EKF [rad]', color='tab:purple')
    ax2 = axs[2].twinx(); ax2.step(t, [r['n_postes'] for r in rows], color='gray', alpha=0.4, where='post')
    ax2.set_ylabel('postes vistos', color='gray')
    for ax in axs: ax.grid(True)
    axs[0].legend(); axs[2].legend(loc='upper left'); axs[-1].set_xlabel('t [s]')
    axs[0].set_title(f'Error contra ground truth e incertidumbre ({name})')
    fig.tight_layout(); fig.savefig(plots / f'{name}_error.png', dpi=200); plt.close(fig)


results = []
for d in sorted(runs.iterdir()):
    f = d / 'poses.csv'
    if not f.exists():
        continue
    rows = load(f)
    if len(rows) < 10:
        print(f'{d.name}: pocas muestras, se ignora'); continue
    results.append((d.name, metrics(rows)))
    plot(d.name, rows)

cols = ['corrida', 'odo_pos_mean', 'odo_pos_max', 'ekf_pos_mean', 'ekf_pos_max',
        'odo_th_mean', 'ekf_th_mean', 'ekf_th_max', 'sigma_xy_mean', 'vis2']
with open(base / 'resumen.csv', 'w', newline='') as f:
    w = csv.writer(f); w.writerow(cols)
    for n, m in results:
        w.writerow([n, *[f'{v:.4f}' for v in (m['odo']['pos_mean'], m['odo']['pos_max'], m['ekf']['pos_mean'],
                    m['ekf']['pos_max'], m['odo']['th_mean'], m['ekf']['th_mean'], m['ekf']['th_max'],
                    m['sigma_xy_mean'], m['vis2'])]])

with open(base / 'resumen.md', 'w') as f:
    f.write('| Corrida | Odo err pos medio [cm] | EKF err pos medio [cm] | EKF err pos máx [cm] | '
            'Odo err θ medio [°] | EKF err θ medio [°] | EKF err θ máx [°] | σxy EKF medio [cm] | % tiempo ≥2 postes |\n')
    f.write('|' + '---|' * 9 + '\n')
    for n, m in results:
        o, e = m['odo'], m['ekf']
        f.write(f"| {n} | {o['pos_mean']*100:.1f} | {e['pos_mean']*100:.1f} | {e['pos_max']*100:.1f} | "
                f"{math.degrees(o['th_mean']):.2f} | {math.degrees(e['th_mean']):.2f} | {math.degrees(e['th_max']):.2f} | "
                f"{m['sigma_xy_mean']*100:.1f} | {m['vis2']*100:.0f} |\n")
print((base / 'resumen.md').read_text())
