#!/usr/bin/env python3
"""Gráficos del punto 3.5: cuadrado con feedback de odometría vs feedback de EKF.
Uso: plot_35.py [carpeta_experimentos]  (usa runs/cuadrado_odom y runs/cuadrado_ekf)"""
import csv, math, sys, pathlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else pathlib.Path(__file__).parent.parent)
out = base / 'plots'; out.mkdir(exist_ok=True)
RUNS = [('cuadrado_odom', 'Feedback: odometría'), ('cuadrado_ekf', 'Feedback: EKF')]


def load(name):
    rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(base / 'runs' / name / 'poses.csv'))]
    path = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(base / 'runs' / name / 'poses_path.csv'))]
    return rows, path


def seg_dist(px, py, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]; L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / L))
    return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)


def cross_track(x, y, path):
    return min(seg_dist(x, y, path[i], path[i + 1]) for i in range(len(path) - 1))


data = {n: load(n) for n, _ in RUNS}
summary = []

# trayectorias lado a lado
fig, axs = plt.subplots(1, 2, figsize=(12, 6))
for ax, (n, title) in zip(axs, RUNS):
    rows, path = data[n]
    ax.plot(*zip(*path), color='black', lw=1.2, ls='--', label='Setpoint (cuadrado)')
    ax.plot([r['gt_x'] for r in rows], [r['gt_y'] for r in rows], color='tab:orange', label='Ground truth')
    ax.plot([r['odo_x'] for r in rows], [r['odo_y'] for r in rows], color='tab:green', label='Odometría')
    ax.plot([r['ekf_x'] for r in rows], [r['ekf_y'] for r in rows], color='tab:blue', lw=1, label='EKF')
    ax.set_title(title); ax.set_xlabel('x [m]'); ax.set_aspect('equal'); ax.set_xlim(-2.4, 0.4); ax.set_ylim(-0.4, 2.4); ax.grid(True)
axs[0].set_ylabel('y [m]'); axs[0].legend(loc='center')
fig.suptitle('Seguimiento del cuadrado a lazo cerrado')
fig.tight_layout(); fig.savefig(out / 'cuadrado_trayectorias.png', dpi=200); plt.close(fig)

# por corrida: cuadrado pedido vs recorrido real vs lo que creia la pose usada como feedback
BELIEF = {'cuadrado_odom': ('odo', 'Odometría (pose usada por el controlador)', 'tab:green'),
          'cuadrado_ekf': ('ekf', 'EKF (pose usada por el controlador)', 'tab:blue')}
for n, title in RUNS:
    rows, path = data[n]
    k, lab, col = BELIEF[n]
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(*zip(*path), color='black', lw=1.2, ls='--', label='Trayectoria pedida (cuadrado)')
    ax.plot([r['gt_x'] for r in rows], [r['gt_y'] for r in rows], color='tab:orange', label='Ground truth (recorrido real)')
    ax.plot([r[f'{k}_x'] for r in rows], [r[f'{k}_y'] for r in rows], color=col, label=lab)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title(f'Cuadrado a lazo cerrado ({title.lower()})')
    ax.set_aspect('equal'); ax.set_xlim(-2.4, 0.4); ax.set_ylim(-0.4, 2.4); ax.grid(True); ax.legend(loc='center')
    fig.tight_layout(); fig.savefig(out / f'{n}_trayectoria.png', dpi=200); plt.close(fig)

# error del robot real respecto del cuadrado en el tiempo
fig, ax = plt.subplots(figsize=(10, 4))
for n, title in RUNS:
    rows, path = data[n]
    ct = [cross_track(r['gt_x'], r['gt_y'], path) for r in rows]
    ax.plot([r['t'] for r in rows], [c * 100 for c in ct], label=title)
    eo = [math.hypot(r['odo_x'] - r['gt_x'], r['odo_y'] - r['gt_y']) for r in rows]
    ee = [math.hypot(r['ekf_x'] - r['gt_x'], r['ekf_y'] - r['gt_y']) for r in rows]
    summary.append((title, sum(ct) / len(ct), max(ct), eo[-1], sum(ee) / len(ee)))
ax.set_xlabel('t [s]'); ax.set_ylabel('distancia al cuadrado [cm]'); ax.grid(True); ax.legend()
ax.set_title('Distancia del robot real (ground truth) a la trayectoria pedida')
fig.tight_layout(); fig.savefig(out / 'cuadrado_error_seguimiento.png', dpi=200); plt.close(fig)

# componentes y velocidades de cada corrida
for n, title in RUNS:
    rows, path = data[n]
    t = [r['t'] for r in rows]
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, c, lab in zip(axs, ('x', 'y', 'th'), ('x [m]', 'y [m]', 'θ [rad]')):
        ax.plot(t, [r[f'goal_{c}'] for r in rows], color='black', ls='--', label='Setpoint (goal)')
        ax.plot(t, [r[f'gt_{c}'] for r in rows], color='tab:orange', label='Ground truth')
        ax.plot(t, [r[f'odo_{c}'] for r in rows], color='tab:green', label='Odometría')
        ax.plot(t, [r[f'ekf_{c}'] for r in rows], color='tab:blue', lw=1, label='EKF')
        ax.set_ylabel(lab); ax.grid(True)
    axs[0].legend(ncol=4); axs[-1].set_xlabel('t [s]'); axs[0].set_title(f'Pose en el tiempo ({title})')
    fig.tight_layout(); fig.savefig(out / f'{n}_componentes.png', dpi=200); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    for k, lab in (('cmd_vx', 'vx [m/s]'), ('cmd_vy', 'vy [m/s]'), ('cmd_w', 'ω [rad/s]')):
        ax.plot(t, [r[k] for r in rows], label=lab)
    ax.set_xlabel('t [s]'); ax.grid(True); ax.legend(); ax.set_title(f'Velocidades comandadas ({title})')
    fig.tight_layout(); fig.savefig(out / f'{n}_velocidades.png', dpi=200); plt.close(fig)

print('| Corrida | Real vs cuadrado medio [cm] | Real vs cuadrado máx [cm] | Odometría vs real final [cm] | EKF vs real medio [cm] |')
print('|---|---|---|---|---|')
for s in summary:
    print(f'| {s[0]} | {s[1]*100:.1f} | {s[2]*100:.1f} | {s[3]*100:.1f} | {s[4]*100:.1f} |')
