#!/usr/bin/env python3
"""Reescribe Q y R en localizer_ekf.cpp multiplicando los valores originales por un factor.
Uso: set_qr.py <factor_Q> <factor_R>   (1 1 = valores originales)"""
import re, sys, pathlib
tools = pathlib.Path(__file__).parent
src = pathlib.Path('/root/ros2_ws/src/robotica/robmovil_ekf/src/localizer_ekf.cpp')
s = (tools / 'localizer_ekf.cpp.orig').read_text()
fq, fr = float(sys.argv[1]), float(sys.argv[2])
base = {'Q(1,1)': 0.001, 'Q(2,2)': 0.001, 'Q(3,3)': 0.02 ** 2, 'R(1,1)': 0.001, 'R(2,2)': 0.00007615435}
for k, v in base.items():
    f = fq if k[0] == 'Q' else fr
    pat = re.escape(k) + r'\s*=\s*[^;]+;'
    s, n = re.subn(pat, f'{k} = {v * f:.10g};', s, count=1)
    assert n == 1, k
src.write_text(s)
print(f'Q x{fq:g}, R x{fr:g}')
