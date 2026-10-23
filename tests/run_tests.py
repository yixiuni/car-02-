#!/usr/bin/env python3
"""Portable host verification, no embedded toolchain required."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--compiler', default=shutil.which('clang') or shutil.which('gcc'))
p.add_argument('--sanitize', action='store_true')
a = p.parse_args()
if not a.compiler:
    p.error('Install a host C compiler or pass --compiler')
out = ROOT / 'build-host'
out.mkdir(exist_ok=True)
def run(name, sources, defines=()):
    exe = out / (name + ('.exe' if os.name == 'nt' else ''))
    cmd = [a.compiler, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-ICore/Inc']
    if a.sanitize:
        cmd += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    subprocess.run(cmd + list(defines) + sources + ['-lm', '-o', str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)
base = ['Core/Src/car_control.c', 'Core/Src/motion_control.c', 'Core/Src/imu_filter.c']
run('control', base + ['tests/test_car_control.c'])
for polarity in (0, 1):
    for reverse in (0, 1):
        run(f'gray-{polarity}-{reverse}', ['Core/Src/gray_protocol.c', 'tests/test_gray_protocol.c'],
            [f'-DCAR_GRAY_TARGET_BIT_IS_ONE={polarity}', f'-DCAR_GRAY_REVERSE_ORDER={reverse}'])
run('course', base + ['tests/sim_course.c'])
subprocess.run([os.sys.executable, 'tests/check_interfaces.py'], cwd=ROOT, check=True)
