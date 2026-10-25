#!/usr/bin/env python3
"""Host-only research variants; source copies stay under build-host.

These are independent designs informed by reference review, not copied drivers.
Reports include failures. Never generate or flash target firmware here.
"""
import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_time_trial as evaluator


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('variant', choices=['bounded', 'edge', 'middle', 'observer', 'projection', 'hold', 'continuity', 'qualified'])
    p.add_argument('--profiles', nargs='+')
    p.add_argument('--phases', nargs='+', default=['0', '2', '5', '7', '10', '12'])
    p.add_argument('--models', nargs='+', default=['1.5', '1.0', '0.9'])
    a = p.parse_args()
    root = evaluator.ROOT
    out = root / 'build-host/t317-reproduce' / a.variant
    out.mkdir(parents=True, exist_ok=True)
    source = out / 'car_control.c'
    original = out / 'baseline.c'
    original.write_bytes(subprocess.check_output([
        'git', 'show', '0764e3272f926dffaad7cb5cc00a08ed39b70293:Core/Src/car_control.c'], cwd=root))
    patch_variant = 'hold' if a.variant == 'qualified' else a.variant
    subprocess.run(['patch', '--silent', '-o', str(source),
                    str(original),
                    str(root / ('tools/research/t317-' + patch_variant + '.patch'))], check=True)
    evaluator.SOURCES[0] = str(source.relative_to(root))
    base = dict(evaluator.PROFILES['arc-step'])
    profiles = {}
    if a.variant == 'bounded':
        profiles['baseline'] = {**base, 'CAR_RESEARCH_D_BOUND': 0.0}
        for speed in (450.0, 500.0, 550.0):
            for bound in (0.5, 1.0, 1.5):
                profiles[f'arc{int(speed)}-bound{bound}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_D_BOUND': bound}
    elif a.variant == 'edge':
        profiles['edge0-arc450'] = {**base, 'CAR_RESEARCH_EDGE_ADVANCE': 0.0}
        for speed in (450.0, 500.0, 550.0):
            for gain in (0.5, 1.0):
                profiles[f'arc{int(speed)}-edge{gain}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_EDGE_ADVANCE': gain}
    elif a.variant == 'middle':
        for speed in (500.0, 550.0, 600.0):
            profiles[f'arc{int(speed)}-middle'] = {
                **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_MID_WINDOW': 1}
    elif a.variant == 'observer':
        for speed in (450.0, 500.0, 550.0):
            for gain in (0.5, 1.0):
                profiles[f'arc{int(speed)}-observer{gain}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_ENTRY_OBSERVER': 1,
                    'CAR_RESEARCH_ENTRY_GAIN': gain}
    elif a.variant == 'projection':
        for speed in (500.0, 550.0):
            for preview in (0, 1):
                profiles[f'arc{int(speed)}-projection-preview{preview}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_ENTRY_OBSERVER': 1,
                    'CAR_RESEARCH_ENTRY_GAIN': 1.0, 'CAR_CROSS_D_ENTRY_PREVIEW': preview}
    elif a.variant == 'hold':
        for speed in (450.0, 500.0, 550.0):
            for hold in (1, 2):
                profiles[f'arc{int(speed)}-hold{hold}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_ENTRY_HOLD': hold}
    elif a.variant == 'continuity':
        for speed in (500.0, 550.0):
            for hold in (45, 75):
                profiles[f'arc{int(speed)}-continuity{hold}'] = {
                    **base, 'CAR_TT_ARC_SPEED': speed, 'CAR_RESEARCH_ENTRY_HOLD': 2,
                    'CAR_RESEARCH_PREVIEW_HOLD_MS': hold}
    else:
        for arc, straight in ((450.,900.),(460.,800.),(470.,800.),(480.,800.),
                              (460.,900.),(470.,900.),(480.,900.)):
            profiles[f'arc{int(arc)}-straight{int(straight)}-hold'] = {
                **base, 'CAR_TT_ARC_SPEED': arc, 'CAR_TT_STRAIGHT_SPEED': straight,
                'CAR_RESEARCH_ENTRY_HOLD': 2}
    chosen = a.profiles or list(profiles)
    if any(name not in profiles for name in chosen):
        p.error('Unknown profile for variant: ' + ', '.join(chosen))
    evaluator.PROFILES = profiles
    sys.argv = [sys.argv[0], '--profiles', *chosen, '--models', *a.models,
                '--phases', *a.phases, '--stress', '--entry-stress',
                '--output', str(out / 'report.json')]
    evaluator.main()


if __name__ == '__main__':
    main()
