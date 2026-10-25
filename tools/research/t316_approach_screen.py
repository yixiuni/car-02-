#!/usr/bin/env python3
"""Reproduce the T315-informed approach screen; host only, never flash.

Candidates shorten the distance speed envelope while preserving entry speed.
No claim that a passing nominal simulation predicts physical performance.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_time_trial as evaluator


def main():
    base = dict(evaluator.PROFILES['arc-step'])
    names = ['arc-step']
    for decel in (1300.0, 1600.0, 1900.0):
        name = 'decel' + str(int(decel))
        evaluator.PROFILES[name] = {**base, 'CAR_TT_STRAIGHT_DECEL': decel}
        names.append(name)
    sys.argv = [sys.argv[0], '--profiles', *names,
                '--models', '1.5', '1.0', '0.9',
                '--phases', '0', '2', '5', '7', '10', '12',
                '--stress', '--entry-stress', '--output',
                str(evaluator.ROOT / 'build-host/t316-decel-screen/report.json')]
    evaluator.main()


if __name__ == '__main__':
    main()
