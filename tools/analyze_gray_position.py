#!/usr/bin/env python3
"""Replay read-only T312 captures through the C estimator. Never writes firmware.

Without --calibration, raw minima/maxima form exploratory references ONLY.
They have no surface labels or measured physical position and cannot authorize
steering. --calibration reuses a prior report's references on a new capture.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]
STATUS = ('OK', 'BAD_CALIBRATION', 'WHITE', 'WIDE', 'SPLIT')


def read_capture(path):
    text = path.read_text()
    rows = [list(map(lambda x: int(x, 0), line.split(',')[1:]))
            for line in text.splitlines() if line.startswith('GRAY_POSITION,')]
    counts = [int(line.split('=')[1]) for line in text.splitlines()
              if line.startswith('GRAY_POSITION_ACCEPTED=')]
    if not rows or counts != [len(rows)] or any(len(r) != 16 for r in rows):
        raise ValueError('Incomplete or malformed capture')
    unique = {}
    for row in rows:
        if any(row[5:8]) or ((row[1]-row[2]) & 0xffffffff) >= 100:
            raise ValueError('Receiver error or stale frame')
        if any(not 0 <= v <= 65535 for v in row[8:]):
            raise ValueError('Raw value outside uint16 range')
        old = unique.get(row[3])
        if old is not None and old[2:] != row[2:]:
            raise ValueError('One frame counter has inconsistent values')
        unique.setdefault(row[3], row)
    return list(unique.values()), len(rows)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--calibration', type=Path, help='Prior analysis JSON, not a firmware header')
    p.add_argument('--white-reference', type=Path, help='Labeled stationary white capture; replaces exploratory white minima')
    p.add_argument('--compiler', default=shutil.which('clang') or shutil.which('gcc'))
    a = p.parse_args()
    if not a.compiler:
        p.error('Host C compiler required')
    rows, count = read_capture(a.capture)
    if a.calibration:
        cal = json.loads(a.calibration.read_text())['candidate_calibration']
        origin = str(a.calibration)
    else:
        cal = dict(white=[min(r[8+i] for r in rows) for i in range(8)],
                   black=[max(r[8+i] for r in rows) for i in range(8)],
                   pitch_mm=15, reversed=False)
        origin = 'Unlabeled capture extrema; exploratory, not approved calibration'
    if a.white_reference:
        white_rows, _ = read_capture(a.white_reference)
        if len(white_rows) < 32:
            raise ValueError('Need at least 32 stationary white frames')
        cal['white'] = [statistics.median(r[8+i] for r in white_rows) for i in range(8)]
        for i in range(8):
            noise = max(r[8+i] for r in white_rows)-min(r[8+i] for r in white_rows)
            if cal['black'][i]-cal['white'][i] < max(128, 8*noise):
                raise ValueError('White noise or white-to-scan-peak contrast inadequate')
        origin = dict(black_reference=origin, white_reference=str(a.white_reference),
                      white_sha256=hashlib.sha256(a.white_reference.read_bytes()).hexdigest(),
                      method='white median and scan peak, trial only; dynamic validation pending')
    if len(cal['white']) != 8 or len(cal['black']) != 8:
        raise ValueError('Expected eight calibration channels')
    out = ROOT / 'build-host'
    out.mkdir(exist_ok=True)
    exe = out / 'gray-position-replay'
    subprocess.run([a.compiler, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-ICore/Inc',
                    'Core/Src/gray_position.c', 'tools/gray_position_replay.c', '-lm',
                    '-o', str(exe)], cwd=ROOT, check=True)
    header = [*cal['white'], *cal['black'], cal['pitch_mm'], int(cal['reversed'])]
    replay_input = '\n'.join(' '.join(map(str, r)) for r in [header, *[r[8:] for r in rows]])+'\n'
    result = subprocess.run([str(exe)], input=replay_input, text=True,
                            capture_output=True, check=True)
    estimates = [line.split(',') for line in result.stdout.splitlines()]
    if len(estimates) != len(rows):
        raise ValueError('Replay row count mismatch')
    statuses = Counter(STATUS[int(e[1])] for e in estimates)
    raw_masks = Counter(f'{255 ^ r[4]:02x}' for r in rows)
    report = dict(source=str(a.capture), source_sha256=hashlib.sha256(a.capture.read_bytes()).hexdigest(),
        accepted_rows=count, unique_frames=len(rows),
        duration_ms=(rows[-1][1]-rows[0][1]) & 0xffffffff,
        candidate_calibration=cal, calibration_origin=origin,
        status_counts=dict(statuses), binary_mask_counts=dict(raw_masks),
        channels=[dict(channel=i+1, minimum=min(r[8+i] for r in rows),
                       maximum=max(r[8+i] for r in rows)) for i in range(8)],
        estimator_sha256=hashlib.sha256((ROOT/'Core/Src/gray_position.c').read_bytes()).hexdigest(),
        physical_position_ground_truth=False, steering_validated=False, firmware_changed=False,
        note='Stationary dwell biases frame percentages. Unlabeled range coverage is not proof '
             'of spatial accuracy, monotonic motion, or faster successful laps.')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.with_suffix('.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['index','tick_ms','frame_ms','samples','raw_status','timeouts','errors',
                    'unexpected', *[f'ch{i}' for i in range(1,9)], 'valid','status',
                    'position_mm','strength','peak','spread_mm2','active_mask','clipped_mask'])
        for r,e in zip(rows,estimates):
            w.writerow([*r, e[0], STATUS[int(e[1])], *e[2:]])
    a.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: report[k] for k in ('unique_frames','duration_ms','status_counts',
                                           'steering_validated')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
