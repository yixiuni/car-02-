#!/usr/bin/env python3
"""Estimate a phase time budget from a saved physical trace, without device access.

Sparse samples assign each interval to its starting state. Distances use signed,
already-normalized encoder counts and the supplied calibration, not ground truth.
The hypothetical speeds are phase averages, never firmware cruise setpoints.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace',type=Path,default=ROOT/'docs/data/t312-cross-four-success-2026-10-25.csv')
    p.add_argument('--summary',type=Path,default=ROOT/'docs/data/t312-cross-four-success-2026-10-25.json')
    p.add_argument('--config',type=Path,default=ROOT/'Core/Inc/car_config.h')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    config=a.config.read_text()
    def constant(name):
        match=re.search(r'^#define\s+'+name+r'\s+(\d+(?:\.\d+)?)f?\b',config,re.M)
        if not match:raise ValueError('Expected explicit measured constant: '+name)
        return float(match[1])
    diameter=constant('CAR_WHEEL_DIAMETER_MM')
    counts_a=constant('CAR_ENCODER_A_COUNTS_REV');counts_b=constant('CAR_ENCODER_B_COUNTS_REV')
    metadata=json.loads(a.summary.read_text())
    rows=list(csv.DictReader(a.trace.open()))
    stop=next(i for i,r in enumerate(rows) if int(r['state'])==5)
    rows=rows[:stop+1]
    if any(int(r['fault']) for r in rows) or metadata['result']!='COMPLETED':
        raise ValueError('This budget requires a completed fault-free physical run')
    labels={1:'straight',3:'arc',4:'braking'}
    phases={name:dict(duration_s=0,distance_mm=0) for name in labels.values()}
    for before,after in zip(rows,rows[1:]):
        dt=((int(after['tick'])-int(before['tick']))&0xffffffff)/1000
        if not 0<dt<=1:raise ValueError('Invalid or excessively sparse timestamp gap')
        distance=math.pi*diameter*.5*((int(after['countsA'])-int(before['countsA']))/counts_a+
                    (int(after['countsB'])-int(before['countsB']))/counts_b)
        state=int(before['state'])
        if state not in labels:raise ValueError('Unsupported state in active trace')
        phase=phases[labels[state]];phase['duration_s']+=dt;phase['distance_mm']+=distance
    for state,name in labels.items():
        phase=phases[name];phase['mean_speed_mm_s']=phase['distance_mm']/phase['duration_s']
        if abs(phase['duration_s']-metadata['state_intervals_s'][str(state)])>.001:
            raise ValueError('Independent saved summary disagrees with phase durations')
    straight,arc,brake=(phases[n] for n in ('straight','arc','braking'))
    remaining=30-arc['duration_s']-brake['duration_s']
    report=dict(scope=__doc__,firmware=metadata['firmware'],physical_elapsed_s=metadata['elapsed_s'],
        calibration=dict(diameter_mm=diameter,counts_a_rev=counts_a,counts_b_rev=counts_b),
        phases=phases,
        straight_only_30s_required_mean_mm_s=straight['distance_mm']/remaining,
        ideal_straight_1000_with_unchanged_arc_s=straight['distance_mm']/1000+arc['duration_s']+brake['duration_s'],
        hypothetical_mean_straight850_arc500_s=straight['distance_mm']/850+arc['distance_mm']/500+brake['duration_s'],
        moving_mean_required_for_18s_mm_s=(straight['distance_mm']+arc['distance_mm'])/(18-brake['duration_s']),
        caveat='Same path and brake duration assumed. Sparse trace/encoder estimates; no tire-slip '
               'correction. These are time-budget calculations, not achievable speed predictions.',
        sources_sha256={str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
            hashlib.sha256(path.read_bytes()).hexdigest() for path in (a.trace,a.summary,a.config)})
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    for name,phase in phases.items():
        print(f"{name}: {phase['duration_s']:.3f} s, {phase['distance_mm']:.1f} mm, mean {phase['mean_speed_mm_s']:.1f} mm/s")
    print('Straight-only 30s requires mean',round(report['straight_only_30s_required_mean_mm_s'],1),'mm/s')
    print('Illustrative 850/500 phase means:',round(report['hypothetical_mean_straight850_arc500_s'],3),'s; not a prediction')


if __name__=='__main__':main()
