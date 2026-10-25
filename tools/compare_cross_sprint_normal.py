#!/usr/bin/env python3
"""Verify NORMAL control fingerprints against the archived T317 controller.

Baseline failures remain failures. This comparison proves host equivalence,
not physical timing, sensor validity or successful ground-truth coverage.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'research'))
import t318_screen as research
from evaluate_time_trial import ROOT,PROFILES,SOURCES

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--profile',choices=['cross-sprint','arc-localize','edge-pose','arc-phase'],default='cross-sprint')
a=p.parse_args();tag='t321' if a.profile=='arc-phase' else 't320' if a.profile=='edge-pose' else 't319' if a.profile=='arc-localize' else 't318'
out=ROOT/('build-host/'+tag+'-normal-equivalence');out.mkdir(parents=True,exist_ok=True)
for name,source,profile in [('t317',research.ensure_baseline(),'entry-hold'),
                            ('trial',ROOT/'Core/Src/car_control.c',a.profile)]:
    flags=[f'-D{k}={v}' for k,v in PROFILES[profile].items()]
    subprocess.run(['clang','-std=c11','-O1','-Wall','-Wextra','-Werror','-ICore/Inc',
                    '-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,
                    str(source),*SOURCES[1:],'-lm','-o',str(out/name)],cwd=ROOT,check=True)
records=[]
for gain in (1.5,1.,.9):
    for phase in range(15):
        env={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
        env.update(SIM_PWM_PER_MM_S=str(gain),SIM_GRAY_PHASE_MS=str(phase),
                   SIM_WEAK_BRAKE='1',SIM_CONTROL_FINGERPRINT='1')
        runs=[subprocess.run([str(out/name)],cwd=ROOT,env=env,capture_output=True,text=True)
              for name in ('t317','trial')]
        hashes=[[line for line in r.stdout.splitlines() if line.startswith('CONTROL_HASH task2 ')] for r in runs]
        assert hashes[0] and hashes[0]==hashes[1],(gain,phase)
        records.append(dict(gain=gain,phase=phase,equivalent=True,normal_hash_count=len(hashes[0]),
                            fingerprints_sha256=hashlib.sha256('\n'.join(hashes[0]).encode()).hexdigest(),
                            baseline_error=runs[0].stderr,trial_error=runs[1].stderr))
report=dict(scope=__doc__,all_normal_equivalent=True,cases=records)
(ROOT/('docs/data/'+tag+'-normal-equivalence-2026-10-25.json')).write_text(json.dumps(report,indent=2)+'\n')
print('NORMAL control fingerprints equivalent:',len(records),'/',len(records))
