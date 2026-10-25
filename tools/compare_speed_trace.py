#!/usr/bin/env python3
"""Check every model control-tick fingerprint with/without T315 logging.

This proves host control equivalence, not hardware timing. Existing baseline
failures must be identical; they are reported rather than relabeled passes.
"""
import hashlib,json,os,shutil,subprocess
from pathlib import Path
from evaluate_time_trial import ROOT,PROFILES,SOURCES
out=ROOT/'build-host/speed-trace-equivalence';out.mkdir(parents=True,exist_ok=True)
cc=shutil.which('clang') or shutil.which('gcc')
for name,trace in [('baseline',0),('recording',1)]:
    flags=[f'-D{k}={v}' for k,v in PROFILES['arc-step'].items()]
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
        '-ICore/Inc','-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',
        f'-DCAR_SPEED_TRACE_BUILD={trace}',*flags,*SOURCES,'Core/Src/speed_trace.c','-lm',
        '-o',str(out/name)],cwd=ROOT,check=True)
records=[]
for phase in (0,2,5,7,10,12):
    for suite in ('nominal','full','d-offset','c-offset'):
        env={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
        env.update(SIM_GRAY_PHASE_MS=str(phase),SIM_WEAK_BRAKE='1',SIM_CONTROL_FINGERPRINT='1')
        key={'nominal':'SIM_NOMINAL_ONLY','d-offset':'SIM_D_ENTRY_SWEEP','c-offset':'SIM_C_ENTRY_SWEEP'}.get(suite)
        if key:env[key]='1'
        runs=[subprocess.run([str(out/n)],cwd=ROOT,env=env,capture_output=True,text=True) for n in ('baseline','recording')]
        same=runs[0].returncode==runs[1].returncode and runs[0].stdout==runs[1].stdout and runs[0].stderr==runs[1].stderr
        records.append(dict(phase=phase,suite=suite,equivalent=same,baseline_passed=runs[0].returncode==0,
            stderr=runs[0].stderr,control_hashes=runs[0].stdout.count('CONTROL_HASH'),
            stdout_sha256=hashlib.sha256(runs[0].stdout.encode()).hexdigest()))
report=dict(scope=__doc__,all_equivalent=all(r['equivalent'] for r in records),runs=records)
(ROOT/'docs/data/t315-control-equivalence-2026-10-25.json').write_text(json.dumps(report,indent=2)+'\n')
print('Equivalent:',sum(r['equivalent'] for r in records),'/',len(records),
      'baseline success:',sum(r['baseline_passed'] for r in records))
assert report['all_equivalent']
