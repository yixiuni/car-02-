#!/usr/bin/env python3
"""Compare every simulated control tick with and without exit diagnostics."""
import hashlib,json,os,shutil,subprocess
from pathlib import Path
from evaluate_time_trial import ROOT,PROFILES,SOURCES
out=ROOT/'build-host/time-trial/t311-equivalence';out.mkdir(parents=True,exist_ok=True)
compiler=shutil.which('clang') or shutil.which('gcc')
for name in ('adaptive','exit-trace'):
    flags=[f'-D{k}={v}' for k,v in PROFILES[name].items()]
    subprocess.run([compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror','-ICore/Inc','-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,*SOURCES,'-lm','-o',str(out/name)],cwd=ROOT,check=True)
records=[]
for model in (1.5,1.0,0.9):
    for phase in (0,5,10):
        for suite in ('nominal','full','d-offset','c-offset'):
            env={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
            env.update(SIM_PWM_PER_MM_S=str(model),SIM_GRAY_PHASE_MS=str(phase),SIM_WEAK_BRAKE='1',SIM_CONTROL_FINGERPRINT='1')
            key={'nominal':'SIM_NOMINAL_ONLY','d-offset':'SIM_D_ENTRY_SWEEP','c-offset':'SIM_C_ENTRY_SWEEP'}.get(suite)
            if key:env[key]='1'
            runs=[subprocess.run([str(out/name)],env=env,cwd=ROOT,capture_output=True,text=True) for name in ('adaptive','exit-trace')]
            same=all(r.returncode==0 for r in runs) and runs[0].stdout==runs[1].stdout and runs[0].stderr==runs[1].stderr
            for name,r in zip(('adaptive','exit-trace'),runs):
                (out/f'{name}-{model}-{phase}-{suite}.log').write_text(r.stdout+r.stderr)
            records.append(dict(model=model,phase=phase,suite=suite,passed=same,control_hashes=runs[0].stdout.count('CONTROL_HASH'),stdout_sha256=hashlib.sha256(runs[0].stdout.encode()).hexdigest()))
report={'scope':'Host model only; compares selected control state and outputs at every tick, not physical timing or stability.','passed':all(r['passed'] for r in records),'runs':records}
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(f"Control equivalence: {sum(r['passed'] for r in records)}/{len(records)} suites")
assert report['passed']
