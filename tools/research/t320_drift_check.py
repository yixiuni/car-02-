#!/usr/bin/env python3
"""Synthetic arc estimator drift, independent physical plant/coverage unchanged.
This is a controlled failure-mode test, not an exact replay of the real run.
"""
import hashlib,json,os,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluate_time_trial import ROOT,PROFILES,SOURCES
out=ROOT/'build-host/t320-drift';out.mkdir(parents=True,exist_ok=True)
records=[]
profiles={name:PROFILES[name] for name in ('arc-localize','edge-pose')}
if '--screen' in sys.argv:
    profiles.update({f'g{g}-s{s}':{**PROFILES['arc-localize'],'CAR_ARC_POSITION_GAIN':g,'CAR_ARC_POSITION_STEP_MM':s}
                     for g,s in ((.2,4.),(.35,6.),(.5,8.))})
for profile,definitions in profiles.items():
    exe=out/profile
    flags=[f'-D{k}={v}'+('f' if isinstance(v,float) else '') for k,v in definitions.items()]
    subprocess.run(['clang','-std=c11','-O1','-Wall','-Wextra','-Werror','-ICore/Inc',
        '-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,*SOURCES,'-lm','-o',str(exe)],cwd=ROOT,check=True)
    for gain in (1.5,1.,.9):
        for phase in (0,2,5,7,10,12):
            for drift in (-130,-80,0,80,130):
                env={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
                env.update(SIM_PWM_PER_MM_S=str(gain),SIM_GRAY_PHASE_MS=str(phase),SIM_ARC_POSE_DRIFT_MM=str(drift),
                           SIM_NOMINAL_ONLY='1',SIM_TASK4_ONLY='1',SIM_WEAK_BRAKE='1')
                proc=subprocess.run([str(exe)],cwd=ROOT,env=env,capture_output=True,text=True)
                records.append(dict(profile=profile,gain=gain,phase=phase,drift_mm=drift,
                                    passed=proc.returncode==0,stdout=proc.stdout,errors=proc.stderr))
    rows=[r for r in records if r['profile']==profile]
    print(profile,sum(r['passed'] for r in rows),'/',len(rows),flush=True)
report=dict(scope=__doc__,runs=records,inputs_sha256={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest()
            for s in [*SOURCES,'Core/Inc/car_config.h']})
name='t320-drift-screen-2026-10-25.json' if '--screen' in sys.argv else 't320-drift-check-2026-10-25.json'
(ROOT/'docs/data'/name).write_text(json.dumps(report,indent=2)+'\n')
