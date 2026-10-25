#!/usr/bin/env python3
"""Independent synthetic encoder-scale stress on both arcs of all four laps.
This is not a measured slip estimate. True plant/coverage remain unchanged.
Requires the separately built T320 and T321 research model executables.
"""
import json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'build-host/t321-scale';OUT.mkdir(parents=True,exist_ok=True)
clean={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
rows=[]
for name,path in [('T320','build-host/t320-final/edge-pose'),('T321-production','build-host/t321-final/arc-phase') if '--production' in sys.argv else ('T321-research','build-host/t321-research/phase-prior')]:
    for gain in (1.5,1.,.9):
        for phase in (0,5,10):
            for scale in (.85,.9,.95,1.,1.05,1.1,1.15):
                env={**clean,'SIM_PWM_PER_MM_S':str(gain),'SIM_GRAY_PHASE_MS':str(phase),
                    'SIM_ARC_ODOMETRY_SCALE':str(scale),'SIM_NOMINAL_ONLY':'1',
                    'SIM_TASK4_ONLY':'1','SIM_WEAK_BRAKE':'1'}
                r=subprocess.run([str(ROOT/path)],cwd=ROOT,env=env,capture_output=True,text=True)
                rows.append(dict(profile=name,gain=gain,phase=phase,scale=scale,passed=r.returncode==0,stdout=r.stdout,errors=r.stderr))
    selected=[r for r in rows if r['profile']==name]
    print(name,sum(r['passed'] for r in selected),'/',len(selected),flush=True)
report=dict(scope=__doc__,runs=rows)
(ROOT/('docs/data/t321-scale-production-2026-10-25.json' if '--production' in sys.argv else 'docs/data/t321-scale-check-2026-10-25.json')).write_text(json.dumps(report,indent=2)+'\n')
