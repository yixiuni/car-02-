#!/usr/bin/env python3
"""Screen edge-pose timing plus stricter C/D coverage; host only."""
import json,os,sys,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import t320_edge_pose as ep
OUT=ep.OUT/'margin';OUT.mkdir(parents=True,exist_ok=True)
base=(ep.OUT/'controller.c').read_text()
helper='''
static bool EntryCovered(const CarControl *c,Point end)
{
    if (!CrossSprint(c)) return Covers(c,end);
    float dx=end.x-c->x_mm,dy=end.y-c->y_mm;
    float forward=cosf(c->heading_rad)*dx+sinf(c->heading_rad)*dy;
    float lateral=-sinf(c->heading_rad)*dx+cosf(c->heading_rad)*dy;
    return forward<=c->geometry.front_mm-RESEARCH_FRONT_MARGIN &&
        forward>=-c->geometry.rear_mm && fabsf(lateral)<=c->geometry.half_width_mm-RESEARCH_SIDE_MARGIN;
}
'''
source=OUT/'controller.c';source.write_text(base.replace('static CarFault InputFault(',helper+'\nstatic CarFault InputFault(').replace('end_distance<CAR_ENDPOINT_GATE_MM && Covers(c,end) &&','end_distance<CAR_ENDPOINT_GATE_MM && EntryCovered(c,end) &&'))
flags=[f'-D{k}={v}' for k,v in ep.PROFILES['arc-localize'].items()]
clean={k:v for k,v in os.environ.items() if not k.startswith('SIM_')};records=[]
full='--full' in sys.argv
settings=[(float(sys.argv[sys.argv.index('--full')+1]),float(sys.argv[sys.argv.index('--full')+2]))] if full else [(30,30),(40,30),(40,40),(50,40)]
for front,side in settings:
    tag=f'f{front:g}-s{side:g}';exe=OUT/tag
    subprocess.run(['clang','-std=c11','-O1','-Wall','-Wextra','-Werror','-I'+str(ep.OUT/'include'),'-ICore/Inc',
        '-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,f'-DRESEARCH_FRONT_MARGIN={front}f' if isinstance(front,float) else f'-DRESEARCH_FRONT_MARGIN={front}',f'-DRESEARCH_SIDE_MARGIN={side}f' if isinstance(side,float) else f'-DRESEARCH_SIDE_MARGIN={side}',str(source),*ep.SOURCES[1:],'-lm','-o',str(exe)],cwd=ep.ROOT,check=True)
    for gain in ((1.5,1.,.9) if full else (1.,.9)):
        for phase in (range(15) if full else (0,3,7,9,11)):
            suites=[('cross-full',{'SIM_TASK4_ONLY':'1'}),('c-offset',{'SIM_C_ENTRY_SWEEP':'1'}),('d-offset',{'SIM_D_ENTRY_SWEEP':'1'})]
            if full:suites+=[('nominal',{'SIM_NOMINAL_ONLY':'1'}),('full',{})]
            for suite,extra in suites:
                env={**clean,'SIM_PWM_PER_MM_S':str(gain),'SIM_GRAY_PHASE_MS':str(phase),'SIM_WEAK_BRAKE':'1',**extra}
                r=subprocess.run([str(exe)],cwd=ep.ROOT,env=env,capture_output=True,text=True)
                log=f'{tag}-g{gain}-p{phase}-{suite}.log';(OUT/log).write_text(r.stdout+r.stderr)
                records.append(dict(front=front,side=side,gain=gain,phase=phase,suite=suite,passed=r.returncode==0,errors=r.stderr,log=log))
    rows=[r for r in records if r['front']==front and r['side']==side]
    print(tag,sum(r['passed'] for r in rows),'/',len(rows),flush=True)
(OUT/('full-report.json' if full else 'screen-coverage-report.json')).write_text(json.dumps(dict(scope=__doc__,runs=records),indent=2)+'\n')
