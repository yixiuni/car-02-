#!/usr/bin/env python3
"""Host-only staged feedback screen. Retains every failed scenario; no flashing."""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import evaluate_time_trial as ev

ROOT = ev.ROOT
OUT = ROOT / 'build-host/t318-research'
CONTROLLER = None

def ensure_baseline():
    OUT.mkdir(parents=True, exist_ok=True)
    baseline=OUT/'baseline.c'
    if not baseline.exists():
        original=OUT/'git-base.c'
        original.write_bytes(subprocess.check_output(['git','show',
            '0764e3272f926dffaad7cb5cc00a08ed39b70293:Core/Src/car_control.c'],cwd=ROOT))
        subprocess.run(['patch','--silent','-o',str(baseline),str(original),
                        str(ROOT/'tools/research/t318-baseline.patch')],check=True)
    assert hashlib.sha256(baseline.read_bytes()).hexdigest()=='9f5e461260dfcd7545b8ef238f28c1ac4c265a50ffd725da357ef7782a40ddbe'
    return baseline

def run_profile(item, phases, full):
    name, overrides = item
    definitions = {**ev.PROFILES['entry-hold'], **overrides}
    exe = OUT / name
    flags = [f'-D{k}={v}' + ('f' if isinstance(v, float) else '') for k, v in definitions.items()]
    sources = [str(CONTROLLER.relative_to(ROOT)), *ev.SOURCES[1:]]
    subprocess.run(['clang', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-ICore/Inc',
                    '-DCAR_TASK1_TEST_BUILD=1', '-DCAR_DUAL_MODE_BUILD=1', *flags,
                    *sources, '-lm', '-o', str(exe)], cwd=ROOT, check=True, capture_output=True)
    results = []
    env0 = {k: v for k, v in os.environ.items() if not k.startswith('SIM_')}
    for gain in (1.0, .9, 1.5):
        for phase in phases:
            for suite in ('nominal', 'full', 'd-offset', 'c-offset'):
                env = {**env0, 'SIM_PWM_PER_MM_S': str(gain), 'SIM_GRAY_PHASE_MS': str(phase),
                       'SIM_WEAK_BRAKE': '1', 'SIM_METRICS': '1'}
                key = {'nominal':'SIM_NOMINAL_ONLY','d-offset':'SIM_D_ENTRY_SWEEP','c-offset':'SIM_C_ENTRY_SWEEP'}.get(suite)
                if key: env[key] = '1'
                proc = subprocess.run([str(exe)], cwd=ROOT, env=env, capture_output=True, text=True)
                log = f'{name}-{gain}-{phase}-{suite}.log'
                (OUT/log).write_text(proc.stdout+proc.stderr)
                results.append(dict(gain=gain, phase=phase, suite=suite, passed=proc.returncode==0,
                                    errors=proc.stderr, log=log, cross_four_seconds=[float(x) for x in
                                    re.findall(r'PASS: task4 ([\d.]+)s, 17 point events, 4 laps', proc.stdout)]))
                if proc.returncode and not full:
                    return dict(name=name, overrides=overrides, runs=results, passed=False, early_exit=True)
    return dict(name=name, overrides=overrides, runs=results, passed=all(r['passed'] for r in results))

def main():
    global CONTROLLER
    OUT.mkdir(parents=True, exist_ok=True)
    base = ensure_baseline().read_text()
    old = 'return gain*CAR_LINE_KP*c->line_error+(derivative ? d_gain*CAR_LINE_KD*c->line_rate : 0);'
    assert base.count(old)==1
    new = '''bool cruise=FastCourse(c) && c->state==CAR_ARC &&
        c->distance_mm-c->segment_distance>=c->geometry.sensor_forward_mm;
    float pscale=cruise ? RESEARCH_P : 1.0f;
    float dscale=cruise ? RESEARCH_D : 1.0f;
    return pscale*gain*CAR_LINE_KP*c->line_error+
        (derivative ? dscale*d_gain*CAR_LINE_KD*c->line_rate : 0);'''
    revised=base.replace(old,new)
    revised='''#ifndef RESEARCH_CROSS_ONLY
#define RESEARCH_CROSS_ONLY 0
#endif
#ifndef RESEARCH_LOOKAHEAD
#define RESEARCH_LOOKAHEAD 250.0f
#endif
#ifndef RESEARCH_C_LINE_GATE
#define RESEARCH_C_LINE_GATE 1000.0f
#endif
#ifndef RESEARCH_C_GATE_ONE_SIDED
#define RESEARCH_C_GATE_ONE_SIDED 0
#endif
#ifndef RESEARCH_D_LINE_GATE
#define RESEARCH_D_LINE_GATE 1000.0f
#endif
'''+revised
    revised=revised.replace('atan2f(lateral,250.0f)', 'atan2f(lateral,RESEARCH_LOOKAHEAD)')
    revised=revised.replace('atan2f(aim_lateral,250.0f)', 'atan2f(aim_lateral,RESEARCH_LOOKAHEAD)')
    revised=revised.replace('static CarFault InputFault', '''#ifndef RESEARCH_MARGIN
#define RESEARCH_MARGIN 0.0f
#endif
#ifndef RESEARCH_C_MARGIN
#define RESEARCH_C_MARGIN RESEARCH_MARGIN
#endif
static bool EntryCovers(const CarControl *c,Point p)
{
    float dx=p.x-c->x_mm,dy=p.y-c->y_mm;
    float f=cosf(c->heading_rad)*dx+sinf(c->heading_rad)*dy;
    float l=-sinf(c->heading_rad)*dx+cosf(c->heading_rad)*dy;
    float margin=c->task==4 && c->segment==0 ? RESEARCH_C_MARGIN : RESEARCH_MARGIN;
    return f<=c->geometry.front_mm-margin &&
        f>=-c->geometry.rear_mm+margin &&
        fabsf(l)<=c->geometry.half_width_mm-margin;
}
static CarFault InputFault''')
    revised=revised.replace('end_distance<CAR_ENDPOINT_GATE_MM && Covers(c,end) &&',
                            'end_distance<CAR_ENDPOINT_GATE_MM && EntryCovers(c,end) &&')
    revised=revised.replace('c->black_frames>=CAR_LINE_CONFIRM_FRAMES;','c->black_frames>=CAR_LINE_CONFIRM_FRAMES &&\n            (c->task!=4 || c->segment!=0 || (RESEARCH_C_GATE_ONE_SIDED ? c->line_error : fabsf(c->line_error))<=RESEARCH_C_LINE_GATE) &&\n            (c->task!=4 || c->segment!=2 || c->line_error>=-RESEARCH_D_LINE_GATE);')
    revised=revised.replace('bool cruise=FastCourse(c)', 'bool cruise=(!RESEARCH_CROSS_ONLY || c->task==4) && FastCourse(c)')
    revised=revised.replace('static float FastArcSpeed(void)', 'static float FastArcSpeed(const CarControl *c)')
    revised=revised.replace('return CAR_TIME_TRIAL_BUILD ? CAR_TT_ARC_SPEED : CAR_FOUR_CRUISE_SPEED;',
        'return RESEARCH_CROSS_ONLY && c->task==2 ? 460.0f : CAR_TIME_TRIAL_BUILD ? CAR_TT_ARC_SPEED : CAR_FOUR_CRUISE_SPEED;')
    revised=revised.replace('FastArcSpeed()', 'FastArcSpeed(c)')
    revised=revised.replace('aligned*(CAR_TT_STRAIGHT_SPEED-CAR_TT_ENTRY_SPEED)',
        'aligned*((RESEARCH_CROSS_ONLY && c->task==2 ? 800.0f : CAR_TT_STRAIGHT_SPEED)-CAR_TT_ENTRY_SPEED)')
    revised=revised.replace('c->state==CAR_ARC ? CAR_TT_ARC_PWM_RESERVE : 0;',
        'c->state==CAR_ARC && (!RESEARCH_CROSS_ONLY || c->task==4) ? CAR_TT_ARC_PWM_RESERVE : 0;')
    revised=revised.replace('atan2f(lateral,RESEARCH_LOOKAHEAD)',
        'atan2f(lateral,RESEARCH_CROSS_ONLY && c->task==2 ? 250.0f : RESEARCH_LOOKAHEAD)')
    revised=revised.replace('float f=cosf(c->heading_rad)*dx+sinf(c->heading_rad)*dy;',
        'if(RESEARCH_CROSS_ONLY && c->task==2) return Covers(c,p);\n    float f=cosf(c->heading_rad)*dx+sinf(c->heading_rad)*dy;')
    mode=sys.argv[1] if len(sys.argv)>1 else 'screen'
    CONTROLLER=OUT/(mode+'-controller.c')
    CONTROLLER.write_text(revised)
    if mode=='screen':
        profiles={f'a{arc}-s{straight}-p{p}-d{d}':
                  dict(CAR_TT_ARC_SPEED=float(arc),CAR_TT_STRAIGHT_SPEED=float(straight),
                       RESEARCH_P=p,RESEARCH_D=d)
                  for arc,straight in ((600,950),(550,900),(500,900))
                  for p in (.5,.75,1.) for d in (0.,.25,.5,1.)}
        phases=[0];full=False
    elif mode=='headroom':
        profiles={f'h-a{arc}-s{straight}-p{p}-d{d}-e{delay}':
                  dict(CAR_TT_ARC_SPEED=float(arc),CAR_TT_STRAIGHT_SPEED=float(straight),
                       CAR_TT_ARC_PWM_RESERVE=900.0,CAR_TT_ARC_ACCEL_START_MM=float(delay),
                       RESEARCH_P=p,RESEARCH_D=d)
                  for arc,straight in ((650,1000),(600,950),(550,950))
                  for p in (.5,.75,1.) for d in (0.,.25) for delay in (0,165)}
        phases=[0];full=False
    elif mode=='margin':
        profiles={f'm-a{arc}-s{straight}-d{d}-m{margin}':
                  dict(CAR_TT_ARC_SPEED=float(arc),CAR_TT_STRAIGHT_SPEED=float(straight),
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=d,
                       RESEARCH_MARGIN=float(margin))
                  for arc,straight in ((650,1000),(600,950),(550,950),(500,900))
                  for d in (0.,.25) for margin in (5,10,15)}
        phases=[0];full=False
    elif mode=='path':
        profiles={f'path-a{arc}-s{straight}-l{look}-aim{aim}':
                  dict(CAR_TT_ARC_SPEED=float(arc),CAR_TT_STRAIGHT_SPEED=float(straight),
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_LOOKAHEAD=float(look),CAR_CROSS_C_ENTRY_AIM_MM=float(aim),
                       CAR_CROSS_D_ENTRY_AIM_MM=float(aim))
                  for arc,straight in ((650,1000),(600,950),(550,950))
                  for look in (300,350,400) for aim in (20,30)}
        phases=[0];full=False
    elif mode=='cross':
        profiles={f'cross-a{arc}-s{straight}-l{look}-aim{aim}':
                  dict(CAR_TT_ARC_SPEED=float(arc),CAR_TT_STRAIGHT_SPEED=float(straight),
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1, RESEARCH_LOOKAHEAD=float(look),
                       CAR_CROSS_C_ENTRY_AIM_MM=float(aim),CAR_CROSS_D_ENTRY_AIM_MM=float(aim))
                  for arc,straight in ((650,1000),(600,950),(550,950))
                  for look in (250,300,350) for aim in (20,30)}
        phases=[0];full=False
    elif mode=='cross-margin':
        profiles={f'cm-a650-s1000-c{aim}-m{margin}':
                  dict(CAR_TT_ARC_SPEED=650.0,CAR_TT_STRAIGHT_SPEED=1000.0,
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1,RESEARCH_LOOKAHEAD=350.0,
                       CAR_CROSS_C_ENTRY_AIM_MM=float(aim),CAR_CROSS_D_ENTRY_AIM_MM=30.0,
                       RESEARCH_C_MARGIN=float(margin))
                  for aim in (25,30,35,40) for margin in (5,10,15)}
        phases=[0];full=False
    elif mode=='center-gate':
        profiles={f'cg-a650-s1000-g{gate}-m{margin}':
                  dict(CAR_TT_ARC_SPEED=650.0,CAR_TT_STRAIGHT_SPEED=1000.0,
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1,RESEARCH_LOOKAHEAD=350.0,
                       CAR_CROSS_C_ENTRY_AIM_MM=30.0,CAR_CROSS_D_ENTRY_AIM_MM=30.0,
                       RESEARCH_C_LINE_GATE=gate,RESEARCH_C_MARGIN=float(margin))
                  for gate in (22.5,37.5) for margin in (0,5,10)}
        phases=[0,2,5,7,10,12];full=True
    elif mode=='signed-gate':
        profiles={f'sg-a650-s1000-c{aim}-g{gate}':
                  dict(CAR_TT_ARC_SPEED=650.0,CAR_TT_STRAIGHT_SPEED=1000.0,
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1,RESEARCH_LOOKAHEAD=350.0,
                       CAR_CROSS_C_ENTRY_AIM_MM=float(aim),CAR_CROSS_D_ENTRY_AIM_MM=30.0,
                       RESEARCH_C_LINE_GATE=gate,RESEARCH_C_GATE_ONE_SIDED=1)
                  for aim in (25,30,35) for gate in (22.5,37.5)}
        phases=[0,2,5,7,10,12];full=True
    elif mode=='held-out':
        profiles={f'heldout-g{gate}':
                  dict(CAR_TT_ARC_SPEED=650.0,CAR_TT_STRAIGHT_SPEED=1000.0,
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1,RESEARCH_LOOKAHEAD=350.0,
                       CAR_CROSS_C_ENTRY_AIM_MM=30.0,CAR_CROSS_D_ENTRY_AIM_MM=30.0,
                       RESEARCH_C_LINE_GATE=gate,RESEARCH_C_GATE_ONE_SIDED=1)
                  for gate in (22.5,37.5)}
        phases=[1,3,4,6,8,9,11,13,14];full=True
    elif mode=='d-preview':
        profiles={f'dp-a650-d{aim}-g{gate}':
                  dict(CAR_TT_ARC_SPEED=650.0,CAR_TT_STRAIGHT_SPEED=1000.0,
                       CAR_TT_ARC_PWM_RESERVE=900.0, RESEARCH_P=.5,RESEARCH_D=.25,
                       RESEARCH_CROSS_ONLY=1,RESEARCH_LOOKAHEAD=350.0,
                       CAR_CROSS_C_ENTRY_AIM_MM=30.0,CAR_CROSS_D_ENTRY_AIM_MM=float(aim),
                       CAR_CROSS_D_ENTRY_PREVIEW=1,RESEARCH_D_LINE_GATE=gate,
                       RESEARCH_C_LINE_GATE=22.5,RESEARCH_C_GATE_ONE_SIDED=1)
                  for aim in (25,30,35,40) for gate in (22.5,37.5,1000.)}
        phases=[0,4,9];full=False
    else:
        previous=mode.removesuffix('-qualify') if mode.endswith('-qualify') else 'screen'
        prev=json.loads((OUT/(previous+'.json')).read_text())
        profiles={r['name']:r['overrides'] for r in prev['profiles'] if r['passed']}
        phases=list(range(15)) if previous=='d-preview' else [0,2,5,7,10,12];full=True
    result=dict(scope='Host simulation with assumed plant; not physical performance.',
                source_sha256=hashlib.sha256(CONTROLLER.read_bytes()).hexdigest(),profiles=[])
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        futures=[pool.submit(run_profile,item,phases,full) for item in profiles.items()]
        for future in concurrent.futures.as_completed(futures):
            r=future.result();result['profiles'].append(r)
            nominal=[t for a in r['runs'] if a['suite']=='nominal' for t in a['cross_four_seconds']]
            print(r['name'],r['passed'],len(r['runs']),nominal,flush=True)
            (OUT/f'{mode}.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
