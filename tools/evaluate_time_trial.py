#!/usr/bin/env python3
"""Host-only speed-planner experiments. Never builds/flashes target firmware.

Each motor response is an explicit assumption, not a measured high-speed fit.
Failures remain in the report; a nominal success does not qualify a profile.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PROFILES={
    'dul5': {},
    'stage1': {'CAR_TIME_TRIAL_BUILD':1},
    'stage2': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1},
    'stage3': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE3':1},
    'straight520': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TT_STRAIGHT_SPEED':520.0},
    'straight540': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TT_STRAIGHT_SPEED':540.0},
    'straight550': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TT_STRAIGHT_SPEED':550.0},
    'c-aim10': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':10.0},
    'c-aim20': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':20.0},
    'c-aim30': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':30.0},
    'c-aim40': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':40.0},
    'c-aim15': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':15.0},
    'c-aim22': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':22.0},
    'c-aim25': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':25.0},
    'c-aim28': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_AIM_MM':28.0},
    'c-preview0': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_PREVIEW':1,'CAR_CROSS_C_ENTRY_AIM_MM':0.0},
    'c-preview20': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_PREVIEW':1,'CAR_CROSS_C_ENTRY_AIM_MM':20.0},
    'c-preview30': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_CROSS_C_ENTRY_PREVIEW':1,'CAR_CROSS_C_ENTRY_AIM_MM':30.0},
    'entry': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1},
    'entry520': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':520.0},
    'entry540': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':540.0},
    'entry550': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':550.0},
    'entry580': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':580.0},
    'entry600': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':600.0},
    'entry530': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':530.0},
    'entry535': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TT_STRAIGHT_SPEED':535.0},
    'straight': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TIME_TRIAL_STRAIGHT_BOOST':1},
    'fast': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TIME_TRIAL_STAGE2':1,'CAR_TIME_TRIAL_C_ENTRY':1,'CAR_TIME_TRIAL_FAST':1},
    'target30': {'CAR_TIME_TRIAL_BUILD':1,'CAR_TT_STRAIGHT_SPEED':800.0,
        'CAR_TT_ARC_SPEED':650.0,'CAR_TT_ENTRY_SPEED':450.0,
        'CAR_TT_WHEEL_SPEED':1000.0,'CAR_TT_D_FILTER_MS':25.0,'CAR_FOUR_PWM_LIMIT':900},
}
SOURCES=['Core/Src/car_control.c','Core/Src/motion_control.c','Core/Src/imu_filter.c',
         'Core/Src/task1_test.c','Core/Src/course_mode.c','tests/sim_course.c']

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profiles',nargs='+',choices=PROFILES,default=list(PROFILES))
    parser.add_argument('--models',nargs='+',type=float,default=[1.5,1.0,0.9],
                        help='Assumed PWM permille per wheel mm/s; default plant stays 1.5')
    parser.add_argument('--stress',action='store_true',help='Also run placement, unequal motors, faults and D-offset suites')
    parser.add_argument('--entry-stress',action='store_true',help='Also sweep assumed C-entry position errors for all four laps')
    parser.add_argument('--compiler',default=shutil.which('clang') or shutil.which('gcc'))
    parser.add_argument('--output',type=Path,default=ROOT/'build-host/time-trial/report.json')
    args=parser.parse_args()
    if not args.compiler:parser.error('Host C compiler required')
    if not all(0.5<=m<=3 for m in args.models):parser.error('Model must be in [0.5,3]')
    args.output=args.output.resolve();out=args.output.parent;out.mkdir(parents=True,exist_ok=True)
    inputs=SOURCES+['Core/Inc/car_config.h','Core/Inc/car_control.h',
                   'Core/Inc/motion_control.h','Core/Inc/course_mode.h','Core/Inc/task1_test.h',
                   'Core/Inc/imu_filter.h','tools/evaluate_time_trial.py']
    report={'scope':'HOST SIMULATION ONLY. Plant gain, response time and tire grip are not high-speed measurements.',
        'inputs_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in inputs},
        'profiles':[]}
    clean_env={k:v for k,v in os.environ.items() if not k.startswith('SIM_')}
    for name in args.profiles:
        definitions=PROFILES[name]
        exe=out/(name+('.exe' if os.name=='nt' else ''))
        flags=[f'-D{k}={v}'+('f' if isinstance(v,float) else '') for k,v in definitions.items()]
        subprocess.run([args.compiler,'-std=c11','-O1','-Wall','-Wextra','-Werror','-ICore/Inc',
            '-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,*SOURCES,'-lm','-o',str(exe)],
            cwd=ROOT,check=True)
        profile={'name':name,'overrides':definitions,'runs':[]}
        for model in args.models:
            for phase in (0,5,10):
                suites=('nominal','full','d-offset') if args.stress else ('nominal',)
                if args.entry_stress:suites+=('c-offset',)
                for suite in suites:
                    env={**clean_env,'SIM_PWM_PER_MM_S':str(model),'SIM_GRAY_PHASE_MS':str(phase),
                         'SIM_WEAK_BRAKE':'1','SIM_METRICS':'1'}
                    if suite=='nominal':env['SIM_NOMINAL_ONLY']='1'
                    if suite=='d-offset':env['SIM_D_ENTRY_SWEEP']='1'
                    if suite=='c-offset':env['SIM_C_ENTRY_SWEEP']='1'
                    proc=subprocess.run([str(exe)],cwd=ROOT,env=env,capture_output=True,text=True)
                    log=out/f'{name}-model{model}-phase{phase}-{suite}.log'
                    log.write_text(proc.stdout+proc.stderr)
                    times=[float(t) for t in re.findall(r'PASS: task4 ([\d.]+)s, 17 point events, 4 laps',proc.stdout)]
                    record={'assumed_pwm_per_mm_s':model,'gray_phase_ms':phase,'suite':suite,
                        'passed':proc.returncode==0,'returncode':proc.returncode,'cross_four_seconds':times,
                        'errors':proc.stderr.strip(),'log':log.name}
                    profile['runs'].append(record)
        profile['all_requested_scenarios_passed']=all(r['passed'] for r in profile['runs'])
        report['profiles'].append(profile)
        nominal=[t for r in profile['runs'] if r['suite']=='nominal' and r['passed'] for t in r['cross_four_seconds']]
        print(name,'simulation suites:',sum(r['passed'] for r in profile['runs']),'/',len(profile['runs']),
              'nominal cross4 seconds:',nominal,flush=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print('Experiment report:',args.output)
    print('Report completion is not a pass gate. Inspect each profile and each failed scenario.')

if __name__=='__main__':main()
