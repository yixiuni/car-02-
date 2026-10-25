#!/usr/bin/env python3
"""Portable host verification, no embedded toolchain required."""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--compiler', default=shutil.which('clang') or shutil.which('gcc'))
p.add_argument('--sanitize', action='store_true')
p.add_argument('--trial-profile', action='append', help='Select trial profile(s); still runs common controller/driver tests')
a = p.parse_args()
if not a.compiler:
    p.error('Install a host C compiler or pass --compiler')
out = ROOT / 'build-host'
out.mkdir(exist_ok=True)
def run(name, sources, defines=()):
    exe = out / (name + ('.exe' if os.name == 'nt' else ''))
    cmd = [a.compiler, '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-ICore/Inc']
    if a.sanitize:
        cmd += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    subprocess.run(cmd + list(defines) + sources + ['-lm', '-o', str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)
base = ['Core/Src/car_control.c', 'Core/Src/motion_control.c', 'Core/Src/imu_filter.c']
trial_defines=['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_DUAL_MODE_BUILD=1', '-DCAR_TIME_TRIAL_BUILD=1']
trial_profiles=[('time-trial', trial_defines),
    ('time-trial2', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1']),
    ('time-trial3', trial_defines+['-DCAR_TIME_TRIAL_STAGE3=1']),
    ('time-trial-entry', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
                                      '-DCAR_TIME_TRIAL_C_ENTRY=1']),
    ('time-trial-straight', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
        '-DCAR_TIME_TRIAL_C_ENTRY=1','-DCAR_TIME_TRIAL_STRAIGHT_BOOST=1']),
    ('time-trial-fast', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
        '-DCAR_TIME_TRIAL_C_ENTRY=1','-DCAR_TIME_TRIAL_FAST=1']),
    ('time-trial-adaptive', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
        '-DCAR_TIME_TRIAL_C_ENTRY=1','-DCAR_TIME_TRIAL_FAST=1','-DCAR_TIME_TRIAL_ADAPTIVE=1']),
    ('time-trial-arc-boost', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
        '-DCAR_TIME_TRIAL_C_ENTRY=1','-DCAR_TIME_TRIAL_FAST=1','-DCAR_TIME_TRIAL_ADAPTIVE=1','-DCAR_TIME_TRIAL_ARC_BOOST=1']),
    ('time-trial-exit-trace', trial_defines+['-DCAR_TIME_TRIAL_STAGE2=1',
        '-DCAR_TIME_TRIAL_C_ENTRY=1','-DCAR_TIME_TRIAL_FAST=1','-DCAR_TIME_TRIAL_ADAPTIVE=1','-DCAR_TIME_TRIAL_EXIT_TRACE=1'])]
trial_profiles.append(('time-trial-arc-step',trial_profiles[-1][1]+['-DCAR_TIME_TRIAL_ARC_STEP=1']))
if a.trial_profile:
    unknown=set(a.trial_profile)-set(dict(trial_profiles))
    if unknown:p.error('Unknown trial profiles: '+', '.join(sorted(unknown)))
    trial_profiles=[(name,flags) for name,flags in trial_profiles if name in a.trial_profile]
for prefix, defines in trial_profiles:
    run(prefix + '-profile', base + ['tests/test_time_trial.c'], defines)
    run(prefix + '-course', base + ['Core/Src/task1_test.c', 'Core/Src/course_mode.c',
                                  'tests/sim_course.c'], defines)
    for phase in ('0', '5', '10'):
        for offset in (False, True):
            env = {**os.environ, 'SIM_GRAY_PHASE_MS': phase, 'SIM_WEAK_BRAKE': '1'}
            if offset:
                env['SIM_D_ENTRY_SWEEP'] = '1'
            exe = prefix + '-course' + ('.exe' if os.name == 'nt' else '')
            subprocess.run([str(out / exe)], cwd=ROOT, env=env, check=True)
for prefix, defines in trial_profiles:
    if '-DCAR_TIME_TRIAL_C_ENTRY=1' not in defines:continue
    run(prefix+'-c-entry',base+['tests/test_c_entry.c'],defines)
    for phase in ('0','5','10'):
        subprocess.run([str(out/(prefix+'-course'+('.exe' if os.name=='nt' else '')))],
            cwd=ROOT,env={**os.environ,'SIM_C_ENTRY_SWEEP':'1','SIM_GRAY_PHASE_MS':phase,
                          'SIM_WEAK_BRAKE':'1'},check=True)
for profile in ('time-trial-arc-boost',):
    if profile in dict(trial_profiles):
        run(profile+'-acquisition',base+['tests/test_arc_boost.c'],dict(trial_profiles)[profile])
for profile in ('time-trial-exit-trace','time-trial-arc-step'):
    if profile not in dict(trial_profiles):continue
    run(profile+'-observation',base+['tests/test_exit_trace.c'],dict(trial_profiles)[profile])
run('arc-preview', base + ['tests/test_arc_preview.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_DUAL_MODE_BUILD=1'])
run('oled-ui',['Core/Src/oled_ui.c','tests/test_oled_ui.c'])
run('gray-analog',['Core/Src/gray_analog.c','tests/test_gray_analog.c'])
subprocess.run([os.sys.executable,'tests/test_gray_calibration.py'],cwd=ROOT,check=True)
run('encoder', ['Core/Src/encoder.c', 'Core/Src/motion_control.c', 'tests/test_encoder.c'],
    ['-Itests/stubs'])
run('motor-test', ['Core/Src/motor_test.c', 'tests/test_motor_test.c'])
run('ground-test', ['Core/Src/motor_test.c', 'Core/Src/ground_test.c',
                   'Core/Src/motion_control.c', 'tests/test_ground_test.c'],
    ['-DCAR_MOTOR_TEST_BUILD=1', '-DCAR_GROUND_TEST_BUILD=1'])
run('task1-test', base + ['Core/Src/task1_test.c', 'tests/test_task1_test.c'],
    ['-DCAR_TASK1_TEST_BUILD=1'])
run('arc-gate', base + ['Core/Src/task1_test.c', 'tests/test_task1_test.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_ARC_TEST_BUILD=1'])
run('arc-course', base + ['Core/Src/task1_test.c', 'tests/sim_course.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_ARC_TEST_BUILD=1'])
run('lap-gate', base + ['Core/Src/task1_test.c', 'tests/test_task1_test.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_LAP_TEST_BUILD=1'])
run('lap-course', base + ['Core/Src/task1_test.c', 'tests/sim_course.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_LAP_TEST_BUILD=1'])
run('cross-gate', base + ['Core/Src/task1_test.c', 'tests/test_task1_test.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_CROSS_TEST_BUILD=1'])
run('cross-course', base + ['Core/Src/task1_test.c', 'tests/sim_course.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_CROSS_TEST_BUILD=1'])
run('four-gate', base + ['Core/Src/task1_test.c', 'tests/test_task1_test.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_FOUR_TEST_BUILD=1'])
run('four-course', base + ['Core/Src/task1_test.c', 'tests/sim_course.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_FOUR_TEST_BUILD=1'])
run('dual-buttons', base + ['Core/Src/task1_test.c', 'Core/Src/course_mode.c', 'tests/test_course_mode.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_DUAL_MODE_BUILD=1'])
run('dual-course', base + ['Core/Src/task1_test.c', 'Core/Src/course_mode.c', 'tests/sim_course.c'],
    ['-DCAR_TASK1_TEST_BUILD=1', '-DCAR_DUAL_MODE_BUILD=1'])
subprocess.run([str(out / ('dual-course.exe' if os.name == 'nt' else 'dual-course'))],
    cwd=ROOT, env={**os.environ, 'SIM_WEAK_BRAKE': '1'}, check=True)
subprocess.run([str(out / ('dual-course.exe' if os.name == 'nt' else 'dual-course'))],
    cwd=ROOT, env={**os.environ, 'SIM_D_ENTRY_SWEEP': '1', 'SIM_WEAK_BRAKE': '1'}, check=True)
for phase in ('5', '10'):
    subprocess.run([str(out / ('dual-course.exe' if os.name == 'nt' else 'dual-course'))],
        cwd=ROOT, env={**os.environ, 'SIM_GRAY_PHASE_MS': phase, 'SIM_WEAK_BRAKE': '1'}, check=True)
    subprocess.run([str(out / ('dual-course.exe' if os.name == 'nt' else 'dual-course'))],
        cwd=ROOT, env={**os.environ, 'SIM_GRAY_PHASE_MS': phase,
                      'SIM_D_ENTRY_SWEEP': '1', 'SIM_WEAK_BRAKE': '1'}, check=True)
subprocess.run([str(out / ('four-course.exe' if os.name == 'nt' else 'four-course'))],
    cwd=ROOT, env={**os.environ, 'SIM_SWEEP': '1', 'SIM_WEAK_BRAKE': '1'}, check=True)
run('motor-pwm', ['tests/test_motor_pwm.c'])
run('control', base + ['tests/test_car_control.c'])
run('control-zero-reference', base + ['tests/test_car_control.c'],
    ['-DCAR_GYRO_Z_BIAS_PRIOR_DPS=0.0f'])
for polarity in (0, 1):
    for reverse in (0, 1):
        run(f'gray-{polarity}-{reverse}', ['Core/Src/gray_protocol.c', 'tests/test_gray_protocol.c'],
            [f'-DCAR_GRAY_TARGET_BIT_IS_ONE={polarity}', f'-DCAR_GRAY_REVERSE_ORDER={reverse}'])
run('course', base + ['tests/sim_course.c'])
subprocess.run([os.sys.executable, 'tests/check_interfaces.py'], cwd=ROOT, check=True)
