#!/usr/bin/env python3
"""Build the same application/HAL sources as CMake using ARM GCC directly.
Useful on machines with Python + ARM GCC but without CMake/Ninja. No flashing.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser()
p.add_argument('--toolchain', type=Path, help='Directory containing arm-none-eabi-gcc')
p.add_argument('--config', choices=['Debug', 'Release'], default='Debug')
p.add_argument('--motor-test', action='store_true', help='Separate one-shot raised-wheel diagnostic firmware')
p.add_argument('--ground-test', action='store_true', help='Separate fixed 200/500/1000 mm ground diagnostic')
p.add_argument('--task1-test', action='store_true', help='One-shot real A-to-B controller, sensors and signals')
p.add_argument('--dual-mode', action='store_true')
p.add_argument('--time-trial', action='store_true', help='Separate experimental T301 speed-planned dual-mode image')
p.add_argument('--four-test', action='store_true', help='One-shot task4 four crossing laps, 160 s guard')
p.add_argument('--cross-test', action='store_true', help='One-shot task 3 crossing lap, 40 s limit')
p.add_argument('--lap-test', action='store_true', help='One-shot full task 2 lap, 30 s limit')
p.add_argument('--arc-test', action='store_true', help='One-shot A-B-right-arc-C stop, 25 s limit')
p.add_argument('--gray-learn', action='store_true', help='No-motion analog grayscale capture')
p.add_argument('--motor-slow-decay', action='store_true', help='Bench-only alternate AT8236 PWM mode')
p.add_argument('--motor-fast-decay', action='store_true', help='Bench comparison with original fast-decay PWM')
a = p.parse_args()
if a.time_trial:
    a.dual_mode=True
if sum((a.arc_test,a.lap_test,a.cross_test,a.four_test,a.dual_mode))>1: p.error('Choose one course diagnostic')
if a.arc_test or a.lap_test or a.cross_test or a.four_test or a.dual_mode:
    if a.task1_test: p.error('--arc-test and --task1-test are separate images')
    a.task1_test=True
if a.gray_learn and (a.task1_test or a.motor_test or a.ground_test or a.motor_fast_decay or a.motor_slow_decay):
    p.error('--gray-learn cannot combine with motion diagnostics')
if a.task1_test and (a.motor_test or a.ground_test or a.motor_fast_decay or a.motor_slow_decay):
    p.error('--task1-test cannot combine with motor diagnostics')
if a.ground_test:
    if a.motor_test or a.motor_fast_decay or a.motor_slow_decay:
        p.error('--ground-test cannot combine with bench switches')
    a.motor_test=True
if a.motor_slow_decay and not a.motor_test:
    p.error('--motor-slow-decay requires --motor-test')
if a.motor_fast_decay and (not a.motor_test or a.motor_slow_decay):
    p.error('--motor-fast-decay requires --motor-test and cannot combine with --motor-slow-decay')
def tool(name):
    name = 'arm-none-eabi-' + name + ('.exe' if os.name == 'nt' else '')
    found = str(a.toolchain / name) if a.toolchain else shutil.which(name)
    if not found or not Path(found).is_file():
        p.error('ARM tool not found: ' + name)
    return found
cc, objcopy, size = tool('gcc'), tool('objcopy'), tool('size')
out = root / 'build-local' / (('GrayLearn' if a.gray_learn else 'TimeTrial' if a.time_trial else 'DualMode' if a.dual_mode else 'FourTest' if a.four_test else 'CrossTest' if a.cross_test else 'LapTest' if a.lap_test else 'ArcTest' if a.arc_test else 'Task1Test' if a.task1_test else 'GroundTest' if a.ground_test else 'MotorTestFast' if a.motor_fast_decay else 'MotorTestSlow' if a.motor_slow_decay else 'MotorTest' if a.motor_test else '') + a.config)
out.mkdir(parents=True, exist_ok=True)
includes = ['Core/Inc', 'Drivers/STM32F1xx_HAL_Driver/Inc',
            'Drivers/STM32F1xx_HAL_Driver/Inc/Legacy', 'Drivers/CMSIS/Device/ST/STM32F1xx/Include',
            'Drivers/CMSIS/Include']
# Read CMake's explicit source lists so this fallback cannot silently omit a
# newly added application source. The files list no generator expressions for C.
import re
texts = [(root/'CMakeLists.txt').read_text(), (root/'cmake/stm32cubemx/CMakeLists.txt').read_text()]
sources = []
for text in texts:
    for path in re.findall(r'(?:\$\{CMAKE_CURRENT_SOURCE_DIR\}/../../)?((?:Core|Drivers)/[\w/.-]+\.c|startup_stm32f103xe\.s)', text):
        if path not in sources:
            sources.append(path)
flags = ['-mcpu=cortex-m3', '-mthumb', '-std=gnu11', '-DUSE_HAL_DRIVER', '-DSTM32F103xE',
         '-Wall', '-Wextra', '-ffunction-sections', '-fdata-sections', '-fstack-usage']
flags += ['-O0', '-g3', '-DDEBUG'] if a.config=='Debug' else ['-Os', '-g0']
if a.motor_test:
    flags += ['-DCAR_MOTOR_TEST_BUILD=1']
if a.ground_test:
    flags += ['-DCAR_GROUND_TEST_BUILD=1']
if a.task1_test:
    flags += ['-DCAR_TASK1_TEST_BUILD=1']
if a.dual_mode:
    flags += ['-DCAR_DUAL_MODE_BUILD=1']
if a.time_trial:
    flags += ['-DCAR_TIME_TRIAL_BUILD=1']
if a.four_test:
    flags += ['-DCAR_FOUR_TEST_BUILD=1']
if a.cross_test:
    flags += ['-DCAR_CROSS_TEST_BUILD=1']
if a.lap_test:
    flags += ['-DCAR_LAP_TEST_BUILD=1']
if a.arc_test:
    flags += ['-DCAR_ARC_TEST_BUILD=1']
if a.gray_learn:
    flags += ['-DCAR_GRAY_LEARN_BUILD=1']
if a.motor_slow_decay:
    flags += ['-DCAR_MOTOR_SLOW_DECAY=1']
if a.motor_fast_decay:
    flags += ['-DCAR_MOTOR_SLOW_DECAY=0']
flags += ['-I'+x for x in includes]
objects = []
for source in sources:
    obj = out / (source.replace('/', '_') + '.o')
    subprocess.run([cc, *flags, '-c', source, '-o', str(obj)], cwd=root, check=True)
    objects.append(str(obj))
elf = out/'stm32-car.elf'
subprocess.run([cc, '-mcpu=cortex-m3', '-mthumb', *objects,
    '-TSTM32F103RCTx_FLASH.ld', '--specs=nano.specs', '-Wl,--gc-sections',
    '-Wl,--print-memory-usage', '-Wl,-Map='+str(out/'stm32-car.map'), '-lm', '-o', str(elf)], cwd=root, check=True)
for fmt, ext in [('ihex','hex'), ('binary','bin')]:
    subprocess.run([objcopy, '-O', fmt, str(elf), str(out/('stm32-car.'+ext))], check=True)
subprocess.run([size, str(elf)], check=True)
print('Built', elf)
