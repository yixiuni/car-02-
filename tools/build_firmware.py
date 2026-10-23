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
a = p.parse_args()
def tool(name):
    name = 'arm-none-eabi-' + name + ('.exe' if os.name == 'nt' else '')
    found = str(a.toolchain / name) if a.toolchain else shutil.which(name)
    if not found or not Path(found).is_file():
        p.error('ARM tool not found: ' + name)
    return found
cc, objcopy, size = tool('gcc'), tool('objcopy'), tool('size')
out = root / 'build-local' / a.config
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
