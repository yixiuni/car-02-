#!/usr/bin/env python3
"""Check generated BIN/HEX and the RCT6 reset/UART4 vectors against ELF symbols."""
import argparse
from pathlib import Path
import re
import struct
import subprocess

root=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser()
p.add_argument('directory', type=Path)
p.add_argument('--nm', default='arm-none-eabi-nm')
p.add_argument('--motor-test', action='store_true')
p.add_argument('--ground-test', action='store_true')
p.add_argument('--task1-test', action='store_true')
p.add_argument('--dual-mode', action='store_true')
p.add_argument('--four-test', action='store_true')
p.add_argument('--cross-test', action='store_true')
p.add_argument('--lap-test', action='store_true')
p.add_argument('--arc-test', action='store_true')
p.add_argument('--gray-learn', action='store_true')
a=p.parse_args()
assert sum((a.arc_test,a.lap_test,a.cross_test,a.four_test,a.dual_mode))<=1
if a.arc_test or a.lap_test or a.cross_test or a.four_test or a.dual_mode: a.task1_test=True
if a.ground_test: a.motor_test=True
assert not (a.task1_test and a.motor_test)
assert not (a.gray_learn and (a.task1_test or a.motor_test))
blob=(a.directory/'stm32-car.bin').read_bytes()
assert 8 <= len(blob) <= 256*1024
sp, reset=struct.unpack_from('<II',blob)
assert sp==0x2000C000, f'Wrong RCT6 initial stack: {sp:#x}'
assert reset&1 and 0x08000000 <= (reset&~1) < 0x08000000+len(blob)
text=subprocess.check_output([a.nm,'--defined-only',str(a.directory/'stm32-car.elf')],text=True)
symbols={parts[2]:int(parts[0],16) for line in text.splitlines()
         if len(parts:=line.split())==3 and re.fullmatch('[0-9a-fA-F]+',parts[0])}
assert reset==(symbols['Reset_Handler']|1)
device=(root/'Drivers/CMSIS/Device/ST/STM32F1xx/Include/stm32f103xe.h').read_text()
irq=int(re.search(r'UART4_IRQn\s*=\s*(\d+)',device)[1])
uart,=struct.unpack_from('<I',blob,(16+irq)*4)
assert uart==(symbols['UART4_IRQHandler']|1)
assert 'Board_EmergencyStop' in symbols
if a.gray_learn:
    assert all(n in symbols for n in ('GrayLearn_Run','GrayAnalog_Decode','gray_learn_diagnostics','gray_learn_samples'))
    assert all(n not in symbols for n in ('CarControl_Update','MotorTest_Run','Task1Test_Apply','Board_MotorApply'))
elif a.motor_test:
    assert all(name in symbols for name in ('MotorTest_Run', 'MotorTest_Update',
                                            'motor_test_request', 'motor_test_diagnostics'))
    assert 'CarControl_Update' not in symbols, 'Course controller included in bench firmware'
    if a.ground_test:
        assert 'GroundTest_Feedback' in symbols and 'ground_test_fault' in symbols
    else:
        assert 'GroundTest_Feedback' not in symbols
else:
    assert 'CarControl_Update' in symbols
    assert 'MotorTest_Run' not in symbols and 'motor_test_request' not in symbols
    assert 'GroundTest_Feedback' not in symbols
if not a.gray_learn:
    assert 'GrayLearn_Run' not in symbols and 'gray_learn_diagnostics' not in symbols
if a.task1_test:
    assert all(name in symbols for name in ('Task1Test_Apply','task1_test_request',
                                            'task1_test_trace','task1_test_phase'))
else:
    assert 'Task1Test_Apply' not in symbols and 'task1_test_request' not in symbols
if a.dual_mode:
    assert 'course_run_metrics' in symbols
    assert all(n in symbols for n in ('CourseMode_Apply','CourseMode_Init','course_run_settings','Oled_Init','Oled_Poll','OledUI_Render','oled_diagnostics'))
else:
    assert 'CourseMode_Apply' not in symbols
base=0; count=0
for line in (a.directory/'stm32-car.hex').read_text().splitlines():
    assert line.startswith(':')
    rec=bytes.fromhex(line[1:]); assert sum(rec)%256==0 and len(rec)==rec[0]+5
    address=int.from_bytes(rec[1:3],'big'); kind=rec[3]; data=rec[4:-1]
    if kind==4: base=int.from_bytes(data,'big')<<16
    if kind==0:
        offset=base+address-0x08000000
        assert offset>=0 and blob[offset:offset+len(data)]==data
        count+=len(data)
assert count>1000
print(f'PASS: {a.directory.name} BIN={len(blob)} B, SP={sp:#010x}, reset={reset:#010x}, UART4={uart:#010x}, HEX checksums/data')
