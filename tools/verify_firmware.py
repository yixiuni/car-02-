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
a=p.parse_args()
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
assert 'Board_EmergencyStop' in symbols and 'CarControl_Update' in symbols
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
