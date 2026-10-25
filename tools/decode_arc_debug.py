#!/usr/bin/env python3
"""Decode T311's eight read-only arc-exit records (116 bytes each)."""
import argparse,json,struct
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('capture',type=Path);a=p.parse_args()
data=a.capture.read_bytes()
if len(data)!=928:p.error('Expected 928-byte T311 arc-debug capture')
bits={1:'no_line',4:'stale',8:'lateral',32:'turn',64:'heading',128:'distance',256:'correction',512:'footprint'}
keys=['tick','rejected','x_mm','y_mm','heading_rad','progress_mm','turned_rad','heading_error_rad','correction_mm','line_error_mm','white_frames','line_age_ms']
rows=[]
for slot in range(8):
    offset=116*slot
    values=struct.unpack_from('<3I2f',data,offset)
    row=dict(zip(['point_events','black_samples','exit_attempts','last_radial_error_mm','max_abs_radial_error_mm'],values));row['slot']=slot
    for name,delta in [('first',20),('last',68)]:
        sample=dict(zip(keys,struct.unpack_from('<2I8f2I',data,offset+delta)))
        sample['reasons']=[name for bit,name in bits.items() if sample['rejected']&bit]
        row[name]=sample
    rows.append(row)
print(json.dumps({'scope':'Observations only; records require white-frame and progress gates first. Zero attempts means no qualifying candidate, not acceptance.','arcs':rows},indent=2))
