#!/usr/bin/env python3
"""Decode T315's frozen wheel response windows. Does not connect to a device.

Speeds/targets are mm/s; integral and PWM are permille, rounded to integers.
Measured speed is the existing 30 ms filtered encoder estimate. PWM in a row is
applied after that row's measurement: do not treat it as its cause or fit Ka
without accounting for the filtering and sample timing.
"""
import argparse,csv,hashlib,json,struct
from pathlib import Path
from statistics import mean

META=struct.Struct('<9I');ROW=struct.Struct('<I8h2H4B')
FIELDS=['tick','requested_left','requested_right','setpoint_left','setpoint_right',
        'measured_left','measured_right','integral_left','integral_right',
        'pwm_left','pwm_right','state','segment','events','flags']

def decode(meta,data):
    if len(meta)!=META.size:raise ValueError('Expected 36-byte metadata')
    magic,size,capacity,n0,n1,d0,d1,done,cycles=META.unpack(meta)
    if magic!=0x53504431 or size!=ROW.size or capacity!=104 or len(data)!=2*capacity*size:
        raise ValueError('Wrong speed trace layout')
    if n0>capacity or n1>capacity or done>3:raise ValueError('Invalid counters')
    windows=[];csv_rows=[]
    for window,(count,dropped) in enumerate(((n0,d0),(n1,d1))):
        rows=[dict(zip(FIELDS,ROW.unpack_from(data,(window*capacity+i)*size))) for i in range(count)]
        gaps=[(b['tick']-a['tick'])&0xffffffff for a,b in zip(rows,rows[1:])]
        if any(g==0 or g>160000 for g in gaps):raise ValueError('Invalid timestamps')
        if rows and not rows[0]['flags']&2:raise ValueError('Missing first marker')
        complete=bool(done&(1<<window))
        if complete and (not rows or not rows[-1]['flags']&4):raise ValueError('Missing terminal marker')
        if any(r['flags']&~7 for r in rows):raise ValueError('Unknown flags')
        valid=[r for r in rows if r['state']==1 and not r['flags']&1]
        report=dict(window=window,segment=2*window,count=count,dropped=dropped,
            complete=complete,clipped_rows=sum(bool(r['flags']&1) for r in rows),
            duration_ms=((rows[-1]['tick']-rows[0]['tick'])&0xffffffff) if rows else 0,
            sample_gap_min_ms=min(gaps) if gaps else None,sample_gap_max_ms=max(gaps) if gaps else None)
        if valid:
            for side in ('left','right'):
                report[side]=dict(
                    mean_abs_target_ramp_gap_mm_s=mean(abs(r['requested_'+side]-r['setpoint_'+side]) for r in valid),
                    mean_abs_tracking_error_mm_s=mean(abs(r['setpoint_'+side]-r['measured_'+side]) for r in valid),
                    peak_pwm=max(r['pwm_'+side] for r in valid),
                    peak_requested_mm_s=max(r['requested_'+side] for r in valid),
                    peak_measured_mm_s=max(r['measured_'+side] for r in valid))
        windows.append(report);csv_rows.extend(dict(window=window,**r) for r in rows)
    return dict(scope=__doc__,windows=windows,max_record_cycles=cycles,
                max_record_us_at_72MHz=cycles/72,automatic_gain_fit=False),csv_rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('prefix',type=Path,help='Read script output prefix')
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    mp=Path(str(a.prefix)+'-speed-meta.bin');dp=Path(str(a.prefix)+'-speed-samples.bin')
    meta,data=mp.read_bytes(),dp.read_bytes();report,rows=decode(meta,data)
    report['sources_sha256']={str(mp):hashlib.sha256(meta).hexdigest(),str(dp):hashlib.sha256(data).hexdigest()}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with a.output.with_suffix('.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['window',*FIELDS]);writer.writeheader();writer.writerows(rows)
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
