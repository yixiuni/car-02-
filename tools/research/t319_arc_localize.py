#!/usr/bin/env python3
"""Test mature-arc position observations; never flashes a device."""
import json,sys,subprocess,concurrent.futures,os,hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import evaluate_time_trial as ev
ROOT=ev.ROOT;OUT=ROOT/'build-host/t319-research'

OBSERVER='''
    /* A fresh centered black sample constrains sensor position to this circle.
     * Correct position only, with bounded innovation and per-frame motion.
     * It cannot create an endpoint, change yaw or extend a loss timeout. */
    if (CrossSprint(c) && c->state==CAR_ARC && fresh_gray && c->black_frames>0 &&
        c->distance_mm-c->segment_distance>=c->geometry.sensor_forward_mm &&
        fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM) {
        float cs=cosf(c->heading_rad),sn=sinf(c->heading_rad);
        float cx=c->segment==1 ? 1000.0f : 0.0f;
        float sx=c->x_mm+c->geometry.sensor_forward_mm*cs+c->line_error*sn;
        float sy=c->y_mm+c->geometry.sensor_forward_mm*sn-c->line_error*cs;
        float dx=sx-cx,dy=sy+400.0f,r=hypotf(dx,dy);
        bool half=c->segment==1 ? dx>=0 : dx<=0;
        if (half && r>=280.0f && r<=520.0f) {
            float correction=Clamp(RESEARCH_GAIN*(400.0f-r),-RESEARCH_STEP,RESEARCH_STEP);
            c->x_mm+=correction*dx/r;c->y_mm+=correction*dy/r;
        }
    }
'''

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    if not (OUT/'baseline.c').exists():
        original=OUT/'git-base.c'
        original.write_bytes(subprocess.check_output(['git','show',
            '0764e3272f926dffaad7cb5cc00a08ed39b70293:Core/Src/car_control.c'],cwd=ROOT))
        subprocess.run(['patch','--silent','-o',str(OUT/'baseline.c'),str(original),
                        str(ROOT/'tools/research/t319-baseline.patch')],check=True)
    base=(OUT/'baseline.c').read_text()
    assert hashlib.sha256(base.encode()).hexdigest()=='1dcafa1739b7a9b1690bcd0d2f66c4df0e59d3290a5e7672ca934283d272b3c9'
    assert base.count('    Gray(c,in);')==1
    source=base.replace('    Gray(c,in);','    bool fresh_gray=in->gray_samples!=c->last_gray_samples;\n    Gray(c,in);'+OBSERVER)
    path=OUT/'observer.c';path.write_text(source)
    ev.SOURCES[0]=str(path.relative_to(ROOT))
    ev.PROFILES={f'observe-g{g}-s{s}':{**ev.PROFILES['cross-sprint'],'RESEARCH_GAIN':g,'RESEARCH_STEP':s}
                 for g,s in ((.05,1.),(.1,2.),(.2,3.))}
    sys.argv=[sys.argv[0],'--profiles',*ev.PROFILES,'--models','1.5','1.0','.9',
              '--phases','0','1','2','3','4','5','6','7','8','9','10','11','12','13','14',
              '--stress','--entry-stress','--cross-full','--output',str(OUT/'report.json')]
    ev.main()

if __name__=='__main__':main()
