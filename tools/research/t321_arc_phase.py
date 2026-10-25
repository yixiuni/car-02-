#!/usr/bin/env python3
"""Host-only bounded along-circle position prior from settled centered tracking.
The heading/tangent relationship is a model assumption, not measured true pose.
"""
import hashlib,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import evaluate_time_trial as ev
OUT=ev.ROOT/'build-host/t321-research'
PHASE='''
    /* In settled centered tracking, the axle lies on the smaller concentric
     * circle and its heading is tangent. This supplies the missing along-circle
     * constraint; never apply this prior during acquisition, white or transients. */
    if (c->distance_mm-c->segment_distance>=300.0f && c->black_frames>=4 &&
        fabsf(c->line_error)<=0.5f*CAR_GRAY_PITCH_MM && fabsf(c->line_rate)<=200.0f) {
        float sign=c->segment==1 ? 1.0f : -1.0f;
        float radius=sqrtf(400.0f*400.0f-c->geometry.sensor_forward_mm*c->geometry.sensor_forward_mm);
        float axle_radius=radius-sign*c->line_error;
        float tx=cx+sign*axle_radius*sn,ty=-400.0f-sign*axle_radius*cs;
        float sensor_x=tx+c->geometry.sensor_forward_mm*cs+c->line_error*sn;
        bool target_half=c->segment==1 ? sensor_x>=cx : sensor_x<=cx;
        float tangent_x=-dy/r,tangent_y=dx/r;
        float innovation=(tx-c->x_mm)*tangent_x+(ty-c->y_mm)*tangent_y;
        if (target_half && fabsf(innovation)<=160.0f) {
            float shift=Clamp(0.1f*innovation,-2.0f,2.0f);
            c->x_mm+=shift*tangent_x;c->y_mm+=shift*tangent_y;
        }
    }
'''
def main():
    OUT.mkdir(parents=True,exist_ok=True)
    base=OUT/'baseline.c'
    if not base.exists():
        blob=(ev.ROOT/'Core/Src/car_control.c').read_bytes()
        expected=json.loads((ev.ROOT/'docs/data/t320-validation-2026-10-25.json').read_text())['source_sha256']['Core/Src/car_control.c']
        assert hashlib.sha256(blob).hexdigest()==expected,'Need archived T320 source'
        base.write_bytes(blob)
    s=base.read_text();anchor='    c->x_mm+=step*dx/r;c->y_mm+=step*dy/r;'
    assert s.count(anchor)==1
    src=OUT/'controller.c';src.write_text(s.replace(anchor,anchor+PHASE))
    ev.SOURCES[0]=str(src.relative_to(ev.ROOT))
    ev.PROFILES={'phase-prior':ev.PROFILES['edge-pose']}
    sys.argv=[sys.argv[0],'--profiles','phase-prior','--models','1.5','1.0','.9',
        '--phases',*[str(p) for p in range(15)],'--stress','--entry-stress','--cross-full',
        '--output',str(OUT/'report.json')]
    ev.main()
if __name__=='__main__':main()
