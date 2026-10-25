#!/usr/bin/env python3
"""Offline timestamped edge-pose experiment. No firmware build or device access.

Interpolates the black/white transition pose; propagates its position observation
to confirmation time with measured odometry. The physical simulator still uses
its independent pose for point coverage. These models are not real-run replay.
"""
import json,os,subprocess,sys,hashlib,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from evaluate_time_trial import ROOT,PROFILES,SOURCES
OUT=ROOT/'build-host/t320-edge-research'
TRACK='''
static void TrackEdgePose(CarControl *c,const CarInputs *in,bool fresh,bool was_missing)
{
    if (!CrossSprint(c) || !fresh) return;
    uint8_t band=in->black_mask;
    if (band && band!=255) {
        while (!(band&1U)) band>>=1;
        c->edge_valid=(band==1 || band==3 || band==7) && c->black_frames>=3 &&
            fabsf(c->line_error)<=1.5f*CAR_GRAY_PITCH_MM;
        c->edge_ready=false;
        c->edge_x=c->x_mm;c->edge_y=c->y_mm;c->edge_heading=c->heading_rad;
    } else if (!band && !was_missing) {
        c->edge_valid=c->edge_valid && in->now_ms-c->last_line_at<=40;
        c->edge_ready=c->edge_valid;
        c->edge_x=(c->edge_x+c->x_mm)*0.5f;
        c->edge_y=(c->edge_y+c->y_mm)*0.5f;
        c->edge_heading+=(c->heading_rad-c->edge_heading)*0.5f;
    } else if (band) {
        c->edge_valid=c->edge_ready=false;
    }
}
'''
PROPAGATE='''
        if (CrossSprint(c) && c->edge_valid && c->edge_ready) {
            float adjust=CAR_ENDPOINT_YAW_BLEND*Clamp(Motion_Wrap(observed-c->edge_heading),-0.15f,0.15f);
            float edge_heading=c->edge_heading+adjust;
            float dx=c->x_mm-c->edge_x,dy=c->y_mm-c->edge_y;
            observed_x=end.x-c->geometry.sensor_forward_mm*cosf(edge_heading)+lateral*sinf(edge_heading)
                +cosf(adjust)*dx-sinf(adjust)*dy;
            observed_y=end.y-c->geometry.sensor_forward_mm*sinf(edge_heading)-lateral*cosf(edge_heading)
                +sinf(adjust)*dx+cosf(adjust)*dy;
            observed_heading=c->heading_rad+adjust;
        }
        if (CrossSprint(c) && (!c->edge_valid || !c->edge_ready)) exit=false;
'''
def prepare():
    inc=OUT/'include';inc.mkdir(parents=True,exist_ok=True)
    for f in (ROOT/'Core/Inc').glob('*.h'):shutil.copy2(f,inc/f.name)
    p=inc/'car_control.h';s=p.read_text();s=s.replace('    CarGeometry geometry;',
        '    float edge_x,edge_y,edge_heading;\n    bool edge_valid,edge_ready;\n    CarGeometry geometry;');p.write_text(s)
    s=(ROOT/SOURCES[0]).read_text()
    s=s.replace('    c->have_line=false; c->line_missing=false;',
        '    c->edge_valid=c->edge_ready=false;\n    c->have_line=false; c->line_missing=false;')
    s=s.replace('static void Drive(',TRACK+'\nstatic void Drive(')
    s=s.replace('    Gray(c,in);','    bool was_missing=c->line_missing;\n    Gray(c,in);')
    s=s.replace('    ObserveArcPosition(c,in,fresh_gray);',
        '    ObserveArcPosition(c,in,fresh_gray);\n    TrackEdgePose(c,in,fresh_gray,was_missing);')
    s=s.replace('float heading_error=fabsf(Motion_Wrap(c->heading_rad-observed));',
        'float heading_error=fabsf(Motion_Wrap((CrossSprint(c) && c->edge_ready ? c->edge_heading : c->heading_rad)-observed));')
    s=s.replace('        float correction_limit=',PROPAGATE+'        float correction_limit=')
    p=OUT/'controller.c';p.write_text(s)
    return p
def main():
    source=prepare();exe=OUT/'edge-pose'
    flags=[f'-D{k}={v}' for k,v in PROFILES['arc-localize'].items()]
    subprocess.run(['clang','-std=c11','-O1','-Wall','-Wextra','-Werror','-I'+str(OUT/'include'),
        '-ICore/Inc','-DCAR_TASK1_TEST_BUILD=1','-DCAR_DUAL_MODE_BUILD=1',*flags,str(source),
        *SOURCES[1:],'-lm','-o',str(exe)],cwd=ROOT,check=True)
    clean={k:v for k,v in os.environ.items() if not k.startswith('SIM_')};records=[]
    for gain in (1.5,1.,.9):
        for phase in range(15):
            for suite,extra in [('nominal',{'SIM_NOMINAL_ONLY':'1'}),('full',{}),
                ('cross-full',{'SIM_TASK4_ONLY':'1'}),('c-offset',{'SIM_C_ENTRY_SWEEP':'1'}),
                ('d-offset',{'SIM_D_ENTRY_SWEEP':'1'})]:
                env={**clean,'SIM_PWM_PER_MM_S':str(gain),'SIM_GRAY_PHASE_MS':str(phase),'SIM_WEAK_BRAKE':'1',**extra}
                r=subprocess.run([str(exe)],cwd=ROOT,env=env,capture_output=True,text=True)
                log=f'g{gain}-p{phase}-{suite}.log';(OUT/log).write_text(r.stdout+r.stderr)
                records.append(dict(gain=gain,phase=phase,suite=suite,passed=r.returncode==0,errors=r.stderr,log=log))
        print(gain,sum(r['passed'] for r in records),'/',len(records),flush=True)
    report=dict(scope=__doc__,runs=records,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
if __name__=='__main__':main()
