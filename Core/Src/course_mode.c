#include "course_mode.h"

enum { NO_ACTION, START, ROUTE, LAPS };
void CourseMode_Init(CourseMode *m, CarControl *c)
{
    *m=(CourseMode){.oval_laps=1,.cross_laps=4,.feedback_pulses=1};
    c->task=2;c->target_laps=1;++c->mode_events;
}
static unsigned Gesture(CourseMode *m,uint32_t now,bool pressed,bool enabled)
{
    if(!enabled) { m->armed=m->candidate=m->click_pending=false; }
    if(!m->initialized) {
        m->initialized=true;m->raw=m->stable=pressed;m->changed_at=now;
        return NO_ACTION;
    }
    if(pressed!=m->raw) { m->raw=pressed;m->changed_at=now; }
    if(now-m->changed_at<25U) return NO_ACTION;
    if(m->stable!=pressed) {
        m->stable=pressed;
        if(pressed) {
            m->candidate=m->armed && enabled;m->armed=false;m->pressed_at=now;
        } else {
            bool click=m->candidate && enabled && now-m->pressed_at<800U;
            m->candidate=false;m->armed=enabled;
            if(click) {
                if(m->click_pending && now-m->click_at<=350U) {
                    m->click_pending=false;return LAPS;
                }
                m->click_pending=true;m->click_at=now;
            }
        }
    }
    if(!pressed && enabled) m->armed=true;
    if(pressed && m->candidate && now-m->pressed_at>=800U) {
        m->candidate=false;m->click_pending=false;return ROUTE;
    }
    if(!pressed && m->click_pending && now-m->click_at>=350U) {
        m->click_pending=false;return START;
    }
    return NO_ACTION;
}
void CourseMode_Apply(CourseMode *m,Task1Test *g,CarInputs *in,CarControl *c)
{
    bool parked=!CarControl_Active(c) &&
        (g->phase==TASK1_IDLE || g->phase==TASK1_FINISHED || g->phase==TASK1_ABORTED);
    uint32_t request=0;
    if(parked) {
        unsigned action=Gesture(m,in->now_ms,in->pressed,in->enabled);
        if(action==ROUTE) c->task=c->task==2 ? 4 : 2;
        uint8_t *laps=c->task==2 ? &m->oval_laps : &m->cross_laps;
        if(action==LAPS) *laps=*laps==1U ? 4U : 1U;
        c->target_laps=*laps;
        if(action==ROUTE || action==LAPS) {
            m->feedback_pulses=action==LAPS ? *laps : c->task==2 ? 1 : 2;
            ++c->mode_events;
        }
        if(action==START) {
            uint8_t task=c->task;
            uint32_t events=c->mode_events;
            CarGeometry geometry=c->geometry;
            CarControl_Init(c,&geometry,in->now_ms,false);
            c->task=task;c->target_laps=*laps;c->mode_events=events;
            *g=(Task1Test){0};request=1;
        }
        /* Gesture recognition owns parked input; do not let the underlying
         * one-shot gate interpret the first click of a double click as start. */
        in->pressed=false;
    } else {
        m->initialized=m->armed=m->candidate=m->click_pending=false;
    }
    Task1Test_Apply(g,in,c->state,request);
}
