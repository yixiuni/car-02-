#include "oled_ui.h"
#include "car_control.h"
#include "task1_test.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    char lines[8][22];
    OledView v={.task=2,.laps=1,.ready=true};
    OledUI_Lines(&v,lines);
    assert(!strcmp(lines[0],"NORMAL") && !strcmp(lines[3],"READY"));
    assert(strstr(lines[2],"LAPS 1") && strstr(lines[6],"2X LAPS"));
    v.task=4;v.laps=4;v.completed=3;v.active=true;v.elapsed_ms=63426;
    OledUI_Lines(&v,lines);
    assert(!strcmp(lines[0],"CROSS") && !lines[5][0]);
    assert(!strcmp(lines[2],"LAPS 4   DONE 3"));
    assert(!strcmp(lines[3],"RUNNING") && !strcmp(lines[4],"63.426 S"));
    assert(!strcmp(lines[6],"PRESS TO STOP") && !lines[7][0]);
    v.active=false;v.phase=TASK1_CALIBRATING;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[3],"CALIBRATING") && !strcmp(lines[6],"PRESS TO STOP"));
    v.phase=TASK1_FINISHED;v.state=CAR_DONE;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[3],"FINISHED"));
    v.elapsed_ms=29972;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[4],"29.972 S"));
    v.elapsed_ms=30005;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[4],"30.005 S"));
    v.elapsed_ms=0;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[4],"0.000 S"));
    v.state=CAR_STOPPED;v.phase=TASK1_ABORTED;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[3],"STOPPED"));
    v.fault=CAR_FAULT_LINE;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[3],"FAULT 7"));
    v.fault=0;v.phase=TASK1_IDLE;v.ready=false;OledUI_Lines(&v,lines);
    assert(!strcmp(lines[3],"WAIT SENSORS"));
    struct { uint8_t before[8],frame[1024],after[8]; } b;
    memset(&b,0xa5,sizeof(b));
    v.elapsed_ms=UINT32_MAX;v.laps=255;v.completed=255;
    OledUI_Render(&v,b.frame);
    for(unsigned i=0;i<8;i++) assert(b.before[i]==0xa5 && b.after[i]==0xa5);
    unsigned pixels=0;for(unsigned i=0;i<1024;i++) pixels+=b.frame[i]!=0;
    assert(pixels>100);
    uint8_t same[1024];OledUI_Render(&v,same);
    assert(!memcmp(same,b.frame,1024));
    v.task=2;OledUI_Render(&v,same);assert(memcmp(same,b.frame,1024));
    /* Enlarged N: 0x7f -> 0x3fff, 0x04 -> 0x0030, two identical columns. */
    assert(same[0]==0xff && same[1]==0xff && same[128]==0x3f && same[129]==0x3f);
    assert(same[2]==0x30 && same[3]==0x30 && same[130]==0 && same[131]==0);
    v.elapsed_ms=29972;OledUI_Render(&v,same);
    /* First column of enlarged '2': 0x42 expands to 0x300c on pages 4/5. */
    assert(same[4*128]==0x0c && same[4*128+1]==0x0c);
    assert(same[5*128]==0x30 && same[5*128+1]==0x30);
    puts("PASS: OLED route/laps/status/time, busy controls, frame bounds and deterministic rendering");
}
