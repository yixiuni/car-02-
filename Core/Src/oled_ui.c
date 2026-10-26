#include "oled_ui.h"
#include "car_control.h"
#include "task1_test.h"
#include <stdio.h>
#include <string.h>

/* 5-column, least-significant bit at the top; digits then A..Z. */
static const uint8_t glyphs[][5]={
 {0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},
 {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
 {0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
 {0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},
 {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e},
 {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},
 {0x3e,0x41,0x41,0x41,0x22},{0x7f,0x41,0x41,0x22,0x1c},
 {0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},
 {0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},
 {0,0x41,0x7f,0x41,0},{0x20,0x40,0x41,0x3f,1},
 {0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
 {0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},
 {0x3e,0x41,0x41,0x41,0x3e},{0x7f,9,9,9,6},
 {0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},
 {0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},
 {0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},
 {0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,8,0x14,0x63},
 {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43}
};
void OledUI_Lines(const OledView *v,char lines[8][22])
{
    memset(lines,0,8U*22U);
    snprintf(lines[0],22,"%s",v->task==2 ? "NORMAL" : "CROSS");
    snprintf(lines[2],22,"LAPS %u   DONE %u",v->laps,v->completed);
    const char *status=v->active ? "RUNNING" : v->state==CAR_DONE ? "FINISHED" :
        v->phase==TASK1_ABORTED ? "STOPPED" :
        (v->phase==TASK1_CALIBRATING || v->phase==TASK1_PRESS) ? "CALIBRATING" :
        v->ready ? "READY" : "WAIT SENSORS";
    if(v->fault) snprintf(lines[3],22,"FAULT %u",v->fault);
    else snprintf(lines[3],22,"%s",status);
    /* Two pages for a readable live/frozen result, with millisecond precision.
     * Route remains identified by the NORMAL/CROSS title above. */
    snprintf(lines[4],22,"%lu.%03lu S",(unsigned long)(v->elapsed_ms/1000U),
             (unsigned long)(v->elapsed_ms%1000U));
    bool busy=v->active || v->phase==TASK1_CALIBRATING || v->phase==TASK1_PRESS;
    snprintf(lines[6],22,"%s",busy ? "PRESS TO STOP" : "HOLD ROUTE  2X LAPS");
    snprintf(lines[7],22,"%s",busy ? "" : "CLICK TO START");
}
static uint8_t Column(char ch,unsigned col)
{
    if(ch>='0' && ch<='9') return glyphs[ch-'0'][col];
    if(ch>='A' && ch<='Z') return glyphs[10+ch-'A'][col];
    if(ch=='-') return 8;
    if(ch=='.') return col==2 ? 0x40 : 0;
    return 0;
}
void OledUI_Render(const OledView *v,uint8_t frame[1024])
{
    char lines[8][22]; OledUI_Lines(v,lines);
    memset(frame,0,1024);
    for(unsigned row=0;row<8;row++) {
        unsigned scale=(row==0 || (row==4 && strlen(lines[row])<=10U)) ? 2 : 1;
        for(unsigned ch=0;lines[row][ch] && (ch*6U+5U)*scale<=128;ch++)
            for(unsigned col=0;col<5;col++) {
                uint8_t bits=Column(lines[row][ch],col);
                unsigned x=(ch*6+col)*scale;
                if(scale==1) frame[row*128+x]=bits;
                else {
                    /* Spread bits into even positions, then duplicate vertically.
                     * Write whole columns instead of drawing individual pixels. */
                    uint16_t wide=bits;
                    wide=(wide|(wide<<4))&0x0f0fU;
                    wide=(wide|(wide<<2))&0x3333U;
                    wide=(wide|(wide<<1))&0x5555U;
                    wide|=wide<<1;
                    frame[row*128+x]=frame[row*128+x+1]=(uint8_t)wide;
                    frame[(row+1)*128+x]=frame[(row+1)*128+x+1]=(uint8_t)(wide>>8);
                }
            }
    }
}
